# 多智能体拥堵解决方案 — 修复与实现计划

## 总体策略

使用 **CBS（Conflict-Based Search，基于冲突的搜索）** 作为响应式集中重规划器，由 server 在检测到拥堵时触发，替代当前已损坏的 MA-RRT* 和 iNash。CBS 复用现有的 `gridmap.py` 和 A* 基础设施，在网格地图上保证完备性（有解必找到）。

---

## 第一步：修复 PREQUEUE 永久卡死 Bug

### 问题描述

`agents/agent_state_machine.py` 第 98-100 行：当 `port.request_enter_permit()` 返回 `False`（端口无空位），agent 进入 PREQUEUE 状态后没有任何退出机制。PREQUEUE 不是 mobile state（第 31 行只有 CRUISE 是 mobile），所以 agent 既不能移动，也无法重新触发 `approaching()`，永远卡在 PREQUEUE。

### 如何确认是这里出错

1. 运行 `python simulator.py --agent 10 --port 2 2`（agent 多、端口少，容易触发）
2. 观察终端输出或在 `approaching()` 函数入口加一行 `print`，确认 agent 进入 PREQUEUE 后不再打印任何状态变化
3. 在 `agent_state_machine.py:98` 加断点或 print，确认 `request_enter_permit()` 持续返回 `False`
4. 观察 pygame 窗口：有 agent 停在端口附近一动不动，其他 agent 正常运行

### 修复方案

在 PREQUEUE 状态下增加步数计数器。超过阈值（如 300 步 = 5 秒）后：
- agent 退出端口控制区，移动到附近的等待位置（holding position）
- 向 server 重新申请任务（可能分配到不同端口）

**需要修改的文件：**
- [agents/agent_state_machine.py](../agents/agent_state_machine.py) — 在 PREQUEUE 分支加计数器和退出逻辑
- [agents/agent.py](../agents/agent.py) — 增加 `prequeue_wait_steps` 计数器属性

---

## 第二步：在 server 中增加拥堵检测器

### 问题描述

当前系统没有任何拥堵检测机制。LayeredAStar 的 `observe_path()`（`layered_astar_planner.py:128`）只检测"附近 agent 超过 4 个"就触发重规划，这是单 agent 视角的局部判断，无法检测多 agent 互相阻塞的全局死锁。

### 如何确认是这里出错

1. 在 `server.py` 的主循环或 `step()` 中加日志，打印每个 agent 的位置
2. 运行模拟器 2 分钟，观察是否有 agent 位置长时间不变（位移 < 0.1m 超过 60 步）
3. 对比 `agent.state`：如果 state 是 CRUISE 但位置不变，说明 agent 认为自己在运动但实际被堵死

### 实现方案

在 `server.py` 中增加 `CongestionDetector` 类：
- 每步记录每个 agent 的位置
- 滑动窗口（K=60 步）内位移 < ε（0.1m）且 agent 有目标地点 → 标记为拥堵
- 拥堵 agent 数量 ≥ 2 → 触发 CBS 重规划

**需要修改的文件：**
- [server.py](../server.py) — 增加 `CongestionDetector`，在 `step()` 或被 simulator 调用的位置插入检测逻辑

---

## 第三步：实现 CBS 规划器

### 问题描述

MA-RRT*（`marrtstar_planner.py`）在 joint-space 搜索，复杂度随 agent 数量指数增长，超时 60 秒，且源码 TODO 注释自述"结果很丑"。iNash（`inash_planner.py:315`）有 `NotImplementedError`，local planner steering 路径未完成。两者都不可靠。

### 如何确认是这里出错

1. 运行 `python simulator.py --gp RRTStar --agent 6`，观察终端：MA-RRT* 会打印 `"MARRT* DONE in X sec"`，X 经常接近 60 秒
2. 运行 iNash：触发 local planner steering 分支时会抛出 `NotImplementedError`（`inash_planner.py:315`）
3. 在 agent 数量 > 6 时，两个规划器都会在超时前找不到解，返回空路径，agent 停止运动

### 实现方案

新建 `global_planners/cbs_planner.py`，实现两层结构：

**低层：Space-Time A\***
- 在现有 `sample_global_planner.py` 的 A* 基础上增加时间维度
- 搜索节点：`(grid_x, grid_y, timestep)`
- 约束：`(agent_id, grid_pos, timestep)` — 该 agent 在该时刻不能在该位置
- 复用 `gridmap.py` 的邻居查询和障碍物检查

**高层：CBS 约束树**
- 每个节点包含：约束集合 + 每个 agent 的当前路径
- 冲突检测：遍历所有 agent 路径，找到第一个时空冲突（两个 agent 在同一时刻占同一格）
- 分支：对冲突的两个 agent 分别加约束，生成两个子节点
- 终止：所有 agent 路径无冲突

**需要新建的文件：**
- [global_planners/cbs_planner.py](../global_planners/cbs_planner.py) — CBS 完整实现

**需要修改的文件：**
- [server.py](../server.py) — 将 CBS 注册为 multiagent planner，在拥堵检测触发时调用

---

## 第四步：将 CBS 接入 server 的调度流程

### 问题描述

`server.py:51-58` 的 `request_multiagent_global_planner_compute_path()` 目前只是把请求转发给已注册的 multiagent planner（MA-RRT* 或 iNash）。没有拥堵触发逻辑，也没有 fallback。

### 实现方案

修改 `server.py`：
1. 每个 simulator step 调用 `CongestionDetector.update(agents)`
2. 检测到拥堵时，收集被堵 agent 的当前位置和目标（复用现有 `collect_agents_info()`）
3. 调用 CBS 规划器计算无冲突路径
4. 调用现有 `refresh_agents_path()` 将新路径分发给 agent

**需要修改的文件：**
- [server.py](../server.py)

---

## 修改文件汇总

| 文件 | 修改类型 | 原因 |
|------|----------|------|
| [agents/agent_state_machine.py](../agents/agent_state_machine.py) | Bug 修复 | PREQUEUE 无退出机制，agent 永久卡死 |
| [agents/agent.py](../agents/agent.py) | 小改 | 增加 `prequeue_wait_steps` 计数器属性 |
| [server.py](../server.py) | 功能增加 | 增加拥堵检测器 + CBS 调度逻辑 |
| [global_planners/cbs_planner.py](../global_planners/cbs_planner.py) | 新建 | CBS 规划器实现（Space-Time A* + 约束树） |

---

## 验证方法（按顺序）

1. **PREQUEUE 修复验证：** 运行 `python simulator.py --agent 10 --port 2 2`，观察 10 分钟内所有 agent 都能持续完成任务，无 agent 永久停止在端口附近。

2. **拥堵检测验证：** 在 `CongestionDetector` 触发时打印日志，确认在明显拥堵场景下（多 agent 挤在同一区域）能在 5 秒内检测到。

3. **CBS 单元测试：** 构造一个 2-agent 对向行走场景（head-on），验证 CBS 输出的路径无时空冲突，且两个 agent 都能到达目标。

4. **集成验证：** 运行 `python simulator.py -t 60 --agent 10`，对比加入 CBS 前后的 PPH（packages per hour）指标，CBS 介入后 PPH 应提升或至少不下降。
