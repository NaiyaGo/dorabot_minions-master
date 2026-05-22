# Dorabot Minions 多机器人路径规划系统研究与改进方案

## 1. 项目背景条件

### 1.1 运动学模型
- **Agent 类型**：全向（holonomic）移动机器人，无差速/阿克曼约束
- **速度控制**：`linearVelocity = (vx, vy)` 直接赋值到 Box2D 刚体，任意方向瞬时切换
- **朝向**：由速度方向决定（`angle = atan2(vy, vx)`），无独立转向动作
- **无离散动作原语**：不存在 stop / spin±90° / forward 这类离散动作集
- **cruise_speed**：1.5 m/s；**TIME_STEP**：1/60 s ≈ 16.67 ms

### 1.2 物理仿真约束（关键背景）
- **速度不是恒定的**：Box2D 物理引擎下，加速、减速、绕障、拐弯都会导致实际速度变化
- **不存在"一 timestep 走一格"的保证**：
  - 切换控制模式那一刻，agent 带着原有惯性速度，不可能立刻达到预期 cruise_speed
  - 通过同一格子可能用 5 tick（直线冲刺）或 15 tick（拐弯减速），但离散规划器都标为"1 timestep"
  - 加速/减速段、拐角段与直线段的物理耗时完全不同
- **仿真时间是连续的**：`simulator.step()` 每步固定推进 1/60 s，但 agent 走过多少空间取决于当前速度

### 1.3 地图与坐标系
- **环境**：2D 仓库/工厂场景，walls / ports / obstacles 为静态 Box2D 刚体
- **网格表示**：`GridmapWithNeighbors`，resolution 由命令行传入（`--resolution 2` 表示每米 2 格，边长 0.5 m）
- **路径点坐标**：`grid_to_float(centroid=False)` 返回格子左下角坐标（路径点位于通道内，不在墙里）

### 1.4 规划—执行架构
- **两层规划**：global_planner（ECBS）给 waypoint 序列 → local_planner（VirtualForce）消费并输出速度
- **共享全局规划器**：所有 agent 共享一个 `ECBSPlanner` 实例，通过 `MultiAgentPlannerLocalEntry` 代理访问
- **全局规划按需触发**：`goal_changed=True` 或 `replan=True` 时才调用，不是每步

---

## 2. 当前方案

### 2.1 常态方案
| 组件 | 选择 | 行为 |
|------|------|------|
| Global Planner | `ECBSPlanner` | 所有 agent 共享，多机器人 Space-Time A* + focal 堆，次优因子 w=1.3 |
| Local Planner | `VirtualForcePlanner`（标准模式） | 滑动窗口选 2m 内最远路径点为目标 + 合力（吸引 + 排斥） |
| 触发 | 新任务或 replan | 每次只算一次，存入 `sequence_of_poses` 复用 |

### 2.2 困境方案
| 组件 | 选择 | 行为 |
|------|------|------|
| 检测 | `CongestionDetector` | 60 步滑动窗口，位移² < 0.01（0.1m）即判定拥堵 |
| 触发 | ≥ 2 个拥堵 agent 且冷却结束 | 创建新 `CBSPlanner` 实例（只覆盖拥堵 agent） |
| Global Planner | `CBSPlanner` | 同步阻塞，节点上限 500，时间上限 300 timestep |
| Local Planner | `VirtualForcePlanner`（strict 模式） | 严格按序 `popleft`，目标点取 `path[0]`，保留合力 |
| 恢复 | 路径走完 | `strict_path=False`, `goal_changed=True` 触发 ECBS 重规划 |
| 冷却 | 120 步（2s） | 防止连续触发 |

---

## 3. 存在的问题

### 3.1 时间模型脱节（最根本的问题）
- **ECBS 的 t 是离散拓扑时间，不是物理时间**
- 约束 `(gx, gy, t)` 语义："第 t 个 timestep 不能占这个格子"，但"第 t 个 timestep"对应的物理时刻取决于每个 agent 的实际速度曲线
- **现实情况**：agent 会加减速、会在拐点慢下来、切换瞬间带惯性速度
- **后果**：ECBS 的时间冲突避免（vertex/edge conflict at time t）**在执行层无法保证**
- **结论**：ECBS 的输出只能当作"空间路径建议"，不是"时空轨迹"

### 3.2 切换入口瞬态未处理
- CBS 触发那一瞬间，agent 仍带着原有速度和方向（通常不是朝新路径的方向）
- strict 模式直接切入 → agent 先冲出去一段才能被吸引力拉回来
- 没有"先停下 → 再按新路径启动"的过渡

### 3.3 strict 模式下的排斥力矛盾
- CBS 保证**组内**路径空间无冲突（同格不同时）
- 但 VirtualForce strict 分支保留了**所有**排斥力（组内 agent + 组外 agent + port）
- 组内 agent 在窄道并行时互相排斥 → 把彼此推离 CBS 路径 → 撞墙
- 完全关闭排斥也不行，因为时间模型脱节，组内 agent 实际相遇时仍可能冲突

### 3.4 规划阻塞与异步
- `cbs.compute_path()` 在 `server.tick()` 内同步调用
- 最坏情况 500 × 300 次节点评估，agent 多时明显卡顿
- 规划期间其他 agent 继续运动 → 规划完成时位置快照已过时

### 3.5 失败降级不完整
- 节点超限时 CBS 返回"最低代价部分解"，可能**仍含冲突**
- 当前代码没检测返回的路径是否真的无冲突，直接分发给 agent
- 没有回退机制（比如回退到 ECBS 全量重算、暂停部分 agent 等）

### 3.6 arrive 阈值与格子大小不匹配
- `Point.arrive()` 硬编码 0.2m 阈值
- resolution=2 时格子 0.5m，cruise_speed=1.5 m/s，每 tick 走 0.025m
- 阈值占格子 40%，高速过冲时容易一步跨过多个 waypoint，或者错过单个 waypoint 导致死磕

### 3.7 DullPlanner / GridStepPlanner 不适用
- DullPlanner 纯吸引力 + 忽略入口速度 → 实测撞墙
- GridStepPlanner 强制四方向移动，但本项目是全向模型，强行离散化运动反而增加延迟和死锁

---

## 4. 改进方案

### 4.1 Global Planner：保留 ECBS，统一去掉 CBSPlanner

**改动 A：常态 + 困境统一使用 ECBSPlanner**

理由：
- ECBS = CBS + focal 堆优化，CBSPlanner 是 ECBS 的退化版，没必要维护两份
- ECBS 的次优因子 w 可调，困境下可临时放宽
- 时间脱节让"严格无冲突"本来就不现实，ECBS 的软约束更符合实际

实现要点：
```
server.tick() 困境分支:
  不再新建 CBSPlanner
  直接复用已有的 ECBSPlanner 实例（从 self.multiagent_global_planners 获取）
  临时调高 SUBOPTIMAL_FACTOR 到 2.0，换取更快收敛
  调用 compute_path() 只对 congested agent 子集规划
  返回后恢复 SUBOPTIMAL_FACTOR=1.3
```

**改动 B：把输出当空间路径，丢弃 t 维度**

```
ECBSPlanner._to_point_paths():
  只保留 (gx, gy) 序列转 Point
  不输出 timestep 信息
  执行层按物理连续时间跟随，不对齐 ECBS 的 t
```

**改动 C：部分解冲突检测与降级**

```
compute_path() 返回前:
  若 nodes_expanded == MAX_CBS_NODES:
    调用 _find_first_conflict(best.paths)
    若仍有冲突:
      print 日志，标记 solution_status = "partial"
  server.tick() 收到 partial 状态:
    不切换 strict_path，保留 VirtualForce 常态
    让反应式方法自己慢慢化解
```

### 4.2 Local Planner：双模式 VirtualForcePlanner（保留全向控制）

**设计原则**：
- 保留 VirtualForcePlanner 作为唯一的 local planner
- 通过 `agent.strict_path` 标志切换内部模式
- 不新建 DullPlanner/GridStepPlanner 等新类

**normal 模式**（`strict_path=False`）：
- 保持当前行为不变
- 滑动窗口目标 + 完整合力

**guided 模式**（`strict_path=True`）：

1. **入口制动期（entry brake phase）**
   - 新增 `agent.strict_entry_counter` 计数器，切换时设为 `BRAKE_STEPS=8`
   - 前 8 步内：输出速度 = `current_velocity * (counter / BRAKE_STEPS)`，线性衰减到 0
   - 计数器归零后才进入正常 guided 行为
   - 解决问题 3.2

2. **按序 waypoint 消费**
   - 目标只看 `path[0]`，不用滑动窗口
   - 到达判定：`arrive(path[0])` OR `path[0]→path[1] 线段投影 > path[0]`
   - 解决问题 3.6

3. **分组排斥力**
   - 新增 `agent.cbs_group_id`：CBS 触发时，同批次的 agent 共享一个 group_id（如时间戳）
   - VirtualForce 计算排斥时按 group 分类：
     - 同组 agent：弱化系数 0.3
     - 异组 agent：保留满排斥
     - port / 墙：保留满排斥
   - 解决问题 3.3

4. **拐点预减速**
   - 检测 `path[0]` 到 `path[1]` 方向夹角
   - 夹角 > 45° 且距 `path[0]` < 1.0m 时，目标速度 = `cruise_speed * (dist / 1.0)`
   - 解决问题 3.6 的过冲

5. **arrive 阈值自适应**
   - 不用硬编码 0.2m
   - 改为 `max(0.2, cruise_speed * TIME_STEP * 3)` = 一次 tick 距离的 3 倍
   - resolution 高时阈值相对格子大小更合理

### 4.3 Server 层改进

**改动 A：切换时重置物理速度**

```python
# server.tick() 触发 CBS 时:
for agent in congested:
    agent.sequence_of_poses = solution[aid]
    agent.strict_path = True
    agent.strict_entry_counter = VirtualForcePlanner.BRAKE_STEPS
    agent.cbs_group_id = self._current_tick_id
    # 不强行把 Box2D 速度归零，交给 guided 模式的制动期平滑处理
```

**改动 B：拥堵检测的双阈值**

当前单一阈值容易抖动：
```
HARD_THRESHOLD = 0.01  # 真正卡死
SOFT_THRESHOLD = 0.05  # 明显变慢但还在动

congested 分类:
  hard: 60 步位移² < 0.01
  soft: 60 步位移² < 0.05
  
触发条件:
  len(hard) >= 2: 立即触发 CBS
  len(soft) >= 4 且冷却结束: 触发（预防性）
```

**改动 C：异步规划（可选，长期）**

- 用 Python 线程池在后台跑 `compute_path()`
- 主循环检查 future 是否就绪，就绪才分发路径
- 规划期间继续跑常态 VirtualForce，不阻塞
- 注意：规划结果基于旧快照，分发时要检查 agent 当前位置是否还在快照附近（误差 > 0.5m 则丢弃重算）

### 4.4 实施优先级

| 优先级 | 改动 | 解决问题 | 预期效果 |
|-------|------|---------|---------|
| P0 | 入口制动（4.2.1） | 3.2 | 消除撞墙主因 |
| P0 | 分组排斥力（4.2.3） | 3.3 | 消除窄道推离 |
| P1 | 投影判定 + 自适应阈值（4.2.2, 4.2.5） | 3.6 | 改善跟随精度 |
| P1 | 拐点预减速（4.2.4） | 3.6 | 避免过冲 |
| P1 | 统一 ECBS 去掉 CBSPlanner（4.1.A） | 3.4 | 代码整洁 |
| P1 | **动态前瞻距离（advice §2）** | 3.3 | normal 模式在狭窄通道不切弯 |
| P2 | 部分解降级（4.1.C） | 3.5 | 失败兜底 |
| P2 | 双阈值拥堵检测（4.3.B） | — | 减少触发抖动 |
| P2 | **路权依赖图（advice §1）** | 3.1 | 比分组排斥更精确的时序协调 |
| P2 | **拓扑锁机制（advice §4）** | — | 根本解决单车道对向死锁 |
| P3 | 异步规划（4.3.C） | 3.4 | 大场景性能 |
| P3 | **新建 ORCAPlanner（advice §3）** | 3.3 | 独立 local planner 选项，替代势场排斥 |

---

## 4.5 新增方案详述（来自 advice.md）

### 4.5.1 动态前瞻距离（P1，改进 normal 模式）

**问题**：normal 模式固定 2m 滑动窗口在狭窄通道会"切弯"，合力把 agent 拉向墙壁。

**实现**：在 `VirtualForcePlanner._normal()` 里，用 `agent.ray_length_list` 的最小值动态调整前瞻距离：

```python
min_ray = min(self.agent.ray_length_list) if self.agent.ray_length_list else 2.0
lookahead = max(0.5, min(2.0, min_ray * 0.8))
goal_pose = [x for x in global_planner_path if x.distance(position) < lookahead][-1]
```

`ray_length_list` 已在每步 `simulator.step()` 里更新，无需额外感知。

### 4.5.2 路权依赖图（P2）

从 ECBS 时序结果提取"A 先于 B 过路口"的相对顺序，转化为 B 接近路口时的减速触发条件。不依赖物理时间，只依赖"谁还没过"的状态。

### 4.5.3 拓扑锁机制（P2）

对单车道走廊建立互斥锁，进入方向相反的 agent 不能同时进入。

实现要点：
1. **地图预处理**：对 `GridmapWithNeighbors` 做 BFS，计算每个格子的通道宽度（最大内切圆半径）。宽度 < 2 × agent_radius 的连通区域标记为 corridor。
2. **Mutex 管理**：`server.py` 维护 `corridor_mutex: {corridor_id: agent_id}`。
3. **目标点钳制**：guided 模式下，若 agent 即将进入被对向 agent 占用的 corridor，将 `path[0]` 钳制在 corridor 入口外，agent 自然减速等待。

### 4.5.4 新建 ORCAPlanner（P3，独立 local planner）

**定位**：作为独立的 `LocalPlanner` 子类，通过 `--lp ORCAPlanner` 选择，不修改现有架构。

**文件**：新建 `src/local_planners/orca_planner.py`，继承 `LocalPlanner`，复用 `rvo_planner.py` 中已有的 `compute_rvo_BA`、`intersect` 等函数。

**与 VirtualForcePlanner 的区别**：
- VirtualForcePlanner：位置空间合力，有局部极小值和震荡问题
- ORCAPlanner：速度空间避障，计算最接近期望速度且不碰撞的实际速度，自然实现平滑停止

**现状**：`src/local_planners/rvo_planner.py` 已有 RVO 实现，但有已知 bug（部分分支 `NotImplementedError`）。新建 ORCAPlanner 时需修复这些 bug。

---

## 5. 不推荐的方向

- ❌ **离散动作模式（stop/spin/forward）**：ECBS 时间模型无法适配物理执行耗时
- ❌ **纯 DullPlanner / GridStepPlanner**：无排斥 = 无局部协调能力，时间脱节时必撞
- ❌ **kinodynamic CBS**：状态空间爆炸（位置×朝向×速度×时间），工程成本与规划时间都不可接受
- ❌ **抛弃 ECBS 改纯反应式**：开阔空间够用，但窄道死局无解

---

## 6. 核心设计原则

1. **接受时间模型的不完美**：ECBS 的 t 仅是规划参考，执行层用物理连续时间
2. **分离关注点**：ECBS 负责空间路径（走哪些格子），local planner 负责时间协调（什么时候走、遇到别人怎么办）
3. **保留全向控制**：不人为增加差速约束，避免引入更多时序问题
4. **分层防撞**：
   - 空间层：ECBS 保证路径不穿墙、不穿 port
   - 中距离层：CBS 空间路径错开走廊使用
   - 近距离层：排斥力兜底防物理穿透
5. **过渡平滑**：控制模式切换点必须处理瞬态（入口制动），不能假设初始状态理想

---

## 7. 关键文件索引

| 功能 | 文件 | 关键符号 |
|------|------|---------|
| 全局规划 | `src/multiagent_global_planners/ecbs_planner.py` | `ECBSPlanner.compute_path` |
| 困境规划（建议移除） | `src/multiagent_global_planners/cbs_planner.py` | `CBSPlanner` |
| Agent 主循环 | `src/agents/naive_agent.py` | `NaiveAgent.plan` |
| 本地规划 | `src/local_planners/virtual_force_planner.py` | `VirtualForcePlanner.compute_plan`（含 strict 分支） |
| 拥堵检测 + 触发 | `src/server.py` | `CongestionDetector`, `Server.tick` |
| 物理推进 | `src/simulator.py` | `Simulator.step`, `__update_agent_state` |
| 代理入口 | `src/global_planners/multiagent_planner_local_entry.py` | `MultiAgentPlannerLocalEntry` |
| 坐标转换 | `src/representation/float_to_grid.py` | `grid_to_float`, `float_to_grid`, `agent_to_gridmap` |

---

## 8. 已知修复记录

- **ECBSPlanner._to_point_paths() KeyError**：未为 grid_paths 缺失的 agent 生成空 deque，导致 `solution_paths_dict[agent.id]` 抛 KeyError。修复：函数开头用 `{aid: deque() for aid in self.agents}` 预填充所有 agent。
- **DullPlanner popleft bug**：`global_planner_path.pop(0)` 在 deque 上抛异常，应为 `popleft()`。
- **simulator.py 角度抖动**：速度近零时 `atan2(0,0)=0` 会把朝向抹为 0。修复：`if vx*vx + vy*vy > 1e-6` 时才更新 angle。
- **server.tick() strict 模式未恢复**：已加入 `strict_path=True and not sequence_of_poses → strict_path=False, goal_changed=True` 恢复逻辑。
