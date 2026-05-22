1.修复错误，当机器人agent与墙壁碰撞发生时:程序崩溃，错误发生如下Traceback (most recent call last):
  File "simulator.py", line 429, in <module>
    start_simulator(args)
  File "simulator.py", line 418, in start_simulator
    simulator.run(show_visualisation)
  File "simulator.py", line 287, in run
    vis.run()
  File "E:\dorabot_minions-master\src\visualisation.py", line 287, in run
    self.simulator.step()
  File "simulator.py", line 256, in step
    self.server.tick(self)
  File "E:\dorabot_minions-master\src\server.py", line 147, in tick
    solution = cbs.compute_path()  # {agent_id: deque([Point, ...])}
  File "E:\dorabot_minions-master\src\multiagent_global_planners\cbs_planner.py", line 286, in compute_path
    conflict = _find_first_conflict(node.paths)
  File "E:\dorabot_minions-master\src\multiagent_global_planners\cbs_planner.py", line 192, in _find_first_conflict    cj = pj[t - 1] if t - 1 < len(pj) else pj[-1]

  2.我们现在新增采用ECBS算法，作为全局planner,这时候我希望你把这些机器人agent，禁止他们全向移动，我希望他们有个启动额外选项，可以禁止机器人当前会按照一定范围内最远的路径规划点走直线距离，完全按照全局planner给他们规划的全局路径,这样就可以防止他们碰撞。另外对于全局planner，关于墙壁的计算，确认墙壁建模是比原来大或者一样，以保证感受器可以正确获得信息，以进行全局路段避障。