# -*- coding: utf-8 -*-
"""
精简版 VirtualForcePlanner
- 删除 strict/guided 逻辑，只保留阻挡/端口/agent 间的相互作用。
- 保持救援 pulse（server 设置）的优先级。
"""
from geometry import Vector, compute_direction, Point
from local_planner import LocalPlanner
from math import sqrt, acos, pi
import os
import time

# Debugging configuration
DEBUG_FORCE_LOG = False
DEBUG_AGENT_IDS = set()
DEBUG_LOG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'logs', 'virtual_force_debug.log'))

# Parameters
SAME_GROUP_FULL_REPULSION_DIST = 0.5  # 保留为参考（不做 group 缩放）
GUIDED_ARRIVE_THRESHOLD = 0.3
CORNER_ANGLE_THRESHOLD = pi / 4
CORNER_SLOWDOWN_RANGE = 1.5
LOOKAHEAD_DIST = 1.5   # 前瞻弧长（米），沿路径累积到此距离取目标点
CORNER_SCAN_STEPS = 3  # 角点减速时向前扫描的路点数

def _to_point(waypoint):
    return Point(waypoint[0], waypoint[1]) if isinstance(waypoint, tuple) else waypoint

def _dot(ax, ay, bx, by):
    return ax * bx + ay * by

def _norm(x, y):
    return sqrt(x * x + y * y) + 1e-9

class VFP(LocalPlanner):
    """
    精简版本地规划器：基于虚拟力（吸引到全局路径点 + 来自其他 agent / port / 障碍的斥力）。
    compute_plan 返回速度命令 (vx, vy)。
    """

    def _lookahead_goal(self, position, global_planner_path):
        """
        先找路径中距离 agent 最近的路点作为当前进度，
        再从该点向前累积弧长，返回第一个累积距离 >= LOOKAHEAD_DIST 的路点。
        """
        if not global_planner_path:
            return self.agent.destination_location
        pts = [_to_point(x) for x in global_planner_path]

        # 找最近路点的索引
        nearest_idx = 0
        nearest_dist = pts[0].distance(position)
        for i in range(1, len(pts)):
            d = pts[i].distance(position)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        # 从最近路点开始向前累积弧长
        arc = 0.0
        prev = position
        for pt in pts[nearest_idx:]:
            arc += prev.distance(pt)
            if arc >= LOOKAHEAD_DIST:
                return pt
            prev = pt
        return pts[-1]

    def compute_plan(self, position, velocity, gridmap, sensor_observation, 
                     global_planner_path):
        # Rescue pulse 优先（server 可能设置）
        rescue_ticks = getattr(self.agent, 'rescue_ticks', 0)
        if rescue_ticks > 0:
            self.agent.rescue_ticks = rescue_ticks - 1
            return getattr(self.agent, 'rescue_velocity', (0.0, 0.0))

        return self._plan(position, velocity, sensor_observation, global_planner_path)

    def _plan(self, position, velocity, sensor_observation, global_planner_path):
        # 前瞻路径跟踪：从最近路点开始累积弧长，取目标点
        goal_pose = self._lookahead_goal(position, global_planner_path)

        # 角点预减速：从最近路点向前扫描 CORNER_SCAN_STEPS 段，取最严重转角缩放速度
        speed_scale = 1.0
        path_pts = [_to_point(x) for x in global_planner_path]
        if path_pts:
            nearest_idx = min(range(len(path_pts)), key=lambda i: path_pts[i].distance(position))
            scan_pts = path_pts[nearest_idx: nearest_idx + CORNER_SCAN_STEPS + 1]
            prev = position
            for k in range(len(scan_pts)):
                cur = scan_pts[k]
                if k + 1 >= len(scan_pts):
                    break
                nxt = scan_pts[k + 1]
                ax, ay = cur.x - prev.x, cur.y - prev.y
                bx, by = nxt.x - cur.x, nxt.y - cur.y
                na, nb = _norm(ax, ay), _norm(bx, by)
                cos_a = max(-1.0, min(1.0, _dot(ax, ay, bx, by) / (na * nb)))
                if acos(cos_a) > CORNER_ANGLE_THRESHOLD:
                    dist_to_corner = position.distance(cur)
                    if dist_to_corner < CORNER_SLOWDOWN_RANGE:
                        scale = max(0.4, dist_to_corner / CORNER_SLOWDOWN_RANGE)
                        speed_scale = min(speed_scale, scale)
                prev = cur

        result_vel = Vector(velocity[0], velocity[1])
        sources = []

        # 斥力：其他 agents（不做 group 缩放）
        for agent_body in sensor_observation.other_agents_state_in_range_of(5):
            self.agent.potential_collision = True
            other = agent_body.userData
            force = self.__repel(position, agent_body.position)
            mag = sqrt(force.x * force.x + force.y * force.y)
            sources.append(('agent', getattr(other, 'id', None), agent_body.position, force, mag))
            result_vel = result_vel + force

        # 端口（port）斥力：若当前任务是该 port 且处于排队/装货等状态则跳过斥力
        for body in sensor_observation.ports_in_range_of(4):
            self.agent.potential_collision = True
            obstacle = body.userData
            from agents.agent_state_machine import AgentState as _AS
            _task = getattr(self.agent, 'task', None)
            if _task is not None and getattr(_task, 'port', None) is obstacle:
                if getattr(self.agent, 'state', None) in (_AS.QUEUING, _AS.LOADING, _AS.PREQUEUE):
                    continue
            pB = Point(obstacle.location.x + obstacle.dimension[0] * 0.5,
                       obstacle.location.y + obstacle.dimension[1] * 0.5)
            wall_force = self.__repel_wall(position, pB)
            mag = sqrt(wall_force.x * wall_force.x + wall_force.y * wall_force.y)
            sources.append(('wall', (pB.x, pB.y), None, wall_force, mag))
            result_vel = result_vel + wall_force

        # 额外检测到的障碍物
        for body in getattr(sensor_observation, 'detected_object', []):
            obstacle = body.userData
            if getattr(obstacle, 'type', None) in ('wall', 'obstacle'):
                pB = obstacle.center
                wall_force = self.__repel_wall(position, pB)
                mag = sqrt(wall_force.x * wall_force.x + wall_force.y * wall_force.y)
                sources.append(('wall', (pB.x, pB.y), None, wall_force, mag))
                result_vel = result_vel + wall_force

        # 吸引力指向目标
        attract_force = self.__attract(position, goal_pose)
        mag = sqrt(attract_force.x * attract_force.x + attract_force.y * attract_force.y)
        sources.append(('attract', 'goal', (goal_pose.x, goal_pose.y), attract_force, mag))
        result_vel = result_vel + attract_force

        # Debug logging（可选）
        try:
            if DEBUG_FORCE_LOG and getattr(self.agent, 'id', None) in DEBUG_AGENT_IDS:
                top = sorted(sources, key=lambda x: x[4], reverse=True)[:5]
                header = "VFP DEBUG agent={} pos=({:.2f},{:.2f}) goal=({:.2f},{:.2f})".format(
                    getattr(self.agent, 'id', None), position.x, position.y, goal_pose.x, goal_pose.y)
                print header
                for s in top:
                    stype, sid, spos, svec, smag = s
                    print "  source={} id={} pos={} vec=({:.3f},{:.3f}) mag={:.4f}".format(
                        stype, sid, (spos.x, spos.y) if hasattr(spos, 'x') else spos, svec.x, svec.y, smag)
                try:
                    d = os.path.dirname(DEBUG_LOG_PATH)
                    if not os.path.exists(d):
                        os.makedirs(d)
                    with open(DEBUG_LOG_PATH, 'a') as fh:
                        fh.write(time.asctime() + '\t' + header + '\n')
                        for s in top:
                            stype, sid, spos, svec, smag = s
                            fh.write('\t{}\t{}\t{}\t({:.3f},{:.3f})\t{:.4f}\n'.format(
                                stype, sid, spos if not hasattr(spos, 'x') else (spos.x, spos.y), svec.x, svec.y, smag))
                except Exception:
                    pass
        except Exception:
            pass

        # 限幅 + 惯性平滑（与原实现相同）
        max_force = max(self.agent.cruise_speed * 1.5, 1.0)
        rv_len = sqrt(result_vel.x ** 2 + result_vel.y ** 2)
        if rv_len > max_force:
            result_vel = result_vel.scale(max_force / rv_len)
        ratio = sqrt(result_vel.x ** 2 + result_vel.y ** 2)
        if ratio < 1e-6:
            return (0.0, 0.0)
        target_speed = self.agent.cruise_speed * speed_scale
        new_vx = result_vel.x / ratio * target_speed
        new_vy = result_vel.y / ratio * target_speed
        INERTIA = 0.3
        smoothed_vx = (1.0 - INERTIA) * new_vx + INERTIA * velocity[0]
        smoothed_vy = (1.0 - INERTIA) * new_vy + INERTIA * velocity[1]
        return (smoothed_vx, smoothed_vy)

    # 助手方法：斥力/墙斥力/吸引力（保留原始常数）
    def __repel(self, pos_a, pos_b):
        dist = pos_a.distance(pos_b)
        vec = compute_direction(pos_a, pos_b).normalize()
        return vec.scale(-3.0 / (dist ** 2 + 0.1))

    def __repel_wall(self, pos_a, pos_b):
        dist = pos_a.distance(pos_b)
        vec = compute_direction(pos_a, pos_b).normalize()
        return vec.scale(-25.0 / (dist ** 2 + 0.05))

    def __attract(self, pos_current, loc_destination):
        dist = pos_current.distance(loc_destination)
        vec = compute_direction(pos_current, loc_destination).normalize()
        return vec.scale(100.0 / (dist + 0.01))

    # 兼容旧名称
    def _VirtualForcePlanner__repel_force(self, pos_a, pos_b):
        return self.__repel(pos_a, pos_b)

    def _VirtualForcePlanner__combined_force(self, pos_a, pos_b):
        return self.__repel(pos_a, pos_b)

    def _VirtualForcePlanner__goal_attraction(self, pos_current, loc_destination):
        return self.__attract(pos_current, loc_destination)