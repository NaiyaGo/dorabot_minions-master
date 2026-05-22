# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {xiaoyu.ge, chen.chong2}@dorabot.com
@brief : local planner based on virtual forces

Two modes controlled by agent.strict_path:
  normal  — sliding-window goal + full repulsion (original behaviour)
  guided  — sequential coordinated-waypoint following with:
              (1) entry brake: linearly decay entry velocity over BRAKE_STEPS ticks
              (2) tiered repulsion: same-group agents get weakened repulsion
              (3) projection-based waypoint advance (P1)
              (4) corner pre-deceleration (P1)
"""
from geometry import Vector, compute_direction, Point
from local_planner import LocalPlanner
from math import sqrt, acos, pi
import os
import time

# Debugging configuration: enable detailed per-agent force logging for selected agent ids
DEBUG_FORCE_LOG = True
DEBUG_AGENT_IDS = set()
# Log file will be placed in ../logs relative to this file
DEBUG_LOG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'logs', 'virtual_force_debug.log'))

# Number of ticks to spend braking when entering guided mode.
# agent.strict_entry_counter is initialised to this value by server.tick().
BRAKE_STEPS = 8

# Repulsion scale applied to agents in the same coordination group (same guided_group_id).
# 0.0 = no repulsion; 1.0 = full repulsion.
SAME_GROUP_REPULSION_SCALE = 0.2

# Distance below which same-group repulsion is always kept at full strength
# to prevent physical body overlap.
SAME_GROUP_FULL_REPULSION_DIST = 0.5

# Arrive threshold for guided mode (slightly larger than Point.arrive()'s 0.2 m).
GUIDED_ARRIVE_THRESHOLD = 0.3

# Distance from a corner waypoint at which pre-deceleration begins.
# Minimum angle (radians) between consecutive path segments to trigger pre-decel.
CORNER_ANGLE_THRESHOLD = pi / 4   # 45 degrees
CORNER_SLOWDOWN_RANGE = 1.5

# Minimum angle (radians) between consecutive path segments to trigger pre-decel.
CORNER_ANGLE_THRESHOLD = pi / 4   # 45 degrees


def _to_point(waypoint):
    return Point(waypoint[0], waypoint[1]) if isinstance(waypoint, tuple) else waypoint


def _dot(ax, ay, bx, by):
    return ax * bx + ay * by


def _norm(x, y):
    return sqrt(x * x + y * y) + 1e-9


class VirtualForcePlanner(LocalPlanner):
    """
    Keyword arguments:
    position            -- agent position (Point)
    velocity            -- current linear velocity (tuple)
    gridmap             -- GridmapWithNeighbors
    sensor_observation  -- Box2DPerception
    global_planner_path -- deque of Point waypoints
    Return:
    (vx, vy) velocity command
    """

    def compute_plan(self, position, velocity, gridmap, sensor_observation,
                     global_planner_path):
        # Rescue pulse: agent is stuck in/against a wall; server has set a
        # short outward pulse to peel it off.  Emit that velocity directly
        # and decrement the counter.  Overrides both normal and guided modes.
        rescue_ticks = getattr(self.agent, 'rescue_ticks', 0)
        if rescue_ticks > 0:
            self.agent.rescue_ticks = rescue_ticks - 1
            return getattr(self.agent, 'rescue_velocity', (0.0, 0.0))

        if getattr(self.agent, 'strict_path', False):
            return self._guided(position, velocity, sensor_observation,
                                global_planner_path)
        return self._normal(position, velocity, sensor_observation,
                            global_planner_path)

    # ------------------------------------------------------------------
    # Normal mode (original behaviour)
    # ------------------------------------------------------------------

    def _normal(self, position, velocity, sensor_observation, global_planner_path):
        try:
            goal_pose = [_to_point(x) for x in global_planner_path
                         if _to_point(x).distance(position) < 2][-1]
        except:
            try:
                goal_pose = _to_point(global_planner_path[-1])
            except:
                goal_pose = self.agent.destination_location

        # Corner pre-deceleration: detect turn angle between current heading and next segment
        speed_scale = 1.0
        if len(global_planner_path) >= 2:
            cur = _to_point(global_planner_path[0])
            nxt = _to_point(global_planner_path[1])
            ax, ay = cur.x - position.x, cur.y - position.y
            bx, by = nxt.x - cur.x, nxt.y - cur.y
            na, nb = _norm(ax, ay), _norm(bx, by)
            cos_a = max(-1.0, min(1.0, _dot(ax, ay, bx, by) / (na * nb)))
            if acos(cos_a) > CORNER_ANGLE_THRESHOLD:
                dist_to_corner = position.distance(cur)
                if dist_to_corner < CORNER_SLOWDOWN_RANGE:
                    speed_scale = max(0.4, dist_to_corner / CORNER_SLOWDOWN_RANGE)

        result_vel = Vector(velocity[0], velocity[1])
        # collect force sources for debugging (type, id/pos, vector, magnitude)
        sources = []
        # Dynamic same-group threshold based on agent radius to avoid late strong pushes
        my_group = getattr(self.agent, 'guided_group_id', None)
        my_radius = getattr(self.agent, 'shape', None).get_radius() if getattr(self.agent, 'shape', None) else SAME_GROUP_FULL_REPULSION_DIST
        full_repulsion = max(SAME_GROUP_FULL_REPULSION_DIST, my_radius)
        fade_end = full_repulsion + 1.5
        for agent_body in sensor_observation.other_agents_state_in_range_of(5):
            self.agent.potential_collision = True
            other = agent_body.userData
            other_group = getattr(other, 'guided_group_id', None)
            force = self.__repel(position, agent_body.position)
            if my_group is not None and other_group == my_group:
                d = position.distance(agent_body.position)
                if d <= full_repulsion:
                    scale = 1.0
                else:
                    t = max(0.0, 1.0 - (d - full_repulsion) / (fade_end - full_repulsion))
                    scale = SAME_GROUP_REPULSION_SCALE * t
                force = Vector(force.x * scale, force.y * scale)
            # record source
            mag = sqrt(force.x * force.x + force.y * force.y)
            sources.append(('agent', getattr(other, 'id', None), agent_body.position, force, mag))
            result_vel = result_vel + force
        for body in sensor_observation.ports_in_range_of(4):
            self.agent.potential_collision = True
            obstacle = body.userData
            # Skip repulsion from the port the agent is currently docking at
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
        for body in getattr(sensor_observation, 'detected_object', []):
            obstacle = body.userData
            if getattr(obstacle, 'type', None) in ('wall', 'obstacle'):
                pB = obstacle.center
                wall_force = self.__repel_wall(position, pB)
                mag = sqrt(wall_force.x * wall_force.x + wall_force.y * wall_force.y)
                sources.append(('wall', (pB.x, pB.y), None, wall_force, mag))
                result_vel = result_vel + wall_force

        attract_force = self.__attract(position, goal_pose)
        mag = sqrt(attract_force.x * attract_force.x + attract_force.y * attract_force.y)
        sources.append(('attract', 'goal', (goal_pose.x, goal_pose.y), attract_force, mag))
        result_vel = result_vel + attract_force
        # Debug: log top contributors before clamping
        try:
            if DEBUG_FORCE_LOG and getattr(self.agent, 'id', None) in DEBUG_AGENT_IDS:
                top = sorted(sources, key=lambda x: x[4], reverse=True)[:3]
                header = "VFP DEBUG NORMAL agent={} pos=({:.2f},{:.2f}) goal=({:.2f},{:.2f})".format(getattr(self.agent, 'id', None), position.x, position.y, goal_pose.x, goal_pose.y)
                print header
                # prepare a concise listing
                for s in top:
                    stype, sid, spos, svec, smag = s[0], s[1], s[2], s[3], s[4]
                    print "  source={} id={} pos={} vec=({:.3f},{:.3f}) mag={:.4f}".format(stype, sid, (spos.x, spos.y) if hasattr(spos, 'x') else spos, svec.x, svec.y, smag)
                # append to file
                try:
                    d = os.path.dirname(DEBUG_LOG_PATH)
                    if not os.path.exists(d):
                        os.makedirs(d)
                    with open(DEBUG_LOG_PATH, 'a') as fh:
                        fh.write(time.asctime() + '\t' + header + '\n')
                        for s in top:
                            stype, sid, spos, svec, smag = s[0], s[1], s[2], s[3], s[4]
                            fh.write('\t{}\t{}\t{}\t({:.3f},{:.3f})\t{:.4f}\n'.format(stype, sid, spos if not hasattr(spos, 'x') else (spos.x, spos.y), svec.x, svec.y, smag))
                except Exception:
                    pass
        except Exception:
            pass

        # Clamp combined force magnitude to avoid instantaneous large maneuvers
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
        # Inertia smoothing: blend with previous velocity to prevent abrupt direction reversals
        INERTIA = 0.3
        smoothed_vx = (1.0 - INERTIA) * new_vx + INERTIA * velocity[0]
        smoothed_vy = (1.0 - INERTIA) * new_vy + INERTIA * velocity[1]
        return (smoothed_vx, smoothed_vy)

    # ------------------------------------------------------------------
    # Guided mode (strict coordinated-path following)
    # ------------------------------------------------------------------

    def _guided(self, position, velocity, sensor_observation, global_planner_path):
        # (1) Entry brake phase
        counter = getattr(self.agent, 'strict_entry_counter', 0)
        if counter > 0:
            self.agent.strict_entry_counter = counter - 1
            ev = getattr(self.agent, 'entry_velocity', (0.0, 0.0))
            decay = float(counter) / BRAKE_STEPS
            return (ev[0] * decay, ev[1] * decay)

        # Path exhausted
        if not global_planner_path:
            return (0.0, 0.0)

        target = _to_point(global_planner_path[0])
        dist = position.distance(target)
        speed_scale = 1.0

        # (3) Waypoint advance + (4) corner pre-deceleration
        if len(global_planner_path) >= 2:
            nxt = _to_point(global_planner_path[1])

            # Corner detection
            ax = target.x - position.x
            ay = target.y - position.y
            bx = nxt.x - target.x
            by = nxt.y - target.y
            na = _norm(ax, ay)
            nb = _norm(bx, by)
            cos_angle = max(-1.0, min(1.0, _dot(ax, ay, bx, by) / (na * nb)))
            angle = acos(cos_angle)
            is_corner = angle > CORNER_ANGLE_THRESHOLD

            # Corner pre-deceleration
            if is_corner and dist < CORNER_SLOWDOWN_RANGE:
                speed_scale = max(0.2, dist / CORNER_SLOWDOWN_RANGE)

            # Projection-based advance (disabled for corners)
            sx = nxt.x - target.x
            sy = nxt.y - target.y
            px = position.x - target.x
            py = position.y - target.y
            seg_len_sq = sx * sx + sy * sy
            if seg_len_sq > 1e-9:
                proj = (px * sx + py * sy) / seg_len_sq
                arrive_threshold = 0.15 if is_corner else GUIDED_ARRIVE_THRESHOLD
                can_advance_by_proj = (proj > 0.8) if not is_corner else False
                if can_advance_by_proj or dist < arrive_threshold:
                    global_planner_path.popleft()
                    if not global_planner_path:
                        return (0.0, 0.0)
                    target = _to_point(global_planner_path[0])
                    dist = position.distance(target)
        else:
            if dist < GUIDED_ARRIVE_THRESHOLD:
                global_planner_path.popleft()
                return (0.0, 0.0)

        # Goal attraction
        result_vel = self.__attract(position, target)
        sources = []
        # record attraction
        mag = sqrt(result_vel.x * result_vel.x + result_vel.y * result_vel.y)
        sources.append(('attract', 'goal', (target.x, target.y), result_vel, mag))

        # (2) Tiered repulsion
        my_group = getattr(self.agent, 'guided_group_id', None)
        my_radius = getattr(self.agent, 'shape', None).get_radius() if getattr(self.agent, 'shape', None) else SAME_GROUP_FULL_REPULSION_DIST
        full_repulsion = max(SAME_GROUP_FULL_REPULSION_DIST, my_radius)
        fade_end = full_repulsion + 1.5
        # Debugging: print forces for a specific agent id (e.g., 5)
        debug_id = 5
        for agent_body in sensor_observation.other_agents_state_in_range_of(5):
            self.agent.potential_collision = True
            other = agent_body.userData
            other_group = getattr(other, 'guided_group_id', None)
            force = self.__repel(position, agent_body.position)
            if my_group is not None and other_group == my_group:
                d = position.distance(agent_body.position)
                if d <= full_repulsion:
                    scale = 1.0
                else:
                    t = max(0.0, 1.0 - (d - full_repulsion) / (fade_end - full_repulsion))
                    scale = SAME_GROUP_REPULSION_SCALE * t
                force = Vector(force.x * scale, force.y * scale)
            mag = sqrt(force.x * force.x + force.y * force.y)
            sources.append(('agent', getattr(other, 'id', None), agent_body.position, force, mag))
            result_vel = result_vel + force

        for body in sensor_observation.ports_in_range_of(4):
            self.agent.potential_collision = True
            obstacle = body.userData
            # Skip repulsion from the port the agent is currently docking at
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
        for body in getattr(sensor_observation, 'detected_object', []):
            obstacle = body.userData
            if getattr(obstacle, 'type', None) in ('wall', 'obstacle'):
                pB = obstacle.center
                wall_force = self.__repel_wall(position, pB)
                mag = sqrt(wall_force.x * wall_force.x + wall_force.y * wall_force.y)
                sources.append(('wall', (pB.x, pB.y), None, wall_force, mag))
                result_vel = result_vel + wall_force

        # Debug: log top contributors before clamping
        try:
            if DEBUG_FORCE_LOG and getattr(self.agent, 'id', None) in DEBUG_AGENT_IDS:
                top = sorted(sources, key=lambda x: x[4], reverse=True)[:3]
                header = "VFP DEBUG GUIDED agent={} pos=({:.2f},{:.2f}) target=({:.2f},{:.2f}) proj={:.3f}".format(getattr(self.agent, 'id', None), position.x, position.y, target.x, target.y, (proj if 'proj' in locals() else 0.0))
                print header
                for s in top:
                    stype, sid, spos, svec, smag = s[0], s[1], s[2], s[3], s[4]
                    print "  source={} id={} pos={} vec=({:.3f},{:.3f}) mag={:.4f}".format(stype, sid, (spos.x, spos.y) if hasattr(spos, 'x') else spos, svec.x, svec.y, smag)
                try:
                    d = os.path.dirname(DEBUG_LOG_PATH)
                    if not os.path.exists(d):
                        os.makedirs(d)
                    with open(DEBUG_LOG_PATH, 'a') as fh:
                        fh.write(time.asctime() + '\t' + header + '\n')
                        for s in top:
                            stype, sid, spos, svec, smag = s[0], s[1], s[2], s[3], s[4]
                            fh.write('\t{}\t{}\t{}\t({:.3f},{:.3f})\t{:.4f}\n'.format(stype, sid, spos if not hasattr(spos, 'x') else (spos.x, spos.y), svec.x, svec.y, smag))
                except Exception:
                    pass
        except Exception:
            pass

        # Clamp combined force magnitude to avoid instantaneous large maneuvers
        max_force = max(self.agent.cruise_speed * 1.5, 1.0)
        rv_len = sqrt(result_vel.x ** 2 + result_vel.y ** 2)
        if rv_len > max_force:
            result_vel = result_vel.scale(max_force / rv_len)
        ratio = sqrt(result_vel.x ** 2 + result_vel.y ** 2)
        if ratio < 1e-6:
            return (0.0, 0.0)
        target_speed = self.agent.cruise_speed * speed_scale
        return result_vel.scale(target_speed / ratio).to_tuple()

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    def __repel(self, pos_a, pos_b):
        dist = pos_a.distance(pos_b)
        vec = compute_direction(pos_a, pos_b).normalize()
        # d^2 decay: effective at 1-3m range without extreme near-field spikes
        return vec.scale(-3.0 / (dist ** 2 + 0.1))

    def __repel_wall(self, pos_a, pos_b):
        dist = pos_a.distance(pos_b)
        vec = compute_direction(pos_a, pos_b).normalize()
        # Reduced constant + larger epsilon to avoid pushing agents into opposite walls
        return vec.scale(-25.0 / (dist ** 2 + 0.05))

    def __attract(self, pos_current, loc_destination):
        dist = pos_current.distance(loc_destination)
        vec = compute_direction(pos_current, loc_destination).normalize()
        return vec.scale(100.0 / (dist + 0.01))

    # Legacy names kept for any external callers
    def _VirtualForcePlanner__repel_force(self, pos_a, pos_b):
        return self.__repel(pos_a, pos_b)

    def _VirtualForcePlanner__combined_force(self, pos_a, pos_b):
        return self.__repel(pos_a, pos_b)

    def _VirtualForcePlanner__goal_attraction(self, pos_current, loc_destination):
        return self.__attract(pos_current, loc_destination)
