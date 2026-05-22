# -*- coding: utf-8 -*-
"""
@brief: Pure ECBS (Enhanced Conflict-Based Search) multi-agent path planner.

ECBS extends CBS with a focal list to trade optimality for speed.
Suboptimality bound w >= 1.0 guarantees paths are at most w times
longer than optimal.

High level : two heaps — OPEN (sorted by cost) and FOCAL (sorted by
             conflict count, contains nodes with cost <= w * min_open_cost).
             Pop from FOCAL (fewest conflicts first).
Low level  : Space-Time A* with per-agent constraints.
"""

import heapq
from collections import deque
from representation.gridmap_a import GridmapWithNeighbors
from representation.float_to_grid import agent_to_gridmap, grid_to_float, float_to_grid
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from global_planners.global_planner import MapType


# ---------------------------------------------------------------------------
# Space-Time A*
# ---------------------------------------------------------------------------

class _STNode(object):
    __slots__ = ('gx', 'gy', 't', 'g', 'h', 'parent')

    def __init__(self, gx, gy, t, g=0, h=0, parent=None):
        self.gx = gx
        self.gy = gy
        self.t  = t
        self.g  = g
        self.h  = h
        self.parent = parent

    @property
    def f(self):
        return self.g + self.h

    def __lt__(self, other):
        return self.f < other.f

    def key(self):
        return (self.gx, self.gy, self.t)


def _manhattan(ax, ay, bx, by):
    return abs(ax - bx) + abs(ay - by)


def _space_time_astar(gridmap, start_grid, goal_grid, constraints, max_t=200):
    """
    Space-Time A* with constraints.

    Returns list of (gx, gy) from start (exclusive) to goal (inclusive),
    or None if no path found within max_t.
    """
    gx, gy = goal_grid
    root = _STNode(start_grid[0], start_grid[1], 0,
                   g=0, h=_manhattan(start_grid[0], start_grid[1], gx, gy))
    open_heap = [root]
    closed = {}  # (gx, gy, t) -> best g seen

    while open_heap:
        node = heapq.heappop(open_heap)
        state = node.key()
        if state in closed and closed[state] <= node.g:
            continue
        closed[state] = node.g

        if node.gx == gx and node.gy == gy:
            path = []
            cur = node
            while cur.parent is not None:
                path.append((cur.gx, cur.gy))
                cur = cur.parent
            path.reverse()
            return path

        if node.t >= max_t:
            continue

        next_t = node.t + 1
        moves = list(gridmap.neighbors((node.gx, node.gy))) + [(node.gx, node.gy)]

        for (nx, ny) in moves:
            if (nx, ny, next_t) in constraints:
                continue
            new_g = node.g + 1
            new_h = _manhattan(nx, ny, gx, gy)
            new_key = (nx, ny, next_t)
            if new_key in closed and closed[new_key] <= new_g:
                continue
            heapq.heappush(open_heap,
                           _STNode(nx, ny, next_t, g=new_g, h=new_h, parent=node))

    return None


# ---------------------------------------------------------------------------
# Conflict detection
# ---------------------------------------------------------------------------

def _find_first_conflict(paths):
    """
    Return (agent_i, agent_j, gx, gy, t) for the first vertex or edge
    conflict found, or None if paths are conflict-free.
    """
    agent_ids = list(paths.keys())
    max_t = max(len(p) for p in paths.values()) if paths else 0
    if max_t == 0:
        return None

    for i in range(len(agent_ids)):
        for j in range(i + 1, len(agent_ids)):
            ai, aj = agent_ids[i], agent_ids[j]
            pi, pj = paths[ai], paths[aj]
            if not pi or not pj:
                continue
            for t in range(1, max_t + 1):
                ci = pi[t - 1] if t - 1 < len(pi) else pi[-1]
                cj = pj[t - 1] if t - 1 < len(pj) else pj[-1]
                if ci == cj:
                    return (ai, aj, ci[0], ci[1], t)
                if t >= 2:
                    ci_prev = pi[t - 2] if t - 2 < len(pi) else pi[-1]
                    cj_prev = pj[t - 2] if t - 2 < len(pj) else pj[-1]
                    if ci == cj_prev and cj == ci_prev:
                        return (ai, aj, ci[0], ci[1], t)
    return None


def _count_all_conflicts(paths):
    """Count total vertex + edge conflicts across all agent pairs."""
    agent_ids = list(paths.keys())
    max_t = max(len(p) for p in paths.values()) if paths else 0
    if max_t == 0:
        return 0

    count = 0
    for i in range(len(agent_ids)):
        for j in range(i + 1, len(agent_ids)):
            pi = paths[agent_ids[i]]
            pj = paths[agent_ids[j]]
            if not pi or not pj:
                continue
            for t in range(1, max_t + 1):
                ci = pi[t - 1] if t - 1 < len(pi) else pi[-1]
                cj = pj[t - 1] if t - 1 < len(pj) else pj[-1]
                if ci == cj:
                    count += 1
                if t >= 2:
                    ci_prev = pi[t - 2] if t - 2 < len(pi) else pi[-1]
                    cj_prev = pj[t - 2] if t - 2 < len(pj) else pj[-1]
                    if ci == cj_prev and cj == ci_prev:
                        count += 1
    return count


def _path_cost(path):
    return len(path) if path else 0


# ---------------------------------------------------------------------------
# ECBS high-level nodes
# ---------------------------------------------------------------------------

class _ECBSNode(object):
    __slots__ = ('constraints', 'paths', 'cost', 'conflict_count')

    def __init__(self, constraints, paths, cost, conflict_count):
        self.constraints    = constraints
        self.paths          = paths
        self.cost           = cost
        self.conflict_count = conflict_count

    def __lt__(self, other):
        return self.cost < other.cost


class _FocalEntry(object):
    """Wrapper for focal heap: ordered by conflict_count then cost."""
    __slots__ = ('conflict_count', 'cost', 'node')

    def __init__(self, node):
        self.conflict_count = node.conflict_count
        self.cost           = node.cost
        self.node           = node

    def __lt__(self, other):
        if self.conflict_count != other.conflict_count:
            return self.conflict_count < other.conflict_count
        return self.cost < other.cost


# ---------------------------------------------------------------------------
# Public ECBS planner
# ---------------------------------------------------------------------------

class ECBSNewPlanner(MultiAgentPlanner):
    """
    Enhanced Conflict-Based Search (ECBS) planner.

    SUBOPTIMAL_FACTOR w >= 1.0: returned paths are at most w times the
    CBS-optimal sum-of-costs. Higher w = faster but less optimal.
    """
    MAP = MapType.GRID

    SUBOPTIMAL_FACTOR = 1.3
    MAX_CBS_NODES     = 500
    MAX_TIME          = 300

    def compute_path(self):
        """
        Compute conflict-free (bounded-suboptimal) paths for all agents.

        Returns
        -------
        dict {agent_id: deque([Point, ...])}
        """
        gridmap    = GridmapWithNeighbors(self.environment_map)
        resolution = gridmap.resolution
        w          = self.SUBOPTIMAL_FACTOR

        # Collect start / goal grid cells for agents that have a destination
        starts = {}
        goals  = {}
        for aid, agent in self.agents.items():
            if not agent.has_destination():
                continue
            sg = agent_to_gridmap(agent.position, gridmap, resolution,
                                  destination_location=agent.destination_location,
                                  centroid=False)
            gg = float_to_grid(
                (agent.destination_location.x, agent.destination_location.y),
                resolution, centroid=False)
            starts[aid] = sg
            goals[aid]  = gg

        if not starts:
            return {aid: deque() for aid in self.agents}

        # Root node: unconstrained individual paths
        root_constraints = {aid: set() for aid in starts}
        root_paths = {}
        for aid in starts:
            p = _space_time_astar(gridmap, starts[aid], goals[aid],
                                  root_constraints[aid], self.MAX_TIME)
            root_paths[aid] = p

        # Drop agents for which no path exists
        for aid in [aid for aid, p in root_paths.items() if p is None]:
            root_paths.pop(aid)
            root_constraints.pop(aid)
            starts.pop(aid)
            goals.pop(aid)

        if not root_paths:
            return {aid: deque() for aid in self.agents}

        root_cost      = sum(_path_cost(p) for p in root_paths.values())
        root_conflicts = _count_all_conflicts(root_paths)
        root_node      = _ECBSNode(root_constraints, root_paths,
                                   root_cost, root_conflicts)

        open_heap  = [root_node]
        focal_heap = [_FocalEntry(root_node)]
        focal_set  = {id(root_node)}

        nodes_expanded = 0
        f_min = root_cost

        while open_heap and nodes_expanded < self.MAX_CBS_NODES:
            # Update f_min and promote newly eligible nodes into focal
            new_f_min = open_heap[0].cost
            if new_f_min > f_min:
                f_min = new_f_min
                for n in open_heap:
                    if n.cost <= w * f_min and id(n) not in focal_set:
                        heapq.heappush(focal_heap, _FocalEntry(n))
                        focal_set.add(id(n))

            # Pop best node from focal (fewest conflicts)
            node = None
            while focal_heap:
                entry = heapq.heappop(focal_heap)
                focal_set.discard(id(entry.node))
                if entry.node in open_heap:
                    node = entry.node
                    break
            if node is None:
                break

            open_heap.remove(node)
            heapq.heapify(open_heap)
            nodes_expanded += 1

            conflict = _find_first_conflict(node.paths)
            if conflict is None:
                return self._to_point_paths(node.paths, resolution)

            ai, aj, cx, cy, ct = conflict

            for constrained_agent in (ai, aj):
                if constrained_agent not in node.constraints:
                    continue

                new_constraints = {aid: set(s) for aid, s in node.constraints.items()}
                new_constraints[constrained_agent].add((cx, cy, ct))

                new_paths = dict(node.paths)
                new_p = _space_time_astar(
                    gridmap,
                    starts[constrained_agent],
                    goals[constrained_agent],
                    new_constraints[constrained_agent],
                    self.MAX_TIME)

                if new_p is None:
                    continue

                new_paths[constrained_agent] = new_p
                new_cost      = sum(_path_cost(p) for p in new_paths.values())
                new_conflicts = _count_all_conflicts(new_paths)
                child = _ECBSNode(new_constraints, new_paths, new_cost, new_conflicts)

                heapq.heappush(open_heap, child)
                if new_cost <= w * f_min:
                    heapq.heappush(focal_heap, _FocalEntry(child))
                    focal_set.add(id(child))

        # Node limit reached — return best partial solution found
        if open_heap:
            best = min(open_heap, key=lambda n: n.conflict_count)
            return self._to_point_paths(best.paths, resolution)

        return {aid: deque() for aid in self.agents}

    def _to_point_paths(self, grid_paths, resolution):
        from geometry import Point as _Point
        result = {aid: deque() for aid in self.agents}
        for aid, path in grid_paths.items():
            pts = deque()
            for (gx, gy) in path:
                fx, fy = grid_to_float((gx, gy), resolution, centroid=False)
                pts.append(_Point(fx, fy))
            result[aid] = pts
        return result
