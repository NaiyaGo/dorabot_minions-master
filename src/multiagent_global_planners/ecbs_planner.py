# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2024
@brief : Enhanced Conflict-Based Search (ECBS) multi-agent path planner.

ECBS extends CBS with a focal list to trade optimality for speed.
A suboptimality bound w >= 1.0 guarantees paths are at most w times
longer than optimal, while dramatically reducing nodes expanded.

Architecture:
  High level : focal CBS — two heaps:
                 OPEN   sorted by cost (same as CBS)
                 FOCAL  sorted by conflict count, contains nodes with
                        cost <= w * min_cost_in_open
               Pop from FOCAL (fewest conflicts first) instead of OPEN.
  Low level  : same Space-Time A* as CBS (reused from cbs_planner).

Data-flow contract: identical to CBSPlanner.
  Input  : agents dict  {agent_id: Agent}
           gridmap      GridmapWithNeighbors
  Output : paths dict   {agent_id: deque([Point, ...])}
"""

import heapq
from collections import deque
from representation.gridmap_a import GridmapWithNeighbors
from representation.float_to_grid import agent_to_gridmap, grid_to_float, float_to_grid
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from global_planners.global_planner import MapType


# ---------------------------------------------------------------------------
# Space-Time A* (self-contained, no CBS dependency)
# ---------------------------------------------------------------------------

class _STNode(object):
    """Node in the space-time search graph: (grid_x, grid_y, timestep)."""
    __slots__ = ('gx', 'gy', 't', 'g', 'h', 'parent')

    def __init__(self, gx, gy, t, g=0, h=0, parent=None):
        self.gx = gx
        self.gy = gy
        self.t  = t
        self.g  = g   # cost from start
        self.h  = h   # heuristic to goal
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

    Parameters
    ----------
    gridmap     : GridmapWithNeighbors
    start_grid  : (gx, gy) integer grid cell
    goal_grid   : (gx, gy) integer grid cell
    constraints : set of (gx, gy, t) — cells forbidden at specific timesteps
    max_t       : hard cap on search depth (prevents infinite loops)

    Returns
    -------
    list of (gx, gy) grid cells from start (exclusive) to goal (inclusive),
    or None if no path found.
    """
    sx, sy = start_grid
    gx, gy = goal_grid

    open_heap = []
    h0 = _manhattan(sx, sy, gx, gy)
    start_node = _STNode(sx, sy, 0, g=0, h=h0)
    heapq.heappush(open_heap, start_node)

    # closed set: key -> best g seen
    closed = {}

    while open_heap:
        node = heapq.heappop(open_heap)
        key = node.key()

        if key in closed and closed[key] <= node.g:
            continue
        closed[key] = node.g

        # goal check — agent may wait at goal once reached
        if node.gx == gx and node.gy == gy:
            path = []
            cur = node
            while cur.parent is not None:
                path.append((cur.gx, cur.gy))
                cur = cur.parent
            path.reverse()
            return path  # list of grid cells, start excluded

        if node.t >= max_t:
            continue

        next_t = node.t + 1

        # neighbours: 4-directional moves + wait-in-place
        moves = gridmap.neighbors((node.gx, node.gy))
        moves = list(moves) + [(node.gx, node.gy)]  # add wait action

        for (nx, ny) in moves:
            if (nx, ny, next_t) in constraints:
                continue  # forbidden by CBS/ECBS constraint
            new_g = node.g + 1
            new_h = _manhattan(nx, ny, gx, gy)
            new_key = (nx, ny, next_t)
            if new_key in closed and closed[new_key] <= new_g:
                continue
            child = _STNode(nx, ny, next_t, g=new_g, h=new_h, parent=node)
            heapq.heappush(open_heap, child)

    return None  # no path found within max_t


def _find_first_conflict(paths):
    """
    Scan all pairs of paths for the first vertex or edge conflict.

    Vertex conflict : two agents at the same cell at the same timestep.
    Edge conflict   : two agents swap cells between consecutive timesteps.

    Returns (agent_i, agent_j, gx, gy, t) or None if no conflict.
    The returned cell/time is the one to constrain.
    """
    agent_ids = list(paths.keys())
    timelines = {}
    for aid, path in paths.items():
        timelines[aid] = path  # index i => timestep i+1

    max_t = max(len(p) for p in paths.values()) if paths else 0
    if max_t == 0:
        return None

    for i in range(len(agent_ids)):
        for j in range(i + 1, len(agent_ids)):
            ai = agent_ids[i]
            aj = agent_ids[j]
            pi = timelines[ai]
            pj = timelines[aj]
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


def _path_cost(path):
    """Sum-of-costs: number of moves (length of path list)."""
    return len(path) if path else 0


# ---------------------------------------------------------------------------
# Conflict counting (secondary heuristic for focal list ordering)
# ---------------------------------------------------------------------------

def _count_all_conflicts(paths):
    """
    Count total vertex + edge conflicts across all agent pairs.
    Used to order nodes in the ECBS focal list (fewer conflicts = higher priority).

    Parameters
    ----------
    paths : dict {agent_id: list of (gx, gy)}

    Returns
    -------
    int — total number of conflicts
    """
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


# ---------------------------------------------------------------------------
# ECBS high-level node (adds conflict_count for focal ordering)
# ---------------------------------------------------------------------------

class _ECBSNode(object):
    """CBS constraint-tree node extended with a conflict count for focal ordering."""
    __slots__ = ('constraints', 'paths', 'cost', 'conflict_count')

    def __init__(self, constraints, paths, cost, conflict_count):
        self.constraints    = constraints
        self.paths          = paths
        self.cost           = cost
        self.conflict_count = conflict_count

    # OPEN heap: ordered by cost (same as CBS)
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
# Public ECBS planner class
# ---------------------------------------------------------------------------

class ECBSPlanner(MultiAgentPlanner):
    """
    Enhanced Conflict-Based Search (ECBS) planner.

    Uses a focal list to prioritise nodes with fewer conflicts, allowing
    suboptimal but faster solutions bounded by SUBOPTIMAL_FACTOR.

    Usage:
        python simulator.py --default --gp ECBSPlanner --agent 4 -d
    """
    MAP = MapType.GRID

    # w >= 1.0: paths are at most w times longer than CBS-optimal.
    # 1.3 gives a good speed/quality tradeoff for warehouse scenarios.
    SUBOPTIMAL_FACTOR = 1.3

    # Maximum high-level nodes to expand before giving up
    MAX_CBS_NODES = 500
    # Maximum timesteps for low-level Space-Time A*
    MAX_TIME = 300

    def compute_path(self):
        """
        Compute conflict-free (or bounded-suboptimal) paths for all agents.

        Returns
        -------
        dict {agent_id: deque([Point, ...])}
        """
        gridmap    = GridmapWithNeighbors(self.environment_map)
        resolution = gridmap.resolution
        w          = self.SUBOPTIMAL_FACTOR

        # --- Collect start / goal grid cells ---
        starts = {}
        goals  = {}
        for aid, agent in self.agents.items():
            if not agent.has_destination():
                continue
            sg = agent_to_gridmap(agent.position, gridmap, resolution,
                                  destination_location=agent.destination_location,
                                  centroid=False)
            gg = float_to_grid((agent.destination_location.x, agent.destination_location.y),
                               resolution, centroid=False)
            starts[aid] = sg
            goals[aid]  = gg

        if not starts:
            return {aid: deque() for aid in self.agents}

        # --- Root node: unconstrained individual paths ---
        root_constraints = {aid: set() for aid in starts}
        root_paths = {}
        for aid in starts:
            p = _space_time_astar(gridmap, starts[aid], goals[aid],
                                  root_constraints[aid], self.MAX_TIME)
            if p is None:
                s = starts[aid]
                g = goals[aid]
                s_ok = gridmap.in_bounds(s) and gridmap.passable(s)
                g_ok = gridmap.in_bounds(g) and gridmap.passable(g)
                print "ECBSPlanner: A* failed for agent {}, start {}, goal {}, start_ok {}, goal_ok {}"\
                    .format(aid, s, g, s_ok, g_ok)
                reachable = gridmap.available_list_from_location(s)
                goal_reachable = g in reachable
                print "ECBSPlanner: reachable_count {}, goal_reachable {}"\
                    .format(len(reachable), goal_reachable)
            # p == [] means already at goal; only None indicates failure.
            root_paths[aid] = p

        # Filter agents for which A* found no path (e.g. trapped by wall)
        failed_agents = [aid for aid, p in root_paths.items() if p is None]
        for aid in failed_agents:
            root_paths.pop(aid)
            root_constraints.pop(aid)
            starts.pop(aid)
            goals.pop(aid)
            print "ECBSPlanner: no path for agent {}, skipping".format(aid)

        if not root_paths:
            return {aid: deque() for aid in self.agents}

        root_cost     = sum(_path_cost(p) for p in root_paths.values())
        root_conflicts = _count_all_conflicts(root_paths)
        root_node     = _ECBSNode(root_constraints, root_paths,
                                  root_cost, root_conflicts)

        # --- OPEN heap (by cost) and FOCAL heap (by conflict_count) ---
        open_heap  = [root_node]          # min-heap by cost
        focal_heap = [_FocalEntry(root_node)]  # min-heap by conflict_count

        # Track which nodes are in focal to avoid duplicates
        focal_set  = {id(root_node)}

        nodes_expanded = 0
        f_min = root_cost  # lowest cost in OPEN

        while open_heap and nodes_expanded < self.MAX_CBS_NODES:
            # Update f_min and rebuild focal if the minimum cost changed
            new_f_min = open_heap[0].cost
            if new_f_min > f_min:
                f_min = new_f_min
                # Add newly eligible nodes to focal
                for node in open_heap:
                    if node.cost <= w * f_min and id(node) not in focal_set:
                        heapq.heappush(focal_heap, _FocalEntry(node))
                        focal_set.add(id(node))

            # Pop best node from focal (fewest conflicts)
            while focal_heap:
                entry = heapq.heappop(focal_heap)
                focal_set.discard(id(entry.node))
                # Verify node is still in open (may have been superseded)
                if entry.node in open_heap:
                    node = entry.node
                    open_heap.remove(node)
                    heapq.heapify(open_heap)
                    break
            else:
                break  # focal empty

            nodes_expanded += 1

            conflict = _find_first_conflict(node.paths)
            if conflict is None:
                # Solution found — convert grid paths to float Points
                return self._to_point_paths(node.paths, resolution)

            ai, aj, cx, cy, ct = conflict

            # Branch on the conflict: constrain ai, then aj
            for constrained_agent in (ai, aj):
                if constrained_agent not in node.constraints:
                    continue  # agent was filtered out earlier

                new_constraints = {}
                for aid in node.constraints:
                    new_constraints[aid] = set(node.constraints[aid])
                new_constraints[constrained_agent].add((cx, cy, ct))

                new_paths = dict(node.paths)
                new_p = _space_time_astar(
                    gridmap,
                    starts[constrained_agent],
                    goals[constrained_agent],
                    new_constraints[constrained_agent],
                    self.MAX_TIME)

                if new_p is None:
                    continue  # branch infeasible, prune

                new_paths[constrained_agent] = new_p
                new_cost      = sum(_path_cost(p) for p in new_paths.values())
                new_conflicts = _count_all_conflicts(new_paths)
                child = _ECBSNode(new_constraints, new_paths,
                                  new_cost, new_conflicts)

                heapq.heappush(open_heap, child)
                # Add to focal if within bound
                if new_cost <= w * f_min:
                    heapq.heappush(focal_heap, _FocalEntry(child))
                    focal_set.add(id(child))

        # Node limit reached — return best partial solution found
        if open_heap:
            best = min(open_heap, key=lambda n: n.conflict_count)
            return self._to_point_paths(best.paths, resolution)

        return {aid: deque() for aid in self.agents}

    def _to_point_paths(self, grid_paths, resolution):
        """Convert grid-cell paths to float-Point deques.

        Every agent registered in self.agents gets an entry in the result —
        agents with no path (filtered out earlier) get an empty deque.
        This prevents KeyError in request_multiagent_global_planner_compute_path().
        """
        from geometry import Point as _Point
        result = {}
        # Ensure every known agent has an entry, even if no path was found
        for aid in self.agents:
            result[aid] = deque()
        for aid, path in grid_paths.items():
            pts = deque()
            for (gx, gy) in path:
                fx, fy = grid_to_float((gx, gy), resolution, centroid=False)
                pts.append(_Point(fx, fy))
            result[aid] = pts
        return result
