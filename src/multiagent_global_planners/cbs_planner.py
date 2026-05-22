# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2024
@brief : Conflict-Based Search (CBS) multi-agent path planner.

Architecture:
  High level : constraint tree — each node holds a set of (agent_id, grid_pos, timestep)
               constraints and a per-agent space-time path that satisfies them.
  Low level  : Space-Time A* — standard A* extended with a time dimension so that
               constraints of the form "agent i must not occupy cell c at time t"
               can be enforced directly.

Data-flow contract (matches the rest of the codebase):
  Input  : agents dict  {agent_id: Agent}  (from server.agents or a subset)
           gridmap      GridmapWithNeighbors
  Output : paths dict   {agent_id: deque([Point, ...])}
           — same format as agent.sequence_of_poses and what
             server.refresh_agents_path() expects.

Grid / float coordinate convention (matches float_to_grid.py):
  grid cell (gx, gy)  <->  float Point(gx / resolution, gy / resolution)
  with centroid=False (corner-aligned), resolution=1 by default.
  agent_to_gridmap() and grid_to_float() from float_to_grid.py are reused.
"""

import heapq
from collections import deque
from geometry import Point
from representation.gridmap_a import GridmapWithNeighbors
from representation.float_to_grid import agent_to_gridmap, grid_to_float
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from global_planners.global_planner import MapType


# ---------------------------------------------------------------------------
# Space-Time A*
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

    # heapq uses < for ordering
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
                continue  # forbidden by CBS constraint
            # also forbid swapping positions with another agent (edge conflict)
            # represented as constraint on (node.gx, node.gy, next_t) from the
            # other agent's perspective — handled by CBS at the high level.
            new_g = node.g + 1
            new_h = _manhattan(nx, ny, gx, gy)
            new_key = (nx, ny, next_t)
            if new_key in closed and closed[new_key] <= new_g:
                continue
            child = _STNode(nx, ny, next_t, g=new_g, h=new_h, parent=node)
            heapq.heappush(open_heap, child)

    return None  # no path found within max_t


# ---------------------------------------------------------------------------
# CBS high-level
# ---------------------------------------------------------------------------

class _CBSNode(object):
    """Node in the CBS constraint tree."""
    __slots__ = ('constraints', 'paths', 'cost')

    def __init__(self, constraints, paths, cost):
        # constraints: dict {agent_id: set of (gx, gy, t)}
        self.constraints = constraints
        # paths: dict {agent_id: list of (gx, gy)}  — start cell excluded
        self.paths = paths
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost


def _find_first_conflict(paths):
    """
    Scan all pairs of paths for the first vertex or edge conflict.

    Vertex conflict : two agents at the same cell at the same timestep.
    Edge conflict   : two agents swap cells between consecutive timesteps.

    Returns (agent_i, agent_j, gx, gy, t) or None if no conflict.
    The returned cell/time is the one to constrain.
    """
    agent_ids = list(paths.keys())
    # build position-at-time lookup: {agent_id: [(gx,gy), ...]} index = timestep
    # timestep 0 = start position (not in path list, handled separately)
    # We store full timeline including t=0 start.
    timelines = {}
    for aid, path in paths.items():
        # path is list of cells AFTER start; prepend start implicitly via index offset
        timelines[aid] = path  # index i => timestep i+1

    max_t = max(len(p) for p in paths.values()) if paths else 0
    if max_t == 0:
        return None  # all paths empty, no conflicts possible

    for i in range(len(agent_ids)):
        for j in range(i + 1, len(agent_ids)):
            ai = agent_ids[i]
            aj = agent_ids[j]
            pi = timelines[ai]
            pj = timelines[aj]
            # An agent with no path cannot conflict with anyone — skip
            if not pi or not pj:
                continue

            for t in range(1, max_t + 1):
                # position at time t: last cell if path ended
                ci = pi[t - 1] if t - 1 < len(pi) else pi[-1]
                cj = pj[t - 1] if t - 1 < len(pj) else pj[-1]

                # vertex conflict
                if ci == cj:
                    return (ai, aj, ci[0], ci[1], t)

                # edge conflict (swap)
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
# Public CBS planner class
# ---------------------------------------------------------------------------

class CBSPlanner(MultiAgentPlanner):
    """
    Conflict-Based Search planner.

    Inherits MultiAgentPlanner so it integrates with the existing
    server.add_multiagent_local_planner() / request_multiagent_global_planner_compute_path()
    infrastructure.

    MAP = GRID so simulator.set_global_planner() passes static_gridmap.
    """
    MAP = MapType.GRID

    # Maximum CBS nodes to expand before giving up (prevents exponential blowup)
    MAX_CBS_NODES = 500
    # Maximum timesteps for space-time A* per agent
    MAX_TIME = 300

    def compute_path(self):
        """
        Compute conflict-free paths for all agents under control.

        Returns
        -------
        dict {agent_id: deque([Point, ...])}
            Paths in the same format as agent.sequence_of_poses.
            Agents for which no path is found keep an empty deque.
        """
        gridmap = GridmapWithNeighbors(self.environment_map)
        resolution = gridmap.resolution

        # Collect start and goal grid cells for each agent that has a destination
        starts = {}   # {agent_id: (gx, gy)}
        goals  = {}   # {agent_id: (gx, gy)}

        for aid, agent in self.agents.items():
            if not agent.has_destination():
                continue
            # Start: snap agent's current position to nearest passable grid cell,
            # biased toward destination when multiple candidates exist.
            sg = agent_to_gridmap(agent.position, gridmap, resolution,
                                  destination_location=agent.destination_location,
                                  centroid=False)
            # Goal: snap destination to nearest passable grid cell
            gg = agent_to_gridmap(agent.destination_location, gridmap, resolution,
                                  centroid=False)
            starts[aid] = sg
            goals[aid]  = gg

        if not starts:
            return {aid: deque() for aid in self.agents}

        # --- Root CBS node: no constraints, each agent plans independently ---
        root_constraints = {aid: set() for aid in starts}
        root_paths = {}
        for aid in starts:
            p = _space_time_astar(gridmap, starts[aid], goals[aid],
                                  root_constraints[aid], self.MAX_TIME)
            root_paths[aid] = p if p is not None else []

        # Remove agents for which A* found no path (e.g. trapped against a wall).
        # They cannot participate in conflict resolution; leave their paths unchanged.
        failed_agents = [aid for aid, p in root_paths.items() if not p]
        for aid in failed_agents:
            root_paths.pop(aid)
            root_constraints.pop(aid)
            starts.pop(aid)
            goals.pop(aid)
            print("CBSPlanner: no path for agent {}, skipping in conflict resolution".format(aid))

        if not root_paths:
            return {aid: deque() for aid in self.agents}

        root_cost = sum(_path_cost(p) for p in root_paths.values())
        root_node = _CBSNode(root_constraints, root_paths, root_cost)

        open_heap = [root_node]
        nodes_expanded = 0

        while open_heap and nodes_expanded < self.MAX_CBS_NODES:
            node = heapq.heappop(open_heap)
            nodes_expanded += 1

            conflict = _find_first_conflict(node.paths)
            if conflict is None:
                # No conflicts — solution found
                return self._to_point_paths(node.paths, resolution)

            ai, aj, cx, cy, ct = conflict

            # Branch: add constraint for ai, then for aj
            for constrained_agent in (ai, aj):
                new_constraints = {}
                for aid in node.constraints:
                    new_constraints[aid] = set(node.constraints[aid])
                new_constraints[constrained_agent].add((cx, cy, ct))

                # Replan only the constrained agent
                new_paths = dict(node.paths)
                new_p = _space_time_astar(
                    gridmap,
                    starts[constrained_agent],
                    goals[constrained_agent],
                    new_constraints[constrained_agent],
                    self.MAX_TIME)

                if new_p is None:
                    continue  # this branch has no solution, prune

                new_paths[constrained_agent] = new_p
                new_cost = sum(_path_cost(p) for p in new_paths.values())
                child = _CBSNode(new_constraints, new_paths, new_cost)
                heapq.heappush(open_heap, child)

        # CBS exhausted or hit node limit — return best partial solution found
        # (paths from the last popped node, which had lowest cost)
        print "CBSPlanner: node limit reached or no solution; returning partial paths"
        if open_heap:
            best = min(open_heap)
            return self._to_point_paths(best.paths, resolution)
        return {aid: deque() for aid in self.agents}

    def _to_point_paths(self, grid_paths, resolution):
        """
        Convert {agent_id: [(gx,gy), ...]} to {agent_id: deque([Point, ...])}.

        grid_to_float(coord, resolution, centroid=False) returns (fx, fy) where
        fx = gx / resolution, matching the convention used by LayeredAStar.
        """
        result = {}
        for aid in self.agents:
            if aid in grid_paths and grid_paths[aid]:
                point_list = []
                for (gx, gy) in grid_paths[aid]:
                    fx, fy = grid_to_float((gx, gy), resolution, centroid=False)
                    point_list.append(Point(fx, fy))
                result[aid] = deque(point_list)
            else:
                result[aid] = deque()
        return result
