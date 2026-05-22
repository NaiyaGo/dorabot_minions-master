# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@brief : GridStepPlanner — follows CBS/ECBS grid paths by walking to cell
         centers one step at a time.

Why this exists
---------------
CBS paths are produced with centroid=False, meaning waypoints land on cell
*corners* rather than centers.  DullPlanner drives straight toward those
corner coordinates, which sit on or inside walls, causing collisions.

GridStepPlanner fixes this by shifting every waypoint to the cell center
(+0.5/resolution in both axes) on first call, then advancing along cardinal
directions only.  No repulsion forces are applied — CBS already guarantees
conflict-free paths, so reactive avoidance would only push agents off them.

State machine
-------------
  ADVANCE  move at cruise_speed toward current waypoint center
  TURN     hold near-zero velocity for TURN_STEPS ticks so Box2D can
           settle the heading before the next straight segment
  DONE     path exhausted; server.tick() will restore VirtualForcePlanner
"""
from local_planner import LocalPlanner
from geometry import Point

_ADVANCE = 0
_TURN    = 1
_DONE    = 2

# Cardinal unit vectors (East, North, West, South)
_CARDINALS = [
    (1.0,  0.0),
    (0.0,  1.0),
    (-1.0, 0.0),
    (0.0, -1.0),
]


def _nearest_cardinal(dx, dy):
    """Return the cardinal direction (vx, vy) closest to (dx, dy) via dot product."""
    best, best_dot = _CARDINALS[0], -2.0
    for c in _CARDINALS:
        dot = dx * c[0] + dy * c[1]
        if dot > best_dot:
            best_dot = dot
            best = c
    return best


class GridStepPlanner(LocalPlanner):
    """
    Local planner that follows a CBS grid path by walking to each cell center.

    Parameters (class-level, override per-instance if needed)
    ----------------------------------------------------------
    ARRIVE_THRESHOLD : float
        Distance (m) at which the agent is considered to have reached a
        waypoint and pops it from the queue.  Slightly larger than the
        default Point.arrive() threshold (0.2 m) to handle overshoot at
        cruise speed.
    TURN_STEPS : int
        Number of simulator ticks to pause at a direction change.  Gives
        Box2D time to settle the body angle before the next straight run.
    """

    ARRIVE_THRESHOLD = 0.3   # metres
    TURN_STEPS       = 3     # ticks  (~50 ms at 60 Hz)

    def __init__(self, agent):
        super(GridStepPlanner, self).__init__(agent)
        self._state       = _ADVANCE
        self._heading     = (1.0, 0.0)  # current cardinal direction
        self._turn_count  = 0
        self._initialized = False       # True after first-call coord fix

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def is_done(self):
        """True when the CBS path has been fully walked.
        server.tick() polls this to know when to restore VirtualForcePlanner."""
        return self._state == _DONE

    def compute_plan(self, position, velocity, gridmap, sensor_observation,
                     global_planner_path):
        """
        Called every simulator tick by NaiveAgent.plan().

        Parameters
        ----------
        position            : geometry.Point  — current agent position
        velocity            : (float, float)  — current linear velocity
        gridmap             : GridmapWithNeighbors (has .resolution attribute)
        sensor_observation  : ignored (no reactive avoidance)
        global_planner_path : collections.deque of geometry.Point
                              Modified in-place on first call to shift
                              waypoints to cell centers.

        Returns
        -------
        (vx, vy) : float tuple — desired linear velocity command
        """
        # --- One-time coordinate correction: corner → cell center ----------
        if not self._initialized:
            self._initialized = True
            resolution = getattr(gridmap, 'resolution', 1)
            offset = 0.5 / float(resolution)
            corrected = [Point(pt.x + offset, pt.y + offset)
                         for pt in global_planner_path]
            global_planner_path.clear()
            global_planner_path.extend(corrected)
            # Snap initial heading toward first waypoint
            if global_planner_path:
                dx = global_planner_path[0].x - position.x
                dy = global_planner_path[0].y - position.y
                self._heading = _nearest_cardinal(dx, dy)

        # --- Terminal states -----------------------------------------------
        if self._state == _DONE:
            return (0.0, 0.0)

        if not global_planner_path:
            self._state = _DONE
            return (0.0, 0.0)

        # --- TURN: pause briefly so Box2D settles the new heading ----------
        if self._state == _TURN:
            self._turn_count += 1
            vx, vy = self._heading
            if self._turn_count >= self.TURN_STEPS:
                self._turn_count = 0
                self._state = _ADVANCE
            # Return a tiny velocity in the new heading direction so that
            # simulator.py's atan2 guard sets the body angle correctly.
            return (vx * 1e-4, vy * 1e-4)

        # --- ADVANCE: walk straight toward current waypoint center ---------
        target = global_planner_path[0]
        dist   = position.distance(target)

        if dist < self.ARRIVE_THRESHOLD:
            global_planner_path.popleft()
            if not global_planner_path:
                self._state = _DONE
                return (0.0, 0.0)
            # Decide whether a direction change is needed
            next_target = global_planner_path[0]
            dx = next_target.x - position.x
            dy = next_target.y - position.y
            new_heading = _nearest_cardinal(dx, dy)
            if new_heading != self._heading:
                self._heading    = new_heading
                self._turn_count = 0
                self._state      = _TURN
                vx, vy = self._heading
                return (vx * 1e-4, vy * 1e-4)
            # Same direction — keep going without pausing
            target = global_planner_path[0]

        speed = self.agent.cruise_speed
        vx, vy = self._heading
        return (vx * speed, vy * speed)
