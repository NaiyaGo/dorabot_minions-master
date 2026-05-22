# -*- coding: utf-8 -*-
"""
@Copyright Dorabot Inc.
@date : 2018-07
@author : xiaoyu.ge@dorabot.com
@brief : emulates a central server that deploys a task manager and a multiple
path finding algorithm
the design philosophy is to minimise the computation on the server side: we
let agent compute as much as it can.
"""
import copy
from collections import deque
from task_managers.task_manager import TaskMananger
from task_managers.naive_task_manager import NaiveTaskManager
from agents.agent_state_machine import AgentState


class CongestionDetector(object):
    """
    Detects agents that are stuck (congested) by tracking their position
    over a sliding window.  An agent is considered congested when its total
    displacement over the last WINDOW steps is below THRESHOLD while it still
    has an unfinished destination.

    Called once per simulator step via Server.tick().
    """
    WINDOW    = 60    # steps (~1 second at 60 Hz)
    THRESHOLD = 0.01  # squared-distance threshold (0.1 m)^2

    def __init__(self):
        # {agent_id: deque of (x, y) positions, max length = WINDOW}
        self._history = {}

    def update(self, agents):
        """
        Update position history for all agents.

        Parameters
        ----------
        agents : list of Agent objects

        Returns
        -------
        list of Agent objects that are currently congested
        """
        congested = []
        for agent in agents:
            aid = agent.id
            if aid not in self._history:
                self._history[aid] = deque(maxlen=self.WINDOW)
            self._history[aid].append((agent.position.x, agent.position.y))

            if not agent.has_destination():
                continue
            if len(self._history[aid]) < self.WINDOW:
                continue  # not enough history yet

            oldest = self._history[aid][0]
            newest = self._history[aid][-1]
            dx = newest[0] - oldest[0]
            dy = newest[1] - oldest[1]
            if dx * dx + dy * dy < self.THRESHOLD:
                congested.append(agent)

        return congested


class Server:
    # Minimum number of congested agents needed to trigger a coordinated replan
    COORDINATED_REPLAN_TRIGGER_COUNT = 2
    # Cooldown steps between coordinated replans (avoid re-triggering every step)
    COORDINATED_REPLAN_COOLDOWN = 120  # ~2 seconds at 60 Hz
    # Wall-stuck rescue: minimum near-zero speed window before triggering a pulse
    RESCUE_STUCK_WINDOW = 30      # steps of near-zero speed required
    RESCUE_SPEED_THRESHOLD = 0.05 # linear_velocity magnitude below this = "stuck"
    RESCUE_PULSE_TICKS = 20       # how long to emit the pulse
    RESCUE_COOLDOWN_TICKS = 60    # min gap between pulses for the same agent

    def __init__(self, environment, agents):
        # it is up to server to select which tast manager to use
        self.agents = agents # server connect to agent

        self.task_manager = NaiveTaskManager(environment=environment, agents=agents)
        self.agents_state = {agent.id: dict() for agent in agents}
        self.loading_ports = environment.loading_ports.values()
        self.unloading_ports = environment.unloading_ports.values()
        self.longest_squared_distance = environment.width_in_meters * environment.width_in_meters + environment.height_in_meters * environment.height_in_meters

        self.multiagent_global_planners = []
        self._congestion_detector = CongestionDetector()
        self._replan_cooldown_counter = 0  # steps remaining before coordinated replan can fire again
        self._environment = environment    # kept for grid access in rescue logic
        # Wall-stuck rescue bookkeeping: per-agent sliding counters
        self._stuck_counter = {agent.id: 0 for agent in agents}         # consecutive near-zero-speed ticks
        self._rescue_cooldown = {agent.id: 0 for agent in agents}       # ticks until this agent can be rescued again
        # {id(ma_planner): (step_counter, solution_dict)} — one ECBS solve per step
        self._path_cache = {}

    def get_loading_task(self, agent):
        port = self.__choose_loading_port(agent)
        task = self.task_manager.next_loading_task(agent, port)
        self.update_data(agent)
        return task

    def get_unloading_task(self, agent, item):
        task = self.task_manager.get_unloading_task(agent, item)
        self.update_data(agent)
        return task

    def update_data(self, agent):
        self.agents_state[agent.id]['task'] = agent.task
        self.agents_state[agent.id]['state'] = agent.state

    def tick(self, simulator):
        """
        Called once per simulator step (from simulator.step()) after all agents
        have executed observe/plan/act.

        Detects congested agents and triggers a coordinated ECBS replan on the
        congested subset.  Replanned paths are distributed via
        sequence_of_poses and agents are put into strict_path (guided) mode so
        VirtualForcePlanner follows the waypoints sequentially.

        """
        # --- Wall-stuck rescue pulse ---
        # If an agent's body has slipped into a wall cell after a collision,
        # the global planner returns no path (agent_to_gridmap's spiral
        # fallback helps, but sometimes even the ring-5 search fails), and
        # the local planner keeps pushing it harder into the wall.
        # Detect this by watching for agents with an unfinished destination
        # whose linear_velocity has been near zero for several ticks, then
        # emit a short pulse toward the nearest passable grid cell.
        self._detect_and_rescue_stuck_agents()

        # --- Restore agents that have finished their guided strict-path walk ---
        # strict_path=True means the agent is executing a coordinated path.
        # When sequence_of_poses is exhausted, restore normal (sliding-window) mode.
        for agent in self.agents:
            if getattr(agent, 'strict_path', False) and not agent.sequence_of_poses:
                agent.strict_path = False
                agent.strict_entry_counter = 0
                agent.entry_velocity = (0.0, 0.0)
                agent.guided_group_id = None
                agent.goal_changed = True   # trigger fresh ECBS replan from current position
                print("Server.tick: agent {} guided path done, restored normal mode".format(agent.id))

        # Tick down cooldown
        if self._replan_cooldown_counter > 0:
            self._replan_cooldown_counter -= 1
            return

        congested = self._congestion_detector.update(self.agents)
        if len(congested) < self.COORDINATED_REPLAN_TRIGGER_COUNT:
            return

        print("Server.tick: {} agents congested, triggering coordinated replan".format(len(congested)))

        # Lazy-import to avoid circular imports at module load time
        from multiagent_global_planners.ecbs_planner import ECBSPlanner
        from local_planners.virtual_force_planner import BRAKE_STEPS

        # Reuse the ECBSPlanner instance registered at startup (by
        # simulator.set_global_planner).  ECBS is used for both normal and
        # congestion-resolution paths — no separate CBS implementation is
        # maintained.  Keeping a single coordinator avoids per-trigger
        # construction cost and focal-heap reuse makes repeated calls cheap.
        ecbs = None
        for planner in self.multiagent_global_planners:
            if isinstance(planner, ECBSPlanner):
                ecbs = planner
                break

        if ecbs is None:
            print("Server.tick: no ECBSPlanner registered, skipping coordinated replan")
            self._replan_cooldown_counter = self.COORDINATED_REPLAN_COOLDOWN
            return

        # Temporarily narrow the ECBS coordination set to just the congested
        # agents.  Non-congested agents are not touched — they keep their
        # current VirtualForce-normal behaviour.
        saved_agents = ecbs.agents
        ecbs.agents = {agent.id: agent for agent in congested}
        try:
            solution = ecbs.compute_path()  # {agent_id: deque([Point, ...])}
        finally:
            ecbs.agents = saved_agents

        # Unique id for this coordination batch: all agents receiving a path
        # in this tick share it, so guided-mode repulsion recognises them as
        # same-group and applies the weakened force.
        batch_id = id(solution)

        # Distribute paths; switch to strict sequential waypoint following.
        # goal_changed stays False so naive_agent.plan() does NOT call ECBS
        # again and overwrite the path we just assigned.
        for agent in congested:
            aid = agent.id
            if aid in solution and len(solution[aid]) > 0:
                agent.sequence_of_poses = solution[aid]
                agent.goal_changed = False
                agent.replan = False
                agent.strict_path = True              # VirtualForcePlanner: guided mode
                agent.strict_entry_counter = BRAKE_STEPS
                agent.entry_velocity = agent.linear_velocity
                agent.guided_group_id = batch_id

        self._replan_cooldown_counter = self.COORDINATED_REPLAN_COOLDOWN

    def _detect_and_rescue_stuck_agents(self):
        """
        Watch for agents whose velocity has been near zero for several ticks
        while they still have a destination — the classic "nudged into a wall
        cell" failure mode — and emit an outward pulse toward the nearest
        passable grid cell.  The pulse is picked up by VirtualForcePlanner's
        rescue branch which overrides normal/guided modes for the duration.
        """
        from math import sqrt
        from representation.gridmap_a import GridmapWithNeighbors
        from representation.float_to_grid import grid_to_float

        gridmap = GridmapWithNeighbors(self._environment.static_gridmap)
        resolution = gridmap.resolution

        for agent in self.agents:
            aid = agent.id

            # Tick down rescue cooldown regardless of stuck status.
            if self._rescue_cooldown.get(aid, 0) > 0:
                self._rescue_cooldown[aid] -= 1

            # Skip if agent is actively receiving a rescue pulse, is idle, or
            # has no destination.
            if getattr(agent, 'rescue_ticks', 0) > 0:
                continue
            if not agent.has_destination():
                self._stuck_counter[aid] = 0
                continue
            if self._rescue_cooldown.get(aid, 0) > 0:
                continue

            vx, vy = agent.linear_velocity
            speed = sqrt(vx * vx + vy * vy)
            if speed < self.RESCUE_SPEED_THRESHOLD:
                self._stuck_counter[aid] = self._stuck_counter.get(aid, 0) + 1
            else:
                self._stuck_counter[aid] = 0
                continue

            if self._stuck_counter[aid] < self.RESCUE_STUCK_WINDOW:
                continue

            # Agent has been effectively stationary long enough — find the
            # nearest passable grid cell and pulse toward it.
            px = agent.position.x * resolution
            py = agent.position.y * resolution
            rx, ry = int(round(px)), int(round(py))
            nearest = None
            best_d2 = None
            for r in range(1, 6):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        if max(abs(dx), abs(dy)) != r:
                            continue
                        c = (rx + dx, ry + dy)
                        if gridmap.in_bounds(c) and gridmap.passable(c):
                            d2 = dx * dx + dy * dy
                            if best_d2 is None or d2 < best_d2:
                                best_d2 = d2
                                nearest = c
                if nearest is not None:
                    break

            if nearest is None:
                self._stuck_counter[aid] = 0
                self._rescue_cooldown[aid] = self.RESCUE_COOLDOWN_TICKS
                continue

            fx, fy = grid_to_float(nearest, resolution, centroid=True)
            dxw = fx - agent.position.x
            dyw = fy - agent.position.y
            norm = sqrt(dxw * dxw + dyw * dyw)
            if norm < 1e-6:
                self._stuck_counter[aid] = 0
                self._rescue_cooldown[aid] = self.RESCUE_COOLDOWN_TICKS
                continue
            pulse_speed = agent.cruise_speed * 0.5
            agent.rescue_velocity = (dxw / norm * pulse_speed,
                                     dyw / norm * pulse_speed)
            agent.rescue_ticks = self.RESCUE_PULSE_TICKS
            self._stuck_counter[aid] = 0
            self._rescue_cooldown[aid] = self.RESCUE_COOLDOWN_TICKS
            print("Server.tick: rescuing stuck agent {} -> cell {}".format(aid, nearest))

    # calls for multiagent_global_planner
    def add_multiagent_local_planner(self, ma_planner):
        self.multiagent_global_planners.append(ma_planner)
    
    def remove_multiagent_local_planner(self, ma_planner):
        for i in range(len(self.multiagent_global_planners)):
            if ma_planner == self.multiagent_global_planners[i]:
                self.multiagent_global_planners.pop(i)

    def request_multiagent_global_planner_compute_path(self, agent):
        '''Agent sends a request to server, asking for multiagent_global_planner to compute path for itself.

        Result is cached per simulator step: N agents with goal_changed in the same step
        trigger only one ECBS solve. Each agent's plan() still receives its own path slice
        and manages its own state — no side effects on other agents.
        '''
        from simulator import Simulator
        current_step = Simulator.step_counter
        for ma_planner in self.multiagent_global_planners:
            if agent.id not in ma_planner.agents:
                continue
            pid = id(ma_planner)
            cached_step, cached_paths = self._path_cache.get(pid, (-1, None))
            if cached_step == current_step and cached_paths is not None:
                return cached_paths.get(agent.id, [])
            solution_paths_dict = ma_planner.compute_path()
            self._path_cache[pid] = (current_step, solution_paths_dict)
            return solution_paths_dict.get(agent.id, [])
        print("Cannot find a multiagent global planner which is in charge of agent {}".format(agent.id))
        return []
            
    def collect_agents_info(self, agents_dict):
        """return dictionary of {agent.id: AgentAbstraction} for multiagent planner"""
        agents_abstraction = {}
        for agent in agents_dict.values():
            if agent.has_destination():
                agent.observe(agent.ray_length_list) # refresh localization
                agents_abstraction[agent.id] = AgentAbstraction(agent)
        return agents_abstraction

    def refresh_agents_path(self, solution_paths_dict):
        """a work around for pass the paths to all agents"""
        for agent_id in solution_paths_dict.keys():
            agent = self.agents[agent_id]
            if agent.has_destination():
                agent.sequence_of_poses = solution_paths_dict[agent_id]
    
    def __choose_loading_port(self, agent):
        
        def evaluate_port(loading_port):
            counter1 = 0
            counter2 = 0
            for i in self.agents_state:
                if 'task' in self.agents_state[i] and self.agents_state[i]['task'].port == loading_port:
                    counter1 +=1
                    if self.agents_state[i]['state'] in [AgentState.QUEUING, AgentState.PREQUEUE]:
                        counter2 +=1       
            return -(counter1 + 3*counter2)\
                    -0.3*loading_port.location.squared_distance(agent.position)/self.longest_squared_distance
        
        port = max(self.loading_ports, key = evaluate_port)
        return port

class AgentAbstraction(object):
    """get necessary information of an agent for multiagent planner"""
    def __init__(self, agent):
        self.id = agent.id
        self.position = agent.position.copy()
        self.radius = agent.shape.get_radius()
        self.destination_location = agent.destination_location
        self.sequence_of_poses = copy.deepcopy(agent.sequence_of_poses)
        if agent.current_local_planner:
            self.current_local_planner = agent.current_local_planner
        else:
            self.current_local_planner = agent.local_planner[0]
        self.root_linear_velocity = agent.linear_velocity # tuple
        self.root_angular_velocity = agent.angular_velocity # scala
        self.perception_module = agent.perception_module
        self.root_history_ray_length_list = agent.history_ray_length_list[:]
        self.root_history_ray_point_list = agent.history_ray_point_list[:]

    def __str__(self):
        return "agent {}; position {}; destination {};\nsequence_of_poses {};\nlocal planner {}; linear velocity {}; angular velocity {}"\
            .format(self.id, self.position, self.destination_location, 
            [str(pose) for pose in self.sequence_of_poses],
            self.current_local_planner, self.root_linear_velocity, self.root_angular_velocity)