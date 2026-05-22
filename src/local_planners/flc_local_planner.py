# -*- coding: utf-8 -*-
from FLC_simulator import Environment, MLSHAgent
from local_planner import LocalPlanner
from rvo_planner import RVOPlanner



class FLCPlanner(LocalPlanner):
    new_agent = MLSHAgent()
    env = Environment(new_agent)

    def compute_plan(self, position, velocity, gridmap, sensor_observation, global_planner_path):
        next_destination = self.compute_sliding_window_des(global_planner_path)
        action = RVOPlanner(self.agent).compute_plan(position, velocity, gridmap, sensor_observation, global_planner_path)
        act = FLCPlanner.env.run(self.agent.ray_length_list, next_destination, [self.agent.speed, 
            self.agent.angular_velocity], self.agent.pose, action, self.agent.cruise_speed, self.agent.max_angular_velocity)
        return act

    def compute_sliding_window_des(self, global_planner_path):
        """Sliding Window — or strict sequential following when agent.strict_path is True."""
        p = self.agent.position
        if getattr(self.agent, 'strict_path', False):
            # Strict mode: follow waypoints in order, no skipping
            try:
                if p.arrive(global_planner_path[0]):
                    global_planner_path.popleft()
                goal = global_planner_path[0]
            except:
                goal = self.agent.destination_location
        else:
            try:
                goal = [x for x in global_planner_path if x.distance(p) < 2.5][-1]
            except:
                goal = self.agent.destination_location
        v_des = [goal.x, goal.y]
        v_des.append(0)
        return v_des

    def __reach(self, p1, p2, bound=0.5):
        if p1.distance(p2)< bound:
            return True
        else:
            return False


    def __norm(self, vector):
        sum_square=sum(i**2 for i in vector)
        return pow(sum_square, 0.5)+0.001