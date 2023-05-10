import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """
        my_map      - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        map_size = -1                                                                       # track number of valid spaces in environment
        last_path_max_length = 0                                                            # track path length of previous agent
           
        ### count the number of valid spaces in the environment
        for l in self.my_map:
           for k in l:
              if k == False: map_size += 1
        
        max_timestep = map_size                                                            # for first agent, max timestep should be number 
                                                                                           # of valid spaces in environment
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            
            counter = 0                                                                        # track time step of constraints
            if len(path) - 1 < last_path_max_length: max_timestep = len(path) - 1 + map_size   # upper bound on path length for an agent,
                                                                                               # based on (longest) path length of higher priority 
                                                                                               # agents + valid space in environment;
                                                                                               # - 1 because starting location is counted in len()
                                                                                           
            ### for every coordinate...
            for j in path:
                
                ### for every (other) agent(s)...
                for k in range(self.num_of_agents):
                    
                    ### if the current iterated pair of agents are the same...
                    if i == k: continue
                        
                    ### if the current iterated pair of agents are different...
                    else: 
                        
                        ### handling vertex constraints
                        constraints.append({'agent': k, 'loc': [j], 'timestep': counter})
                        
                        ### handling edge constraints: if coordinate is not i'th's goal location
                        if j != path[-1]: 
                            constraints.append({'agent': k, 'loc': [path[counter + 1], path[counter]], 'timestep': counter + 1})
                            
                        ### handling additional constraints: if coordinate is i'th's goal location
                        else:
                            counter2 = counter                                             # track number of constraints to make for other agents 
                                                                                           # after i'th agent is at goal location
                                                                                           
                            ### goal constraints: locations where an agent cannot pass because agent i is already at its goal location
                            while counter2 < max_timestep:
                                constraints.append({'agent': k, 'loc': [j], 'timestep': counter2})
                                counter2 += 1
                            
                            ### if a goal location of a higher priority agent blocks the goal location of a lower priority agent    
                            if max_timestep < counter2:
                                raise BaseException('No solutions')
                                
                            
                counter += 1                                                               # increment time step for next coordinate
            last_path_max_length = len(path) - 1                                           # - 1 because starting location is counted in len()
            ##############################

        self.CPU_time = timer.time() - start_time
        print("\n Found a solution! \n")        
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
