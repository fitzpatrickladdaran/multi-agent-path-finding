import time as timer
import heapq
import random
import copy
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision).
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep.
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    t_max = max(len(path1), len(path2))
    first_collision = []
    
    ### for every location (up to the maximum number of locations in both paths)...
    for t in range(t_max - 1):
        
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)
        loc1_next = get_location(path1, t + 1) 
        loc2_next = get_location(path2, t + 1)
        
        ### vertex collision check 
        if loc1 == loc2:
            first_collision.append({'loc': [loc1], 'timestep': t})
            return first_collision
        
        ### edge collision check
        if loc1_next == loc2 and loc2_next == loc1: 
            first_collision.append({'loc': [loc1, loc2], 'timestep': t + 1})
            return first_collision
            
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    
    all_collisions = []
    i = 0
    j = 0
    
    while i < len(paths) - 1:
        
        collision = detect_collision(paths[i], paths[j + 1])
        j += 1
        
        if collision != None: all_collisions.append({'a1': i, 'a2': j, 'loc': collision[0]['loc'], 'timestep': collision[0]['timestep']})
        
        if j == len(paths) - 1: 
            i += 1
            j = i
        
    return all_collisions
    
def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    constraints = []
    assert len(collision['loc']) == 1 or len(collision['loc']) == 2 
    
    ### creating vertex constraints
    if (len(collision['loc']) == 1):
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
    
    ### creating edge constraints
    else:
        constraints.append({'agent': collision['a1'], 'loc': [collision['loc'][0], collision['loc'][1]], 'timestep': collision['timestep'], 'positive': False})
        constraints.append({'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep': collision['timestep'], 'positive': False})
        
    return constraints
    

def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    
    constraints = []
    agent = random.randint(0, 1)
    assert len(collision['loc']) == 1 or len(collision['loc']) == 2 
    
    ### creating positive vertex constraints
    if len(collision['loc']) == 1:
    
        if agent == 1:
            constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True})
            constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
        else:
            constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
            constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True})
        
    ### creating positive edge constraints
    else:
       
        if agent == 1:
            constraints.append({'agent': collision['a1'], 'loc': [collision['loc'][0], collision['loc'][1]], 'timestep': collision['timestep'], 'positive': True})
            constraints.append({'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep': collision['timestep'], 'positive': False})
        else:
            constraints.append({'agent': collision['a1'], 'loc': [collision['loc'][0], collision['loc'][1]], 'timestep': collision['timestep'], 'positive': False})
            constraints.append({'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep': collision['timestep'], 'positive': True})
    
    return constraints


### return: list of agents that violate a given positive constraint
def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
        no_sol = 0                                                          # track if no solution is found
        violating_agents = []
        
        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            
            p = self.pop_node()

            if len(p['collisions']) == 0: 
                self.print_results(p)
                return p['paths']

            collision = p['collisions'][0]
            constraints = disjoint_splitting(collision) if disjoint == True else standard_splitting(collision)
            no_path = 0
            
            for constraint in constraints:
        
                p_path = copy.deepcopy(p['paths'])
                p_constraints = copy.deepcopy(p['constraints'])
                
                q = {'cost': [],
                    'constraints': p_constraints + [constraint],
                    'paths': p_path,
                    'collisions': []}
                
                a_i = constraint['agent']
                path = a_star(self.my_map, self.starts[a_i], self.goals[a_i], self.heuristics[a_i], a_i, q['constraints'])
                
                if path != None:
                
                    if disjoint == True and constraint['positive'] == True: violating_agents = paths_violate_constraint(constraint, q['paths'])
                    
                    if len(violating_agents) > 0:
                        
                        for i in violating_agents:
                        
                            q['constraints'].append({'agent': i, 'loc': constraint['loc'], 'timestep': constraint['timestep'], 'positive': False})
                            new_path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, q['constraints'])
                            if new_path == None: no_path = 1
                            q['paths'][i] = new_path
                    
                    if no_path == 1: break
                    q['paths'][a_i] = path
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    self.push_node(q)
                    
                else: no_sol = 1                                            # no solution
                
                if no_sol == 1: break
                
            if no_sol == 1: break
                
        raise BaseException('No solutions') 
       
                

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
