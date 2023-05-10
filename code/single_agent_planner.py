import heapq

def move(loc, dir):
    
    ### south, east, north, west
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):

    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):

    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    constraint_table = []
    
    ### for every constraint in the list of constraints (dictionaries) passed...
    for constraint in constraints: 
    	
    	### adding key 'positive' to every constraint
    	if 'positive' not in constraint.keys(): constraint['positive'] = False
    	
    	### if the constraint corresponds to the agent passed...
    	if constraint['agent'] == agent: constraint_table.append(constraint)
    	
    ### sorting constraints by timestep
    results = sorted(constraint_table, key = lambda c: constraint['timestep'])
    return results
    
    
def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):

    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    ### for every constraint...
    for constraint in constraint_table:
        
        ### if a constraint exists at some time...
        if next_time == constraint['timestep']:
            
            ### handling edge constraints...
            if len(constraint['loc']) > 1:
                if curr_loc == constraint['loc'][0] and next_loc == constraint['loc'][1]: return True

            ### handling vertex constraints...
            elif next_loc == constraint['loc'][0]: return True
            
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

### returns the dimensions [x,y] of the map (environment)
def map_dims(my_map):

    cols = 0
    rows = 0

    for i in my_map: rows += 1
    for i in my_map[0]: cols += 1
    
    dims = [rows, cols]
    return dims

### boolean: check if a location is within the map's environment
def within_map(dims, loc):
    
    if loc[0] >= 0 and loc[0] < dims[0] and loc[1] >= 0 and loc[1] < dims[1]: return True
    return False


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):

    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []                                                            # open list
    closed_list = dict()                                                      # closed list
    earliest_goal_timestep = 0                                                # earliest goal timestep
    h_value = h_values[start_loc]                                             # h-value
    constraint_table = build_constraint_table(constraints, agent)             # constraint table
    lock = 0                                                                  # locks goal test condition
    n_constraints = len(constraint_table)                                     # number of constraints
    
    ### generating root node
    root = {'loc': start_loc, 
    	   'g_val': 0, 
    	   'h_val': h_value, 
    	   'parent': None, 
    	   'timestep': 0,
    	   'positive': False}
    
    ### pushing root node into open list
    push_node(open_list, root)
    
    ### adding root node into closed list                                                
    closed_list[(root['loc'], root['timestep'])] = root
    
    ### get dimensions of map (for Section 3.4)
    dims = map_dims(my_map)
    
    ### while there are still nodes to explore...
    while len(open_list) > 0:
        
        ### get next node to explore
        curr = pop_node(open_list)
        
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        
        ### if there are constraints...
        if n_constraints > 0:
            
            ### if solution has been found and time matches the latest constraint's time step...
            if curr['loc'] == goal_loc and lock == 1: return get_path(curr)
        
            ### unlock if: 1 time step before latest (constraint) time step (minimize animation run time)
            if (curr['timestep'] + 1) == constraint_table[-1]['timestep']: lock = 1
            
        ### if there are no constraints...
        else:
            
            ### if solution has been found...
            if curr['loc'] == goal_loc: return get_path(curr)
        #############################
            
        ### for every direction (south, east, north, west, wait)...
        for dir in range(5):
            
            ### child location: if dir == 4, parent's current location (wait)
            ###                 if dir != 4, from parent's location, move towards a direction (south, east, north, west)
            child_loc = curr['loc'] if dir == 4 else move(curr['loc'], dir)
            
            if within_map(dims, child_loc):
            
                ### if child node's location is invalid (in a barrier/wall, or violates a constraint)...
                if my_map[child_loc[0]][child_loc[1]] or is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table): continue
                
            else: continue
            
            ### generate child node based on new location and next time step
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1,
                    'positive': False}
                    
            ### if generated child with similar location and time step has already been generated...
            if (child['loc'], child['timestep']) in closed_list:
            
                existing_node = closed_list[(child['loc'], child['timestep'])]
                
                ### if newly generated child node costs less than previously generated node...
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            
            ### if generated child with similar location and time step has not been generated...
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
