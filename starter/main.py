import time
from search.algorithms import State
from search.map import Map
import heapq
import getopt
import sys

def verify_path(start, goal, path, map):
    if path is None:
        return True

    if not (start == path[0]) or not (goal == path[-1]):
        return False

    for i in range(len(path) - 1):
        current = path[i]
        children = map.successors(current)
        
        contains_next = False
        for child in children:
            if child == path[i + 1]:
                contains_next = True
                break

        if not contains_next:
            return False
    return True

def dijkstra(gridded_map, start, goal):
    open_heap = []
    best_g = {}

    start.set_g(0.0)
    start.set_parent(None)
    start.set_cost(0.0)  # priority = g

    best_g[start.state_hash()] = 0.0
    heapq.heappush(open_heap, start)

    nodes_expanded = 0

    while open_heap:
        current = heapq.heappop(open_heap)
        nodes_expanded += 1

        cur_hash = current.state_hash()
        if current.get_g() != best_g.get(cur_hash, float("inf")):
            continue

        if current == goal:
            path = []
            cur = current
            while cur is not None:
                path.append(cur)
                cur = cur.get_parent()
            path.reverse()
            return path, current.get_g(), nodes_expanded

        for child in gridded_map.successors(current):
            ch = child.state_hash()
            new_g = child.get_g()  # successors already added step cost

            if new_g < best_g.get(ch, float("inf")):
                best_g[ch] = new_g
                child.set_parent(current)
                child.set_cost(new_g)
                heapq.heappush(open_heap, child)

    return None, -1, nodes_expanded

def astar(gridded_map, start, goal):
    open_heap = []
    best_g = {}

    def octile(s):
        dx = abs(s.get_x() - goal.get_x())
        dy = abs(s.get_y() - goal.get_y())
        return 1.5 * min(dx, dy) + abs(dx - dy)

    start.set_g(0.0)
    start.set_parent(None)
    start.set_cost(octile(start))

    best_g[start.state_hash()] = 0.0
    heapq.heappush(open_heap, start)

    nodes_expanded = 0

    while open_heap:
        current = heapq.heappop(open_heap)
        nodes_expanded += 1

        cur_hash = current.state_hash()
        if current.get_g() != best_g.get(cur_hash, float("inf")):
            continue

        if current == goal:
            path = []
            cur = current
            while cur is not None:
                path.append(cur)
                cur = cur.get_parent()
            path.reverse()
            return path, current.get_g(), nodes_expanded

        for child in gridded_map.successors(current):
            ch = child.state_hash()
            new_g = child.get_g()

            if new_g < best_g.get(ch, float("inf")):
                best_g[ch] = new_g
                child.set_parent(current)
                child.set_cost(new_g + octile(child))
                heapq.heappush(open_heap, child)

    return None, -1, nodes_expanded

def main():
    """
    Function for testing your A* and Dijkstra's implementation. There is no need to edit this file.
    Run it with a -help option to see the options available. 
    """
    test_instances = "test-instances/testinstances.txt"
                              
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []  
    nodes_expanded_astar = []

    time_dijkstra = []  
    time_astar = []

    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
    
    solved_all_problems = True

    for i in range(0, len(start_states)):    
        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        path, cost, expanded_diskstra = dijkstra(gridded_map, start, goal) # Replace the None, None, None with a call to Dijkstra's algorithm (done)
        time_end = time.time()
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_dijkstra.append(time_end - time_start)
        verified_path = verify_path(start, goal, path, gridded_map)

        if cost != solution_costs[i] or not verified_path:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print("Is the path correct?", verified_path)
            print()

            solved_all_problems = False

        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        path, cost, expanded_astar = astar(gridded_map, start, goal) # Replace the None, None, None with a call to A* (done)
        time_end = time.time()

        nodes_expanded_astar.append(expanded_astar)
        time_astar.append(time_end - time_start)

        verified_path = verify_path(start, goal, path, gridded_map)
        if cost != solution_costs[i] or not verified_path:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print("Is the path correct?", verified_path)
            print()

            solved_all_problems = False

    if solved_all_problems: print('The implementation successfully passed all test cases.')

    from search.plot_results import PlotResults
    plotter = PlotResults()
    plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
    plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")

if __name__ == "__main__":
    main()