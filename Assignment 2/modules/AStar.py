import numpy as np
from queue import PriorityQueue

manhattan = lambda p1, p2: np.absolute(p1[0] - p2[0]) + np.absolute(p1[1] - p2[1])
euclidian = lambda p1, p2: np.sqrt(np.power(p1[0] - p2[0], 2) + np.power(p1[1] - p2[1], 2))
offsets = [[-1, 0], [0, 1], [1, 0], [0, -1]]

def solve(map, start, goal, heuristic='manhattan'):
    h = manhattan if heuristic=='manhattan' else euclidian

    width = len(map)
    height = len(map[0])
    index = lambda p: p[0] + p[1] * width

    goal_index = index(goal)

    nodes = {}
    '''
    Priorityqueue does not allow editing priorityvalue in existing elements.
    Therefore if it finds a cheaper path to a node, it is added to the
    queue again with the new value. Therefore it will always check if the
    node was previously explored, because it could have been to the queue
    multiple times.
    '''
    open_q = PriorityQueue()

    start_index = index(start)
    nodes[start_index] = {"p": start, "g": 0, "f": h(start, goal), "parent": None, "closed": False}
    open_q.put((nodes[start_index]["f"], start_index))

    while not open_q.empty():
        current_index = open_q.get()[1]

        #Node already explored
        if nodes[current_index]["closed"]:
            continue
        
        #Check if current node is goal
        if current_index == goal_index:
            print("DONE!")
            return 0
        
        current_p = nodes[current_index]["p"]

        #Check every neighbor
        for offset in offsets:
            neighbor_p = np.add(current_p, offset)
            
            #Check if point is out of bounds
            if neighbor_p[0] < 0 or neighbor_p[0] > width or neighbor_p[1] < 0 or neighbor_p[1] > height:
                continue

            step_cost = map[neighbor_p[0]][neighbor_p[1]]
            #Check if point is a wall
            if step_cost < 0:
                continue

            g = nodes[current_index]["g"] + step_cost

            neighbor_index = index(neighbor_p)

            #Better path already exists, skip
            if nodes.get(neighbor_index) is not None and g > nodes[neighbor_index]["g"]:
                continue

            closed = True if nodes.get(neighbor_index) is not None and nodes[neighbor_index]["closed"] else False

            nodes[neighbor_index] = {"p": neighbor_p, "g": g, "f": g + h(neighbor_p, goal), "parent": current_index, "closed": closed}
            
            if not closed:
                open_q.put((nodes[neighbor_index]["f"], neighbor_index))
        
        nodes[current_index]["closed"] = True