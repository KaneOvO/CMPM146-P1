from queue import Queue, PriorityQueue
from math import inf, sqrt
from heapq import heappop, heappush

def Euclidean_distances(p1, p2):
    distance = 0

    for i in range(len(p1)):
        distance += (p1[i] - p2[i]) ** 2
    
    distance = sqrt(distance)
    
    return distance

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = []
    detail_points = {}
    point_number = 0

    # step 1 Identify the source and destination boxes.
    sourceBox = None
    destinationBox = None

    for box in mesh['boxes']:
        x1, x2, y1, y2 = box
        p1, p2 = source_point
        p3, p4 = destination_point

        if(x1 <= p1 and x2 >= p1 and y1 <= p2 and y2 >= p2):
            sourceBox = box
        if(x1 <= p3 and x2 >= p3 and y1 <= p4 and y2 >= p4):
            destinationBox = box
    
    if sourceBox == None or destinationBox == None:
        print("No path!")
        return path, boxes


    # step 2 Implement the simplest complete search algorithm you can
    # a simple BFS   
    # frontier = Queue()
    # frontier.put(sourceBox)
    # came_from = dict()
    # came_from[sourceBox] = None

    # detail_points[point_number] = source_point[0], source_point[1]
    # path.append(source_point)
    # print(detail_points[point_number])
    # while not frontier.empty():
    #     current = frontier.get()
    #     boxes.append(current)
    #     if current == destinationBox: 
    #         break

    #     for next in mesh['adj'][current]:
    #         if next not in came_from:
    #             #find detail point in 'next'
    #             if detail_points[point_number][0] <= next[0]:
    #                 detail_x = next[0]
    #             elif detail_points[point_number][0] >= next[1]:
    #                 detail_x = next[1]
    #             else:
    #                 detail_x = detail_points[point_number][0]

    #             if detail_points[point_number][1] <= next[2]:
    #                 detail_y = next[2]
    #             elif detail_points[point_number][1] >= next[3]:
    #                 detail_y = next[3]
    #             else:
    #                 detail_y = detail_points[point_number][1]

    #             point_number += 1
    #             detail_points[point_number] = detail_x, detail_y

    #             frontier.put(next)
    #             came_from[next] = current

    
    frontier = PriorityQueue()
    came_from = dict()
    cost_so_far = dict()
    came_from[sourceBox] = None
    cost_so_far[sourceBox] = 0
    frontier.put((cost_so_far[sourceBox],sourceBox))

    detail_points[point_number] = source_point[0], source_point[1]
    #path.append(source_point)
    print(detail_points[point_number])

    while not frontier.empty():
        temp = frontier.get()
        print("temp:", temp)
        current = temp[1]
        cost_so_far[current] = temp[0]
        
        boxes.append(current)

        if current == destinationBox: 
            break

        for next in mesh['adj'][current]:
            #find detail point in 'next'
            if detail_points[point_number][0] <= next[0]:
                detail_x = next[0]
            elif detail_points[point_number][0] >= next[1]:
                detail_x = next[1]
            else:
                detail_x = detail_points[point_number][0]

            if detail_points[point_number][1] <= next[2]:
                detail_y = next[2]
            elif detail_points[point_number][1] >= next[3]:
                detail_y = next[3]
            else:
                detail_y = detail_points[point_number][1]

            point_number += 1
            detail_points[point_number] = detail_x, detail_y
            
            new_cost = cost_so_far[current] + Euclidean_distances(detail_points[point_number - 1], detail_points[point_number])
           

            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = Euclidean_distances(detail_points[point_number], destination_point)
                frontier.put((priority,next))
                came_from[next] = current 


    if destinationBox in came_from:
        print("Found path!")

        curr_box = destinationBox
        path.append(destination_point)
        while curr_box != sourceBox:
            path.append(detail_points[boxes.index(curr_box)])
            curr_box = came_from[curr_box]

        path.append(source_point)

    else:
        print("No path!")

    print("source point: ", source_point)
    print("source Box: ",sourceBox)
    print("destination point: ", destination_point)
    print("Destination Box: ",destinationBox)
    print(path)
    return path, boxes
