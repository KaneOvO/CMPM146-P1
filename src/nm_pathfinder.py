from queue import Queue
from math import inf, sqrt
from heapq import heappop, heappush

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
    segment_number = 0

    # step 1 Identify the source and destination boxes.
    sourceBox = None
    destinationBox = None

    for box in mesh['boxes']:
        x1, x2, y1, y2 = box
        p1, p2 = source_point
        p3, p4 = destination_point

        if(x1 <= p1 and x2 >= p1 and y1 <= p2 and y2 >= p2):
            sourceBox = box
            print(sourceBox)
        if(x1 <= p3 and x2 >= p3 and y1 <= p4 and y2 >= p4):
            destinationBox = box
            print(destinationBox)



    # step 2 Implement the simplest complete search algorithm you can
    # a simple BFS   
    frontier = Queue()
    frontier.put(sourceBox)
    came_from = dict()
    came_from[sourceBox] = None

    #detail_points[segment_number] = source_point
    #segment_number+=1

    while not frontier.empty():
        current = frontier.get()
        boxes.append(current)
        if current == destinationBox: 
            break

        for next in mesh['adj'][current]:
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current

    if destinationBox in came_from:
        print("Found path!")
        print(path)
    else:
        print("No path!")
    




    return path, boxes
