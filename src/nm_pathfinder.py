# CMPM 146 P1: Navmesh Pathfinding
# Team Member:
# Zexuan Li
# Malachi Maldonado
from queue import Queue, PriorityQueue
from math import inf, sqrt
from heapq import heappop, heappush

def find_point(point,box):
    if point[0] <= box[0]:
        x = box[0]
    elif point[0] >= box[1]:
        x = box[1]
    else:
        x = point[0]

    if point[1] <= box[2]:
        y = box[2]
    elif point[1] >= box[3]:
        y = box[3]
    else:
        y = point[1]

    return x, y

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
    
    print("source point: ", source_point)
    print("source Box: ",sourceBox)
    print("destination point: ", destination_point)
    print("Destination Box: ",destinationBox)

    # # step 2 Implement the simplest complete search algorithm you can
    # # a simple BFS   
    # frontier = Queue()
    # frontier.put(sourceBox)
    # came_from = dict()
    # came_from[sourceBox] = None

    # detail_points[sourceBox] = source_point

    # while not frontier.empty():
    #     current = frontier.get()
    #     boxes.append(current)
    #     if current == destinationBox: 
    #         break

    #     for next in mesh['adj'][current]:
    #         if next not in came_from:
    #             # find detail point in 'next'
    #             if detail_points[current][0] <= next[0]:
    #                 detail_x = next[0]
    #             elif detail_points[current][0] >= next[1]:
    #                 detail_x = next[1]
    #             else:
    #                 detail_x = detail_points[current][0]

    #             if detail_points[current][1] <= next[2]:
    #                 detail_y = next[2]
    #             elif detail_points[current][1] >= next[3]:
    #                 detail_y = next[3]
    #             else:
    #                 detail_y = detail_points[current][1]

    #             detail_points[next] = detail_x, detail_y

    #             frontier.put(next)
    #             came_from[next] = current

    # step 4 Implement the A* algorithm
    frontier = PriorityQueue()
    #A dictionary with key as the box's coordinates stores the following content
    #           [0]box's coordinates,[1]detail_point,[2]goal,       [3]prev_box, [4]cost so far
    forward = {sourceBox: [sourceBox, source_point, destination_point, None, 0]}
    frontier.put((0,sourceBox, 0))
    # Determine if the path was found 
    find = False
    # step 5: make the A* algorithm bidirectional
    #         Making a new dictionary to search from the destination
    backward = {destinationBox: [destinationBox, destination_point, source_point, None, 0]}
    frontier.put((0,destinationBox, 1))

    while not frontier.empty():
        #A temporary variable, internally the priority, box, and goal (0 for dest, 1 for source)
        temp = frontier.get()
        #Extraction Box
        currentBox = temp[1]
        boxes.append(currentBox)

        #check if the current box has been seen by the other search. This means we're done!
        if  temp[2] == 0 and temp[1] in backward:
            print("forward met backward")

            #Found the path
            find = True
            
            curr_box = currentBox

            #Traverse to the beginning
            while curr_box != sourceBox:
                path.append(forward[curr_box][1])
                curr_box = forward[curr_box][3]
            
            path.append(source_point)

            #Traverse to the end
            curr_box = currentBox
            while curr_box != destinationBox:
                path.insert(0,backward[curr_box][1])   
                curr_box = backward[curr_box][3]
            
            path.insert(0,destination_point)
            path.reverse()
            break
        elif temp[2] == 1 and temp[1] in forward:
            print("backward met forward")

            #Found the path
            find = True

            curr_box = currentBox

            #Traverse to the beginning
            while curr_box != sourceBox:
                path.append(forward[curr_box][1])
                curr_box = forward[curr_box][3]
            
            path.append(source_point)

            #Traverse to the end
            curr_box = currentBox
            while curr_box != destinationBox:
                path.insert(0,backward[curr_box][1])   
                curr_box = backward[curr_box][3]
            
            path.insert(0,destination_point)
            path.reverse()
            
            break


        #Extract the box dictionary; if goal is destination, get box info from forward table.
        #Otherwise, get it from backward table.
        if temp[2] == 0: 
            forwardCurrent = forward[currentBox]
        else:
            forwardCurrent = backward[currentBox]
        #Record the boxes found by the algorithm for each box
        
        
        #if goal box is reached, break, for either direction
        if temp[2] == 0 and currentBox == destinationBox: 
            #Found the path
            find = True
            #Record the end point path
            path.append(destination_point)
            curr_box = destinationBox
            #Traverse to the beginning
            while curr_box != sourceBox:
                path.append(forward[curr_box][1])
                curr_box = forward[curr_box][3]
            #Record the start point path and Rotate the path to the correct order
            path.append(source_point)
            path.reverse()
            break
        elif temp[2] == 1 and currentBox == sourceBox:
            #Found the path
            find = True
            #Record the start point path
            path.append(source_point)
            curr_box = sourceBox
            #Traverse to the end
            while curr_box != destinationBox:
                path.append(backward[curr_box][1])
                curr_box = backward[curr_box][3]
            #Record the end point path
            path.append(destination_point)
            break

        #Traversing the box's neighbors
        for forwardNext in mesh['adj'][currentBox]:
            #Finding the point of connection to neighbors
            forwardNext_point = find_point(forwardCurrent[1],forwardNext)
            #Calculate the cost, the current cost + the cost from the current point to the next point
            forwardNext_cost = forwardCurrent[4] + Euclidean_distances(forwardNext_point, forwardCurrent[1])

            #If unrecorded or less costly than past records
            if temp[2] == 0:
                if forwardNext not in forward or forwardNext_cost < forward[forwardNext][4]:
                    #Update dictionary
                    forward[forwardNext] = [forwardNext, forwardNext_point, destination_point, currentBox, forwardNext_cost]
                    #Calculate the cost of this path to the end point
                    priority = forwardNext_cost + Euclidean_distances(forwardNext_point, destination_point)
                    #Put in priority queue
                    frontier.put((priority,forwardNext, 0))
            elif temp[2] == 1:
                if forwardNext not in backward or forwardNext_cost < backward[forwardNext][4]:
                    #Update dictionary
                    backward[forwardNext] = [forwardNext, forwardNext_point, source_point, currentBox, forwardNext_cost]
                    #Calculate the cost of this path to the source point
                    priority = forwardNext_cost + Euclidean_distances(forwardNext_point, source_point)
                    #Put in priority queue
                    frontier.put((priority,forwardNext, 1))
                
    if find == False:
        print("No path!")



    # if destinationBox in forward:

    #     #Record the end point path
    #     path.append(destination_point)

    #     curr_box = destinationBox

    #     #Traverse to the beginning
    #     while curr_box != sourceBox:
    #         path.append(forward[curr_box][1])
    #         curr_box = forward[curr_box][3]

    #     #Record the end point path and Rotate the path to the correct order
    #     path.append(source_point)
    #     path.reverse()
    # else:
    #     print("No path!")


    print(path)
    return path, boxes
