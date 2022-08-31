from __future__ import print_function
from ctypes.wintypes import HACCEL
from logging import Handler
import math
import matplotlib.pyplot as plt


def chevHeuristic(start,goal):
    #Chebyshev distance heuristic
    diagonal = min(abs(start[0]-goal[0]), abs(start[1]-goal[1])) 
    straight = abs(start[0]-goal[0]) + abs(start[1]-goal[1]) 

    return 1* diagonal  + 1*(straight-2*diagonal)

def manHeuristic(start,goal):
    #Manhattan distance heuristic
    dx = abs(start[0]-goal[0])
    dy = abs(start[1]-goal[1])
    return (dx+dy)*1

def eucHeuristic(start,goal):
    #Euclidean distance Heuristic 
    a = (start[0]-goal[0])**2
    b = (start[1]-goal[1])**2
    return  1*(math.sqrt(a + b))
def neighbourVertex(position, dimension):
    p = []

    for dx ,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:

        x = position[0] +dx
        y = position[1]+dy

        if x < 0 or x>dimension or y < 0 or y >dimension:
            continue
        p.append((x,y))
    return p

def costOfMove(x,y):
    for bar in barriers:
        if y in bar:
            return 100 #make the cost high to enter a barried
        return 1 # cost of a normal movement without barriers

def aStarMeth(start,end,typeH,dim):

    functionG = {} #Movement cost from position to start
    functionF = {}#estimated movement from start to end 
    if typeH.lower() == "manhattan":

        #the starting values
        functionG[start] = 0
        functionF[start] = manHeuristic(start,end)

    elif typeH.lower() == "chebyshev":
        functionG[start] = 0
        functionF[start] = manHeuristic(start,end)
    elif typeH.lower() == "euclidean":
        functionG[start] = 0
        functionF[start] = eucHeuristic(start,end)

    vertciesThatAreClosed = set()
    vertciesThatAreOpen = set([start])
    c_from =  {}

    while len(vertciesThatAreOpen) > 0:
        #obtain the vertex in the open list wuth the lowest F(f)
        current = None
        currentFscore = None

        for position in vertciesThatAreOpen:
            if current is None or functionF[position] < currentFscore:
                currentFscore = functionF[position]
                current = position


        #check if the goal has been reached

        if current == end:
            route = [current]
            while current in c_from:
                current = c_from[current]
                route.append(current)
            route.reverse()

            return route, functionF[end]

        vertciesThatAreOpen.remove(current)
        vertciesThatAreClosed.add(current)

        for neighbor in neighbourVertex(current,dim):
            if neighbor in vertciesThatAreClosed:
                continue
            gCand = functionG[current] + costOfMove(current,neighbor)

            if neighbor not in vertciesThatAreOpen:
                vertciesThatAreOpen.add(neighbor)
            elif gCand >= functionG[neighbor]:
                continue

            c_from[neighbor] = current
            functionG[neighbor] = gCand
            if typeH.lower() == "manhattan":
                functionH = manHeuristic(neighbor,end)

            elif typeH.lower() == "chebyshev":
                functionH = chevHeuristic(neighbor,end)
            elif typeH.lower() == "euclidean":
                functionH = eucHeuristic(neighbor,end)

            functionF[neighbor] = functionG[neighbor] +functionH

    raise RuntimeError("No path found")

if __name__ == '__main__':
    barriers = []
    result = []
    barriers.append([(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)])
    dim = int(input("Dimenstion of the board 10:"))
    robot = int(input("How many robots: ")) 
    H = (input("which type of heuristic, A. Manhattan or B. Chevysyan: Choose A or B ")) 
    if H.lower() == "a":
        H = "manhattan"
    elif H.lower() =="b":
        H = "chebyshev"

    elif H.lower() =="c":
        H = "euclidean"
    for barrier in barriers:			
        plt.plot([v[0] for v in barrier], [v[1] for v in barrier])

    for i in range(robot):
		
        startx = int(input("please indicate x statrting position "))		
        starty = int(input("please indicate y statrting position "))		
        endx = int(input("please indicate x end position "))		
        endy = int(input("please indicate y end position "))		
        result, cost = aStarMeth((startx,starty), (endx,endy), H,dim)		
        print ("route", result)
        print ("cost", cost)

        plt.plot([v[0] for v in result], [v[1] for v in result])
 
        for i in range (1):
            for j in range(0,len(result)):
                barriers[i].append(result[j])
        print(barriers)
	

	
    plt.xlim(1,dim)	
    plt.ylim(1,dim)	
    plt.show()

print(barriers)



