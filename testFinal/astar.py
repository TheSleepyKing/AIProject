from __future__ import print_function
from ctypes.wintypes import HACCEL
from logging import Handler
import math
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import *
import io
from PIL import Image, ImageTk 

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

barriers = []
def createAgents():
    global barriers
    result = []
    barriers.append([(2,4),(2,5),(2,6),(3,6),(4,6),(5,6),(5,5),(5,4),(5,3),(5,2),(4,2),(3,2)])
    dim = dimensionMenuVariables.get()
    dim_split = dim.split("x")
    dim = int(dim_split[0])
    #dim = 10
    #robot = int(input("How many robots: ")) 
    #H = (input("which type of heuristic, A. Manhattan or B. Chevysyan: Choose A or B ")) 
    #if H.lower() == "a":
    H = heuristicVariables.get().lower()
    #elif H.lower() =="b":
    #    H = "chebyshev"

    #elif H.lower() =="c":
    #    H = "euclidean"
    for barrier in barriers:			
        plt.plot([v[0] for v in barrier], [v[1] for v in barrier])

    for i in range(int(num_agents_input.get())):
		
        #startx = int(input("please indicate x statrting position "))		
        #starty = int(input("please indicate y statrting position "))		
        #endx = int(input("please indicate x end position "))		
        #endy = int(input("please indicate y end position "))
        
        startx  = int(ref[(0 + 4 * i)].get()) 
        starty  = int(ref[(1 + 4 * i)].get()) 
        endx    = int(ref[(2 + 4 * i)].get()) 
        endy    = int(ref[(3 + 4 * i)].get()) 
        	
        result, cost = aStarMeth((startx,starty), (endx,endy), H,dim)		
        print ("route", result)
        print ("cost", cost)

        plt.plot([v[0] for v in result], [v[1] for v in result])
 
        for i in range (1):
            for j in range(0,len(result)):
                barriers[i].append(result[j])
        print(barriers)
	
    img_buf = io.BytesIO()
    plt.savefig(img_buf, format='png')
    im = Image.open(img_buf)
    tk_image = ImageTk.PhotoImage(im)
    chart= Label(canvas, image = tk_image)
    chart.image = tk_image
    chart.place(x = 860, y = 200, width = 600, height = 480)
    canvas.pack()
	
    #plt.xlim(1,dim)	
    #plt.ylim(1,dim)	
    #plt.show()

#  print(barriers)


ref = []
def GenerateAgentInputs(input_counts):
    global ref
    for i in range(len(ref)):
        ref[i].grid_remove()
        
    
    
    ref = []
    for i in range(input_counts):
        for j in range (4):
            l=tk.Label(agentCanvas,text='Agent '+str(i+1),font=10)
            l.grid(row=i,column=0,padx=3)

            entry = tk.Entry(agentCanvas, font=10, width = 10)
            entry.grid(row=i, column=(j+1),padx=3,pady=3) 
            
            
            ref.append(entry) # store references 

############################# tkinter stuff #############################
root = tk.Tk()
root.title('AI traversal')
root.geometry("1480x900")
canvas = Canvas(
        root,
        bg = "#FFFFFF",
        height = 960,
        width = 1600,
        bd = 0,
        highlightthickness = 0,
        relief = "ridge"
    )

agentCanvas = Canvas(
    canvas,
    bg = "lightgrey",
    height = 800,
    width = 420,
)

agentCanvas.place(x = 200 , y = 25)



OPTIONS = [
    "1x1","2x2","3x3","4x4","5x5",
    "6x6","7x7","8x8","9x9","10x10"
    ] 
dimensionMenuVariables = StringVar(canvas)
dimensionMenuVariables.set(OPTIONS[0]) # default value
dimensionMenu = OptionMenu(canvas, dimensionMenuVariables, *OPTIONS)

heuristicOPTIONS = [
    "Manhattan","Chevysyan","Euclidean"
    ] 
heuristicVariables = StringVar(canvas)
heuristicVariables.set(heuristicOPTIONS[0]) # default value
heuristicMenu = OptionMenu(canvas, heuristicVariables, *heuristicOPTIONS)


def callback(sv):
    if (sv.get() != ""):
        GenerateAgentInputs(int(sv.get()))
    return True 

sv = StringVar(value = "1")
sv.trace("w", lambda name, index, mode, sv=sv: callback(sv))
num_agents_input   = tk.Entry(master = canvas, width = 100 , bd = 1, textvariable=sv) 
sv.set(value ="1")

def createFrontend():
    
    global num_agents_input

    canvas.place(x = 0, y = 0)

    #Dimension Dropdown Menu
    dimension_label = tk.Label(master = canvas, text = "Board Dimensions")
    dimension_label.place(x=10,y=25,width=100.0,height=10.0)
    dimensionMenu.place( x=10.0,y=30,width=100.0,height=50.0)

    #Create Agents Button
    button_1 = tk.Button(master = canvas, text = "Create Agents", command=lambda: createAgents())
    button_1.place( x=10.0,y=800,width=100.0,height=50.0)

    #Number of agents input
    num_agents_label = tk.Label(master = canvas, text = "Number of agents")
    num_agents_label.place(x=10,y=85,width=100.0,height=15.0)
    num_agents_input.place(x=10.0,y=100,width=100.0,height=25.0)

    #Heuristic Dropdown Menu
    heuristic_label = tk.Label(master = canvas, text = "Heuristic Options")
    heuristic_label.place(x=10,y=135,width=100.0,height=15.0)
    heuristicMenu.place( x=10.0,y=150,width=100.0,height=50.0)

    #Start and end x/y 
    num_agents_label = tk.Label(master = canvas, text = "Start X")
    num_agents_label.place(x=300,y=10,width=50.0,height=15.0)
    num_agents_label = tk.Label(master = canvas, text = "Start Y")
    num_agents_label.place(x=400,y=10,width=50.0,height=15.0)
    num_agents_label = tk.Label(master = canvas, text = "End X")
    num_agents_label.place(x=500,y=10,width=50.0,height=15.0)
    num_agents_label = tk.Label(master = canvas, text = "End Y")
    num_agents_label.place(x=600,y=10,width=50.0,height=15.0)
    
    


    root.resizable(False, False)  
    root.mainloop()


if __name__ == '__main__':
    createFrontend()




