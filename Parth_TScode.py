import numpy as np
import random
from pprint import pprint

flow, distance, tabu = [], [], []
tabuSize = 15 #lower bound 5, upper bound 30
listSize = 20

with open("d:/Tron/4A/ECE457A/Assignments/Flow.csv") as f:
    for line in f.readlines():
        flow. append([int(s) for s in line[:-1].split(',')])

with open("d:/Tron/4A/ECE457A/Assignments/Distance.csv") as f:
    for line in f.readlines():
        distance. append([int(s) for s in line[:-1].split(',')])

def calculateCost(val):
    cost = 0
    for i in range(listSize):
        for j in range(listSize):
            cost += flow[val[i]][val[j]] * distance[i][j]
    return cost

#Create neighbors array 
# 190 (20 choose 2) by 23 (list size plus 3 for storing swapped values and cost at the end)
neighbors = np.zeros((190, 23), dtype=int)

def checkTabu(pair, tabu):
    notTabu = False
    if not pair.tolist() in tabu:
        pair[0], pair[1] = pair[1], pair[0]
        if not pair.tolist() in tabu:
            notTabu = True
    return notTabu

def tabuSearch():
    initialSol = random.sample(range(listSize),listSize)
    optimalSol = initialSol
    iterations = 250

    print("Initial Solution:", initialSol, "Cost =", calculateCost(initialSol))

    while iterations > 0:

        #moving condition using swap
        row = 0
        for i in range(listSize):
            j=i+1
            for j in range(listSize):
                if i < j:
                    initialSol[j], initialSol[i] = initialSol[i], initialSol[j]
                    neighbors[row, :20] = initialSol
                    neighbors[row, 20:] = [initialSol[i], initialSol[j], calculateCost(initialSol)]
                    row += 1
                    initialSol[i], initialSol[j] = initialSol[j], initialSol[i]

        # sort neighborhood based on cost
        neighborsSort = neighbors[neighbors[:,22].argsort()]
        # currNeighbors = neighborsSort

        # Use less than whole neighborhood, take 15 best neighbors 
        currNeighbors = neighborsSort[0:16,:]
        dynamicTabuSize = random.randint(5, 30)
        for i in range(len(currNeighbors)):
            if (checkTabu(currNeighbors[i, 20:22], tabu)):
                initialSol = currNeighbors[i, :20].tolist()
                tabu.append(currNeighbors[i, 20:22].tolist())
                if len(tabu) > dynamicTabuSize-1:
                    tabu.pop(0)
                    #best solution in neighborhood
                if currNeighbors[i,22] < calculateCost(optimalSol):
                    optimalSol = initialSol
                break
            #best solution so far
            elif currNeighbors[i,22] < calculateCost(optimalSol):
                initialSol = currNeighbors[i, :20].tolist()
                optimalSol = initialSol
            

        iterations -=1
        
    #print(neighborsSort[0:5])
    return optimalSol, calculateCost(optimalSol)


if __name__== "__main__":
    # pprint(flow)
    # pprint(distance)
    for tests in range(10):
        optimalpath, optimalcost = tabuSearch()
        print("optimal path is:", optimalpath, "Cost =", optimalcost)
        tests = tests + 1