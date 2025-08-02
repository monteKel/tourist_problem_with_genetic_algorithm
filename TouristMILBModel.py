import numpy as np
import pulp
import ast
import time

t0 = time.time()

#     Graph Creation     #######################################################
def createPlacesList(filename="vertex.txt"):
    vertexList = []
    malformedLines = 1

    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            parts = line.split('-')
            if len(parts) != 4:
                print(f"Skkiping a Malformed Line, the actual number of Malformed lines is: {malformedLines}\n")
                malformedLines+=1
                continue
            vertexIndex = int(parts[0])
            vertexName = parts[1]
            vertexName.strip('"')
            vertexTime= int(parts[2])
            RawVertexCharacteristicsList = parts[3]
            if RawVertexCharacteristicsList == '[None]':
                vertexCharacteristicsList = None
            else:
                vertexCharacteristicsList = ast.literal_eval(RawVertexCharacteristicsList)
            vertexList.append([vertexIndex,vertexName,vertexTime,vertexCharacteristicsList])
    return vertexList

def createGraph(filename="edges.txt"):
    malformedLines = 1
    biggestNumber = 0

    with open(filename, 'r') as file:
        for line in file:
            parameters = line.split('-')
            vertexNumber1 = int(parameters[0])
            vertexNumber2 = int(parameters[1])

            if vertexNumber1 > vertexNumber2 and vertexNumber1 > biggestNumber:
                biggestNumber = vertexNumber1
            elif vertexNumber2 > biggestNumber:
                biggestNumber = vertexNumber2
            
    
    graph = np.zeros((biggestNumber+1,biggestNumber+1),dtype=int)

    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            parts=line.split('-')
            if len(parts) != 3:
                print(f"Skkiping a Malformed Line, the actual number of Malformed lines is: {malformedLines}\n")
                malformedLines+=1
                continue
            firstIndex = int(parts[0])
            secondIndex = int(parts[1])
            weight = int(parts[2])
            graph[firstIndex][secondIndex] = weight
            graph[secondIndex][firstIndex] = weight
    EdgesList = []
    EdgesListWeight = {}
    for VertexLine in range(biggestNumber+1):
        for VertexColumn in range(biggestNumber+1):
            if graph[VertexLine][VertexColumn] > 0:
                EdgesList.append((VertexLine,VertexColumn))
                EdgesListWeight[(VertexLine,VertexColumn)] = int(graph[VertexLine][VertexColumn])


    return EdgesList,EdgesListWeight
################################################################################

#     Entries     ##############################################################
Problem = pulp.LpProblem("Tourist_Problem", pulp.LpMaximize) #Creating the Problem
placesList = createPlacesList() # A List with all places with the time that activity wates
placesIndexesList = []
startPoint = 0 # (Vertex) The Starting point of the tour
for index in range(len(placesList)):
    placesIndexesList.append(placesList[index][0])
graphEdges,graphEdgesWeight = createGraph() # The Graph with the connections and weight of each Connection


















TouristCharacteristics = [6,9,6,9,1,2,3,4,5] # A List of tourist preferences based on his/her profile
























#                         Nature  Noise  Music  Culture  Movement  Money  Historical  SocialInteraction  Fun
weightV = [0] #Weight of the vertex with just the Start Point
for index in range(1,len(placesList)):
    PlaceCharacteristics = placesList[index][3]
    weight=0
    for preference in range(len(PlaceCharacteristics)):
        weight += 10 - abs(PlaceCharacteristics[preference] - TouristCharacteristics[preference])
    weightV.append(weight) # A For lasso that append every weight of every place
timeV = [0] #Time of the vertex with just the Start Point
for index in range(1,len(placesList)):
    PlaceTime = placesList[index][2]
    timeV.append(PlaceTime) # A For lasso that append every weight of every place



################################################################################


#     Callout     ##############################################################
Tmax = 600 # Maximum time (in minutes) that the tourist can go on a tour
wishedNumberTourPlaces = 5 # Number of places that the tourist wanna go
X = pulp.LpVariable.dicts("X", [(i, j) for (i, j) in graphEdges], 0, 1, pulp.LpBinary) #Variable
Y = pulp.LpVariable.dicts("Y", placesIndexesList, 0, 1, pulp.LpBinary) #Variable
U = pulp.LpVariable.dicts("U", placesIndexesList, 1, len(placesIndexesList), pulp.LpInteger) #Variable
Problem += U[0] == 1, "Start at Start" #Start Point is ALWAYS the first
Problem += pulp.lpSum(weightV[k] * Y[k] for k in placesIndexesList), "Max Satisfaction" #Objective Function
Problem += pulp.lpSum(X[(startPoint, j)] for j in placesIndexesList if (startPoint, j) in graphEdges) == 1, "StartPoint"
Problem += pulp.lpSum(X[(i, startPoint)] for i in placesIndexesList if (i, startPoint) in graphEdges) == 1, "EndPoint"
Problem += Y[startPoint] == 1, "Start at Start lol"

for k in placesIndexesList:
    if k != startPoint:
        Problem += pulp.lpSum(X[(i, k)] for i in placesIndexesList if (i, k) in graphEdges) == Y[k], f"Enter flow {k}"

        Problem += pulp.lpSum(X[(k, j)] for j in placesIndexesList if (k, j) in graphEdges) == Y[k], f"Exit Flow {k}"

for i, j in graphEdges:
    Problem += X[(i, j)] <= Y[i], f"X {i},{j} -> Y{i}"
    Problem += X[(i, j)] <= Y[j], f"X {i},{j} -> Y{j}"

Problem += pulp.lpSum(Y[k] for k in placesIndexesList if k != startPoint) == wishedNumberTourPlaces, "Total of visited Places"

TotalActivity = pulp.lpSum(timeV[k] * Y[k] for k in placesIndexesList)
TotalDisplaceTime = pulp.lpSum(graphEdgesWeight[(i, j)] * X[(i, j)] for (i, j) in graphEdges)
Problem += TotalActivity + TotalDisplaceTime <= Tmax, "Total of maximum time"

M = wishedNumberTourPlaces + 1
for i in placesIndexesList:
    for j in placesIndexesList:
        if i != j and (i, j) in graphEdges and i != startPoint and j != startPoint:
            Problem += U[i] - U[j] + M * X[(i, j)] + (M - 1) * (1 - Y[j]) <= M - 1, f"Sub Route {i},{j}"

Problem.writeLP("Tourist_Problem.Lp")

#Problem.solve(pulp.PULP_CBC_CMD(timeLimit=300))
#Problem.solve()

print("\n" * 100)
print(f"Tempo máximo (Tmax): {Tmax} minutos")
print(f"Locais desejados: {wishedNumberTourPlaces}")
print(f"Tempo total mínimo estimado: {sum(sorted(timeV)[1:wishedNumberTourPlaces+1])} minutos (sem deslocamento)")
print("\n------------ Results ------------")
print(f"Status: {pulp.LpStatus[Problem.status]}")

if Problem.status == pulp.LpStatusOptimal:
    print(f"Optimal Value (Satisfaction Level): {pulp.value(Problem.objective)}")

    print("\nVisited Places:\n")
    visitedPlaces = []
    for k in placesIndexesList:
        if Y[k].varValue == 1:
            place = placesList[k][1]
            visitedPlaces.append(k)
            print(f"  {place}")

    print("\n")

    next_place = {}
    for (i, j) in graphEdges:
        if X[(i, j)].varValue == 1:
            next_place[i] = j

    print("Path Made:")
    print("\n")
    visited_set = set()
    ordered_path = []
    current = 0
    while current in next_place and current not in visited_set:
        visited_set.add(current)
        next_node = next_place[current]
        ordered_path.append((current, next_node))
        current = next_node

    for i, j in ordered_path:
        originalPlace = placesList[i][1]
        finalPlace = placesList[j][1]
        print(f"  {originalPlace} -> {finalPlace}")
    print("\n")

    print("\nIdeal Order To Visit:")

    for k in sorted([v for v in placesIndexesList if Y[v].varValue == 1], key=lambda node: U[node].varValue):
        place = placesList[k][1]
        print(f"  {place}: is the {int(U[k].varValue)}* to visit")

    TotalTime = sum(timeV[k] * Y[k].varValue for k in placesIndexesList) + \
                       sum(graphEdgesWeight[(i, j)] * X[(i, j)].varValue for (i, j) in graphEdges)
    print(f"\nTotal time of tour: {TotalTime} minutes (Tmax: {Tmax})")
    t1 = time.time()
    diff = t1 - t0
    print(f"Time of Operation: {diff:.2f}s")
else:
    print("Optimal soluction not found :(")

################################################################################
