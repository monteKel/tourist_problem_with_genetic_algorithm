import TouristMILBModel
import random
import time

t0 = time.time()
def initPopulation(AllPlacesIndexes,wishedNumberTourPlaces,PopulationSize=50):
    population = []
    placesIndexes = AllPlacesIndexes
    for repeat in range(PopulationSize):
        person =[0]
        random.shuffle(placesIndexes)
        for index in range(wishedNumberTourPlaces):
            person.append(placesIndexes[index])
        person.append(0)
        population.append(person)
    return population

def FitnessFunction(person,timeV,weightV,graphEdgesWeight,Tmax):
    TotalSatisfaction = 0
    TotalActivityTime = 0
    TotalTravelTime = 0
    middlePlaces = person[1:-1]
    if len(set(middlePlaces)) != wishedNumberTourPlaces:
        return -float('inf')
    
    for i in range(len(person)-1):
        placeIndex = person[i]
        if placeIndex != 0:
            TotalSatisfaction += weightV[placeIndex]
            TotalActivityTime += timeV[placeIndex]
    penalty = 500
    numMissingEdges = 0
    for i in range(len(person)-1):
        currentPlace = person[i]
        nextPlace = person[i+1]
        if (currentPlace,nextPlace) in graphEdgesWeight:
            TotalTravelTime += graphEdgesWeight[(currentPlace,nextPlace)]
        elif (nextPlace,currentPlace) in graphEdgesWeight:
            TotalTravelTime += graphEdgesWeight[(nextPlace,currentPlace)]
        else:
            numMissingEdges += 1
            TotalTravelTime += 100
        
    TotalSatisfaction -= (numMissingEdges * penalty)
    totalTime = TotalActivityTime + TotalTravelTime
    if totalTime > Tmax:
        exceedPenalty = (totalTime - Tmax) * 10
        TotalSatisfaction -= exceedPenalty

    if TotalSatisfaction < 0 and numMissingEdges > 0:
            return -1
    elif TotalSatisfaction < 0 and totalTime > Tmax:
            return -1
    elif TotalSatisfaction < 0:
            return 0

    return TotalSatisfaction 

def TournamentSelection(population,populationSatisfaction,tournamentSize):
    if len(population) == 0:
        return None
    tournamentSize = int(tournamentSize) 
    if tournamentSize < 1:
            tournamentSize = 1
    if tournamentSize > len(population):
        tournamentSize = len(population)
    tournamentIndexes = random.sample(range(len(population)),tournamentSize)
    bestPerson = population[tournamentIndexes[0]] 
    bestFitness = populationSatisfaction[tournamentIndexes[0]]
    for i in range(1,len(tournamentIndexes)):
        index = tournamentIndexes[i]
        currentPerson =  population[index]
        currentFitness = populationSatisfaction[index]

        if currentFitness > bestFitness:
            bestFitness = currentFitness
            bestPerson = currentPerson

    return bestPerson

def swapMutation(person,wishedNumberTourPlaces):
    mutatedPerson = list(person)
    if wishedNumberTourPlaces < 2:
        return mutatedPerson
    
    index1,index2 = random.sample(range(1,wishedNumberTourPlaces+1),2)
    mutatedPerson[index1],mutatedPerson[index2] = mutatedPerson[index2],mutatedPerson[index1]

    return mutatedPerson

def geneticAlgoritm(
        populationSize,
        wishedNumberTourPlaces,
        maxGenerations,
        crossoverRate,
        mutationRate,
        tournamentSize,
        allPlacesIndexes,
        timeV,
        weightV,
        graphEdgesWeight,
        Tmax):
    population=initPopulation(TouristMILBModel.placesIndexesList,TouristMILBModel.wishedNumberTourPlaces,populationSize)
    bestOverallSoluction = None
    bestOverallFitness = -float('inf')

    for generation in range(maxGenerations):
        print(f"\n--- Generation {generation + 1}/{maxGenerations} ---")
        fitnessScore = []
        for person in population:
            fitness = FitnessFunction(person, timeV, weightV, graphEdgesWeight, Tmax)
            fitnessScore.append(fitness)

        currentBestFitnessByGen = -float('inf')
        currentBestSolutionByGen = None

        for i, fitness in enumerate(fitnessScore):
            if fitness > currentBestFitnessByGen:
                currentBestFitnessByGen = fitness
                currentBestSolutionByGen = population[i]
            
            if fitness > bestOverallFitness:
                bestOverallFitness = fitness
                bestOverallSoluction = population[i]

        print(f"Best Fitness of Generation: {currentBestFitnessByGen}")
        print(f"Best Soluction of Generation: {currentBestSolutionByGen}")

        if generation == maxGenerations - 1:
            break

        newPopulation = []

        for i in range(populationSize // 2):
            parent1 = TournamentSelection(population,fitnessScore,tournamentSize)
            parent2 = TournamentSelection(population,fitnessScore,tournamentSize)

            child1, child2 = parent1, parent2
            if random.random() < crossoverRate:
                child1, child2 = crossover(parent1, parent2, wishedNumberTourPlaces)

            if random.random() < mutationRate:
                child1 = swapMutation(child1, wishedNumberTourPlaces)
            if random.random() < mutationRate:
                child2 = swapMutation(child2, wishedNumberTourPlaces)
            newPopulation.append(child1)

            if len(newPopulation) < populationSize:
                newPopulation.append(child2)

        population = newPopulation

    return bestOverallSoluction, bestOverallFitness
            
def crossover(person1, person2, wishedNumberTourPlaces):
    parent1Middle = person1[1:-1]
    parent2Middle = person2[1:-1]
    
    length = wishedNumberTourPlaces 
    
    if length < 2:
        return person1, person2

    startCut = random.randint(0, length - 1)
    endCut = random.randint(startCut, length - 1)


    child1Middle = [None] * length
    child1Middle[startCut : endCut + 1] = child1Middle[startCut : endCut + 1]
    

    p2Index = (endCut + 1) % length
    c1Index = (endCut + 1) % length

    while None in child1Middle:
        if parent2Middle[p2Index] not in child1Middle:
            child1Middle[c1Index] = parent2Middle[p2Index]
            c1Index = (c1Index + 1) % length
        
        p2Index = (p2Index + 1) % length

    child2Middle = [None] * length
    child2Middle[startCut : endCut + 1] = parent2Middle[startCut : endCut + 1]

    p1Index = (endCut + 1) % length
    c2Index = (endCut + 1) % length

    while None in child2Middle:
        if parent1Middle[p1Index] not in child2Middle:
            child2Middle[c2Index] = parent1Middle[p1Index]
            c2Index = (c2Index + 1) % length
        
        p1Index = (p1Index + 1) % length

    child1 = [0] + child1Middle + [0]
    child2 = [0] + child2Middle + [0]

    return child1, child2    
################################################################################

populationSize = 200
maxGenerations = 500
tournamentSize = 10
crossOverRate = 0.9
mutationRate = 0.05
placesIndexesList = TouristMILBModel.placesIndexesList
timeV = TouristMILBModel.timeV
weightV = TouristMILBModel.weightV
graphEdgesWeight = TouristMILBModel.graphEdgesWeight
Tmax = TouristMILBModel.Tmax
wishedNumberTourPlaces = TouristMILBModel.wishedNumberTourPlaces

finalBestSoluction, finalBestFitness = geneticAlgoritm(
    populationSize,
    wishedNumberTourPlaces,
    maxGenerations,
    tournamentSize,
    crossOverRate,
    mutationRate,
    placesIndexesList,
    timeV,
    weightV,
    graphEdgesWeight,
    Tmax
)

t1 = time.time()

diff = t1 - t0
print("\n\n------------ Final Results of the Algoritm ------------")
print(f"Best Soluction: {finalBestSoluction}")
print(f"Fitness of best Soluction: {finalBestFitness}")
print(f"Time of Operation: {diff:.2f}s")


print("\nPlaces order in the soluction found:")
if finalBestSoluction:
    for index in finalBestSoluction:
        print(f"  {TouristMILBModel.placesList[index][1]}")

