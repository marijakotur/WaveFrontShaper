'''
Created on 22 aug. 2016

@author: Marija
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#from scipy import interpolate



class geneticAlgorithm(object):
    
    def __init__(self):
        
        a = 1.0
        h = a*np.sqrt(3)/2
        self.electrodeCoordinates = [[-3*a/2,3*h],[-a/2,3*h],[a/2,3*h],[3*a/2,3*h],[-2*a,2*h],[-a,2*h],[0,2*h],[a,2*h],[2*a,2*h],
[-5*a/2,h],[-3*a/2,h],[-a/2,h],[a/2,h],[3*a/2,h],[5*a/2,h],[-3*a,0],[-2*a,0],[-a,0],[0,0],[a,0],[2*a,0],[3*a,0], 
[-5*a/2,-h],[-3*a/2,-h],[-a/2,-h],[a/2,-h],[3*a/2,-h],[5*a/2,-h], 
[-2*a,-2*h],[-a,-2*h],[0,-2*h],[a,-2*h],[2*a,-2*h], 
[-3*a/2,-3*h],[-a/2,-3*h],[a/2,-3*h],[3*a/2,-3*h]]
        self.electrodeCoordinates = np.array(self.electrodeCoordinates)
        
        self.populationSize = 50
        self.numberOfGenes = 37
        self.population = []   
        self.elitismCoef = 0.3   
        self.mutationCoef = 0.1
        self.twoPtXoverCoef = 0.6
        
        self.fitness = [None] * self.populationSize   
        self.geneLowerBound = -2.00
        self.geneUpperBound = 2.00
        self.creepRate = 0.1  
        
        self.fitnessCostWeight = 0.25 / self.populationSize   
        self.referencePopulation = [0] * self.numberOfGenes    

    def calcIdx(self):
        # returns the indices of new generation individuals to be created through elitism, mutation and x-over 
        self.elitismIdx = np.arange(0, int(max(1,np.floor(self.populationSize * self.elitismCoef))))
        self.mutationIdx = np.arange(int(max(self.elitismIdx))+1, int(max(self.elitismIdx)+1  + np.floor(self.populationSize * self.mutationCoef)))
        self.crossOverIdx = np.arange(max(self.mutationIdx)+1, self.populationSize)


    def generateInitialPopulation(self,popSize,numberOfGenes):
        self.population = []
        for i in range(0, self.populationSize):
            self.population.append(np.random.uniform(low=self.geneLowerBound, high=self.geneUpperBound, size=self.numberOfGenes))
        self.population = np.array(self.population)

    
    def sortByFitness(self, pop):        
                
        self.fitnesses = pop*((self.electrodeCoordinates[:,0]-1)**2 - (self.electrodeCoordinates[:,1]-1)**2) 
        
        #fitness with cost functional
        #fitnessCorrection = []
        
        #=======================================================================
        # for i in range(0,len(pop)):
        #     fitnessCorrection.append(np.sum(np.abs(pop[i,:]-self.referencePopulation))**2)
        # fitnessCorrection = np.array(fitnessCorrection)            
        # self.fitnessesWithCost = self.fitnesses  - self.fitnessCostWeight*fitnessCorrection
        #=======================================================================   
                
        #print pop
        #self.fitnesses = pop[:,1]
        self.avgFitness = np.average(self.fitnesses)
        self.maxFitness = np.max(self.fitnesses) 
        j = np.argsort(self.fitnesses)
        j = np.flipud(j)
        pop = pop[j]
        return pop  

    def creepMutation(self,individualToMutate):
        #change one gene according to
        #g_new = min( imax, max( imin, g_old + r*s*(imax-imin)
        # r - random number in the [-1,1] range
        # s - creep rate, usually 0.02
        # imin, imax - gene range bounds
        mutationLocation = np.random.randint(0,self.numberOfGenes)
        #print "mutation location is %d" %mutationLocation
        #print individualToMutate
        oldGene = individualToMutate[mutationLocation]
        r = np.random.uniform(low=-1.0, high=1.0, size=1)
        newGene = min( self.geneUpperBound, max( self.geneLowerBound, oldGene + r*self.creepRate*(self.geneUpperBound-self.geneLowerBound)))
        individualToMutate[mutationLocation] = newGene
        return individualToMutate    

    def onePtXover(self,ind1,ind2):
        cxpoint = np.random.randint(1,self.numberOfGenes) 
        #print cxpoint
        ind1[cxpoint:], ind2[cxpoint:] = ind2[cxpoint:], ind1[cxpoint:]    
        return np.array([ind1, ind2])
    
    def newGeneration(self):
        sortedPopulation = np.copy(self.sortByFitness(self.population))   
        # print sortedPopulation     
        newPopulation = []
        for i in range(0,self.populationSize):
            if i in self.elitismIdx:
                newPopulation.append(sortedPopulation[i])
            elif i in self.mutationIdx:
                #select individual j (from the top half of the population) to mutate 
                j = np.random.randint(0,np.floor(self.populationSize/2))
                #print "individual to mutate is %d" %j
                indToMutate = np.copy(sortedPopulation[j])
                print indToMutate
                newPopulation.append(self.creepMutation(indToMutate))
            elif i in self.crossOverIdx:
                #loop through all the xOver indices
                #select two individuals to mate
                j1 = np.random.randint(0,np.floor(self.populationSize/2))
                j2 = np.random.randint(0,np.floor(self.populationSize/2))
                # make sure they are different
                while j2 == j1:
                    j2 = np.random.randint(0,np.floor(self.numberOfGenes/2))
                    #print j1, j2
                    # taking only one child from x-over
                #newPopulation.append(self.onePtXover(self.population[j1], self.population[j2]))
                newPopulation.append(np.array(self.onePtXover(self.population[j1], self.population[j2])[1]))
                #self.population = np.array(self.population)
                
        self.population = np.array(newPopulation)
        #print type(self.population)
        #print self.population.shape
        #self.population = np.array(self.population)
        
#===============================================================================
# #generate electrodes
# a = 1.0
# h = a*np.sqrt(3)/2
# coordinates = [(-3*a/2,3*h),(-a/2,3*h),(a/2,3*h),(3*a/2,3*h),(-2*a,2*h),(-a,2*h),(0,2*h),(a,2*h),(2*a,2*h), 
# (-5*a/2,h),(-3*a/2,h),(-a/2,h),(a/2,h),(3*a/2,h),(5*a/2,h),(-3*a,0),(-2*a,0),(-a,0),(0,0),(a,0),(2*a,0),(3*a,0), 
# (-5*a/2,-h),(-3*a/2,-h),(-a/2,-h),(a/2,-h),(3*a/2,-h),(5*a/2,-h), 
# (-2*a,-2*h),(-a,-2*h),(0,-2*h),(a,-2*h),(2*a,-2*h), 
# (-3*a/2,-3*h),(-a/2,-3*h),(a/2,-3*h),(3*a/2,-3*h)]
#===============================================================================

if __name__ == '__main__':
    ga = geneticAlgorithm()
    ga.calcIdx()
    ga.generateInitialPopulation(ga.populationSize, ga.numberOfGenes)
        
    
    f = []
    gen = []
    for i in range(1,50):
        print i
        ga.newGeneration()
        f.append(ga.fitnesses)
        gen.append(i)




                           
#===============================================================================
# electrode = [[xs, ys, shape(xs,ys)] for xs,ys in coordinates]
# electrode = np.array(electrode)
# 
# # interpolation function
# #f = interpolate.interp2d(xs, ys, zs, kind='cubic',copy=True, bounds_error=False)
# 
# 
# #generate membrane
# #xm = np.linspace(-2.8*a, 2.8*a, 30)
# #ym = np.linspace(-2.8*h, 2.8*h, 30)
# xm = np.arange(-3*a, 3*a,  1.5e-1)
# ym = np.arange(-3*h, 3*h,  1.5e-1)
# XM, YM = np.meshgrid(xm, ym)
# 
# ZM = [[0]*len(xm)]*len(ym)
# for point in XM, YM:
#     for t in electrode:
#         ZM += t[2]*np.exp(-(XM-t[0])**2-(YM-t[1])**2)
# 
# 
# #fig electrodes
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(electrode[:,0], electrode[:,1], electrode[:,2], zdir='z', s=20, c='b', depthshade=True) 
# 
# ax.scatter(XM, YM, ZM, zdir='z', s=20, c='r', depthshade=True) 
# 
#   
#   
# #fig membrane
# #ax.plot_surface(XM, YM, ZM)
#   
#       
# plt.show()  
#===============================================================================
