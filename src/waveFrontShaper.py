'''
Created on 24 aug. 2016

@author: Marija

A simple genetic algorithm for use with a given electrode configuration for shaping the laser beam wavefront using a deformable mirror.
'''
from PyQt4 import QtGui, QtCore
import sys
import numpy as np
import PyTango as pt
#import threading
import time
#from blaze.expr.expressions import symbol
import pyqtgraph as pq


# noinspection PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit,PyAttributeOutsideInit
class myWidgetTest(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        #self.settings = QtCore.QSettings('MaxIV', 'GA')

        self.cameraName = pt.DeviceProxy('gunlaser/thg/camera')
        self.cameraDevice = pt.DeviceProxy(self.cameraName)
        self.image = self.cameraName.read_attribute('Image').value

        self.initGA()
        self.myLayout()

        self.xData = np.array(range(0, 10))
        self.yData = np.array(range(0, 10)) #np.random.rand(1,10)

        #self.measureData()

        self.scanTimer = QtCore.QTimer()
        self.scanTimer.timeout.connect(self.scanUpdateAction)


    def initGA(self):
        a = 1.0
        h = a*np.sqrt(3)/2
        self.electrodeCoordinates = [[-3*a/2,3*h],[-a/2,3*h],[a/2,3*h],[3*a/2,3*h],[-2*a,2*h],[-a,2*h],[0,2*h],[a,2*h],[2*a,2*h],
[-5*a/2,h],[-3*a/2,h],[-a/2,h],[a/2,h],[3*a/2,h],[5*a/2,h],[-3*a,0],[-2*a,0],[-a,0],[0,0],[a,0],[2*a,0],[3*a,0],
[-5*a/2,-h],[-3*a/2,-h],[-a/2,-h],[a/2,-h],[3*a/2,-h],[5*a/2,-h],
[-2*a,-2*h],[-a,-2*h],[0,-2*h],[a,-2*h],[2*a,-2*h],
[-3*a/2,-3*h],[-a/2,-3*h],[a/2,-3*h],[3*a/2,-3*h]]
        self.electrodeCoordinates = np.array(self.electrodeCoordinates)

        self.populationSize = 20
        self.numberOfGenes = 37
        self.population = []

        self.elitismCoef = 0.3
        self.mutationCoef = 0.2
        self.twoPtXoverCoef = 0.5
        self.calcIdx()

        self.fitness = [None] * self.populationSize
        self.geneLowerBound = -20.00
        self.geneUpperBound = 20.00
        self.creepRate = 0.1

        self.fitnessCostWeight = 0.25 / self.populationSize
        self.referencePopulation = [0]*self.numberOfGenes


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
                #print indToMutate
                newPopulation.append(self.creepMutation(indToMutate))
            elif i in self.crossOverIdx:
                #loop through all the xOver indices
                #select two individuals to mate
                j1 = np.random.randint(0,np.floor(self.populationSize/2))
                j2 = np.random.randint(0,np.floor(self.populationSize/2))
                # make sure they are different
                while j2 == j1:
                    j2 = np.random.randint(0,np.floor(self.populationSize/2))
                    #print j1, j2
                    # taking only one child from x-over
                #newPopulation.append(self.onePtXover(self.population[j1], self.population[j2]))
                newPopulation.append(np.array(self.onePtXover(self.population[j1], self.population[j2])[1]))
                #self.population = np.array(self.population)

        self.population = np.array(newPopulation)
        #print type(self.population)
        #print self.population.shape
        #self.population = np.array(self.population)

    def onePtXover(self,ind1,ind2):
        cxpoint = np.random.randint(1,self.numberOfGenes)
        #print cxpoint
        ind1[cxpoint:], ind2[cxpoint:] = ind2[cxpoint:], ind1[cxpoint:]
        return np.array([ind1, ind2])

    def generateInitialPopulation(self,popSize,numberOfGenes):
        self.population = []
        for i in range(0, self.populationSize):
            self.population.append(np.random.uniform(low=self.geneLowerBound, high=self.geneUpperBound, size=self.numberOfGenes))
        self.population = np.array(self.population)
        #print self.population.shape


    def sortByFitness(self, pop):
        self.fitness = []
        for ind in pop:
            s = 0
            for i in range(0,self.numberOfGenes):
                s += ind[i] * ((self.electrodeCoordinates[i,0])**2 + (self.electrodeCoordinates[i,1])**2)
            self.fitness.append(s)

        #fitness with cost functional
        #fitnessCorrection = []

        #for i in range(0,len(pop)):
        #    fitnessCorrection.append( np.sum((pop[i,:]-self.referencePopulation)**2) )
        #fitnessCorrection = np.array(fitnessCorrection)

        #self.fitnessesWithCost = self.fitnesses  - self.fitnessCostWeight*fitnessCorrection
        #- self.fitnessCostWeight * fitnessCorrection

        #self.avgFitness = np.average(self.fitness)
        #self.maxFitness = np.max(self.fitness)
        j = np.argsort(self.fitness)
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

    def myLayout(self):
        self.layout = QtGui.QVBoxLayout(self)
        self.inputLayout = QtGui.QHBoxLayout()
        self.plotLayout = QtGui.QHBoxLayout()
        self.camLayout = QtGui.QHBoxLayout()
        self.graphLayout = QtGui.QHBoxLayout()
        self.gridLayout1 = QtGui.QGridLayout()
        self.gridLayout2 = QtGui.QGridLayout()
        self.gridLayout3 = QtGui.QGridLayout()
        self.gridLayout4 = QtGui.QGridLayout()


        #QVBoxLayout1 = QtGui.QHBoxLayout()
        #self.layout1 = self.layout.addLayout(QVBoxLayout1)

        self.gridLayout1.addWidget(QtGui.QLabel("Population size"), 0, 0)
        self.popSizeSpinbox = QtGui.QSpinBox()
        self.popSizeSpinbox.setValue(self.populationSize)
        self.popSizeSpinbox.valueChanged.connect(self.setPopSize)
        self.gridLayout1.addWidget(self.popSizeSpinbox,0,1)

        self.gridLayout1.addWidget(QtGui.QLabel("Number of genes"), 1,0)
        self.numberOfGenesSpinbox = QtGui.QSpinBox()
        self.numberOfGenesSpinbox.setValue(37)
        self.gridLayout1.addWidget(self.numberOfGenesSpinbox, 1,1)
        self.gridLayout1.addItem(QtGui.QSpacerItem(10, 15,QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding), 2,0)

        self.gridLayout2.addWidget(QtGui.QLabel("Elitism coef."), 0,0)
        self.elitismCoefSpinbox = QtGui.QDoubleSpinBox()
        self.elitismCoefSpinbox.setValue(0.2)
        self.elitismCoefSpinbox.setDecimals(2)
        self.elitismCoefSpinbox.setMaximum(1)
        self.elitismCoefSpinbox.setMinimum(0)
        self.elitismCoefSpinbox.valueChanged.connect(self.setElitismCoef)
        self.gridLayout2.addWidget(self.elitismCoefSpinbox, 0,1)

        self.gridLayout2.addWidget(QtGui.QLabel("Mutation coef."), 1,0)
        self.mutationCoefSpinbox = QtGui.QDoubleSpinBox()
        self.mutationCoefSpinbox.setValue(0.2)
        self.mutationCoefSpinbox.setDecimals(2)
        self.mutationCoefSpinbox.setMaximum(1)
        self.mutationCoefSpinbox.setMinimum(0)
        self.mutationCoefSpinbox.valueChanged.connect(self.setMutationCoef)
        self.gridLayout2.addWidget(self.mutationCoefSpinbox, 1,1)

        self.gridLayout2.addWidget(QtGui.QLabel("Crossover coef."), 2,0)
        self.crossOverCoefSpinbox = QtGui.QDoubleSpinBox()
        self.crossOverCoefSpinbox.setValue(0.6)
        self.crossOverCoefSpinbox.setDecimals(2)
        self.crossOverCoefSpinbox.setMaximum(1)
        self.crossOverCoefSpinbox.setMinimum(0)
        self.crossOverCoefSpinbox.valueChanged.connect(self.setCrossOverCoef)
        self.gridLayout2.addWidget(self.crossOverCoefSpinbox, 2,1)

        self.gridLayout2.addItem(QtGui.QSpacerItem(10, 15,QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding), 3,0)

        self.startButton = QtGui.QPushButton('Start')
        self.startButton.clicked.connect(self.startScan)
        self.stopButton = QtGui.QPushButton('Stop')
        self.stopButton.clicked.connect(self.stopScan)
        self.gridLayout3.addWidget(QtGui.QLabel("Start scan"), 0, 0)
        self.gridLayout3.addWidget(self.startButton, 0, 1)
        self.gridLayout3.addWidget(QtGui.QLabel("Stop scan"), 1, 0)
        self.gridLayout3.addWidget(self.stopButton, 1, 1)
        self.gridLayout3.addItem(QtGui.QSpacerItem(10, 15,QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding), 3,0)

        self.currentGenLabel = QtGui.QLabel()

        self.plotWidget2 = pq.PlotWidget(useOpenGL=True)
        self.plotWidget2.setSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Maximum)
        self.plot21 = self.plotWidget2.plot()
        self.plot22 = self.plotWidget2.plot()

        self.plotWidget1 = pq.PlotWidget()
        #self.plot11 = self.plotWidget1.plot()
        #self.plotWidget1.setSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Maximum)
        self.plot11 = pq.ScatterPlotItem(size=50, pen=pq.mkPen('w'), pxMode=True, symbol='o')
        self.plotWidget1.addItem(self.plot11)
        self.plotWidget1.setAspectLocked(1)

        self.cameraWindow = pq.GraphicsLayoutWidget()
        #self.cameraWindow.setSizePolicy(QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Expanding)
        self.camLayout.addWidget(self.cameraWindow)
        view = self.cameraWindow.addViewBox()
        #self.cameraWindow.
        view.setAspectLocked(True)
        img = pq.ImageItem(border='w')
        view.addItem(img)
        img.setImage(self.image)

        self.plotLayout = QtGui.QHBoxLayout()
        self.plotLayout.addWidget(self.plotWidget2)
        self.plotWidget1.setVisible(True)
        self.plotWidget1.setMaximumHeight(400)
        self.plotLayout.addWidget(self.plotWidget1)
        self.plotWidget2.setVisible(True)
        self.plotWidget2.setMaximumHeight(400)


        self.graphLayout.addLayout(self.plotLayout)
        self.graphLayout.addLayout(self.camLayout)
        self.layout.addLayout(self.inputLayout)
        self.layout.addLayout(self.graphLayout)
        self.inputLayout.addLayout(self.gridLayout1)
        self.inputLayout.addLayout(self.gridLayout2)
        self.inputLayout.addLayout(self.gridLayout3)
        #self.inputLayout.addSpacerItem(QtGui.QSpacerItem(10, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum))

        self.startButton = QtGui.QPushButton('Start')
        self.startButton.clicked.connect(self.startScan)
        self.stopButton = QtGui.QPushButton('Stop')
        self.stopButton.clicked.connect(self.stopScan)

        #running indicator
        self.runningIndicator = QtGui.QRadioButton('GA running')
        self.gridLayout3.addWidget(self.runningIndicator, 2, 1)

    def setPopSize(self):
        self.populationSize = self.popSizeSpinbox.value()
        self.calcIdx()

    def setElitismCoef(self):
        self.elitismCoef = self.elitismCoefSpinbox.value()
        self.calcIdx()
        print 'Setting elitism to', self.elitismCoef

    def setMutationCoef(self):
        print "setmut"
        self.mutationCoef = self.mutationCoefSpinbox.value()
        self.calcIdx()

    def setCrossOverCoef(self):
        self.crossOverCoef = self.crossOverCoefSpinbox.value()
        self.calcIdx()

    def calcIdx(self):
        pass
        # returns the indices of new generation individuals to be created through elitism, mutation and x-over
        self.elitismIdx = np.arange(0, int(max(1,np.floor(self.populationSize * self.elitismCoef))))
        self.mutationIdx = np.arange(int(max(self.elitismIdx))+1, int(max(self.elitismIdx)+1  + np.floor(self.populationSize * self.mutationCoef)))
        self.crossOverIdx = np.arange(max(self.mutationIdx)+1, self.populationSize)

    def startScan(self):
        print 'Scan started'
        self.scanData = None
        self.trendData = None
        #self.running = True
        self.genData = np.array([])
        self.maxFitnessTrend = np.array([])
        self.avgFitnessTrend = np.array([])
        self.generation = 0
        self.initGA()
        self.scanUpdateAction()


    def stopScan(self):
        print 'Scan stopped'
        self.scanData = None
        #self.running = False
        self.scanTimer.stop()


    def scanUpdateAction(self):
        time.sleep(0.0)

        if self.generation  == 0:
            print 'Current gen: ', self.generation
            self.fitness = 0.0
            self.calcIdx()
            self.fitnessTrend = []
            self.generateInitialPopulation(self.populationSize, self.numberOfGenes)
            print 'Generating initial population'
        else:
            print 'Current gen: ', self.generation
            self.genData = np.append(self.genData, self.generation)
            self.newGeneration()
            self.maxFitnessTrend = np.append(self.maxFitnessTrend,np.max(self.fitness))
            self.avgFitnessTrend = np.append(self.avgFitnessTrend,np.average(self.fitness))
            self.plot21.setData(self.genData,self.maxFitnessTrend)
            self.plot22.setData(self.genData,self.avgFitnessTrend, color='r')
            self.plot22.setPen('r')

            spots = []
            for i in range(0,self.numberOfGenes):
                spots.append({'pos': self.electrodeCoordinates[i,:], 'data': 1, 'brush':pq.intColor(np.round(self.population[9,i])), 'symbol': 'o', 'size': 50})

            #spots = [{'pos': self.electrodeCoordinates[i,:], 'data': 1, 'brush':pq.hsvColor(0.7), 'symbol': 'o', 'size': 50} for i in range(self.numberOfGenes)]
            self.plot11.addPoints(spots)
        #print type(pq.intColor(np.round(self.population[0,9])))

        self.generation += 1

        #print 'Generations: ', self.genData
        #print 'Fitness trend: ', self.fitnessTrend

        self.scanTimer.start(100)
        #print self.elitismCoef


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    myapp = myWidgetTest()
    myapp.show()
    sys.exit(app.exec_())
