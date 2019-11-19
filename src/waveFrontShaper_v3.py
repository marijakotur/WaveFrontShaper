'''
Created on 15 sep. 2016

@author: Marija
'''
'''
Created on 24 aug. 2016

@author: Marija
'''
from PyQt5 import QtGui, QtCore # (the example applies equally well to PySide)
import sys
import numpy as np
#import threading
import time
#from blaze.expr.expressions import symbol
import pyqtgraph as pq
import PyTango as pt

class myWidgetTest(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        #self.settings = QtCore.QSettings('MaxIV', 'GA')
             
        
        self.adName = 'gunlaser/devices/ad5370dac'
        self.adDevice = pt.DeviceProxy(self.adName)
        #self.ad = self.adName.read_attribute('channel0')
        
        #self.cameraName = 'gunlaser/thg/camera'
        self.cameraName = 'gunlaser/cameras/blackfly_test01'
        self.cameraDevice = pt.DeviceProxy(self.cameraName)
        self.cameraDevice.set_timeout_millis(3000)
        #expTime = self.cameraDevice.read_attribute('ExposureTime').value
        #print expTime
        self.cameraDevice.write_attribute('Gain',15)
        self.cameraDevice.write_attribute('ExposureTime',1000)
        self.image = self.cameraDevice.read_attribute('Image').value.astype(np.double)
        
        
        self.ampFactor = 73.0
        self.maxVoltage = 4.1 * self.ampFactor
        #self.voltages = np.linspace(0, self.maxVoltage, 37)
        #self.voltages = np.linspace(0, 36, 37)
        self.noChannels = 40
        self.voltages = [5]*self.noChannels #for 37 electrodes
          
        self.scanTimer = QtCore.QTimer()
        self.scanTimer.timeout.connect(self.updateImage)
          
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
[-3*a/2,-3*h],[-a/2,-3*h],[a/2,-3*h],[3*a/2,-3*h],[-9*a/2,-3*h],[-7*a/2,-3*h],[-4*a,-2*h]]
        self.electrodeCoordinates = np.array(self.electrodeCoordinates)
        
        self.channelsList = [23,21,0,2,
                             24,22,20,1,3,
                             27,26,25,4,5,6,
                             28,29,30,32,17,18,7,
                             39,38,36,13,15,16,
                             37,34,9,11,14,
                             35,33,10,12,
                             8,19,31]
        self.channelsOuter = [23,21,0,2,24,3,27,6,28,7,39,16,37,14,35,33,10,12]
        self.channelsMiddle = [22,20,1,26,5,29,18,38,15,34,9,11]
        self.channelsInner = [25,4,30,17,36,13]
        self.channelsCenter = [32]
        self.channelsDisconnected = [8,19,31]
              
        self.populationSize = 20
        self.numberOfGenes = 40
        self.population = []   
        
        self.elitismCoef = 0.2
        self.mutationCoef = 0.2
        self.twoPtXoverCoef = 0.6
        self.calcIdx()
        
        self.fitness = [None] * self.populationSize
        self.geneLowerBound = 0
        self.geneUpperBound = self.maxVoltage
        self.creepRate = 0.1  
                
        self.fitnessCostWeight = 0.25 / self.populationSize
        self.referencePopulation = [0]*self.numberOfGenes
       
    def newGeneration(self):
        sortedPopulation = np.copy(self.sortByFitness(self.population))   
        #print 'sorted population shape' + str(sortedPopulation.shape)
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
                #print 'individual to mutate' + str(j)
                print indToMutate.shape
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
            lower = self.geneLowerBound + (self.geneUpperBound-self.geneLowerBound)*2/5
            upper = self.geneLowerBound + (self.geneUpperBound-self.geneLowerBound)*4/5
            self.population.append(np.random.uniform(low=lower, high=upper, size=self.numberOfGenes))
        #print self.population
        self.population = np.array(self.population)
        print 'population shape' + str(self.population.shape)
        #print self.population.shape

    def sortByFitness(self, population):
        print 'measuring fitness'
        self.fitness = []
        #measure fitness
        image_size_x = self.image.shape[0]
        image_size_y = self.image.shape[1]
        for individual in population:
            for chInd, channel in enumerate(self.channelsList):
                chName = 'channel'+ str(self.channelsList[chInd])
                #print chInd, channel
                self.adDevice.write_attribute(chName,individual[chInd]/self.ampFactor)
            self.updateImage()  
            s = -self.fwhm(np.sum(self.image[525:625,600:700],axis=0))-self.fwhm(np.sum(self.image[525:625,600:700],axis=1))
            self.fitness.append(s)
        #sort by fitness          
        j = np.argsort(self.fitness)
        j = np.flipud(j)
        population = population[j]
        population = np.squeeze(population)
        #write the best shape again before updating image
        for chInd, channel in enumerate(self.channelsList):
                chName = 'channel'+ str(self.channelsList[chInd])
                self.adDevice.write_attribute(chName,population[0][chInd]/self.ampFactor)
        self.updateImage()  

        return population                 
        
    def creepMutation(self,individualToMutate):
        #change one gene according to
        #g_new = min( imax, max( imin, g_old + r*s*(imax-imin)
        # r - random number in the [-1,1] range
        # s - creep rate, usually 0.02
        # imin, imax - gene range bounds
        mutationLocation = np.random.randint(0,self.numberOfGenes)
        #print "mutation location is %d" %mutationLocation
        oldGene = individualToMutate[mutationLocation]
        r = np.random.uniform(low=-1.0, high=1.0, size=1)
        newGene = min( self.geneUpperBound, max( self.geneLowerBound, oldGene + r*self.creepRate*(self.geneUpperBound-self.geneLowerBound)))
        individualToMutate[mutationLocation] = newGene
        return individualToMutate    

    def updateImage(self):
        #print 'updating image'
        self.image = self.cameraDevice.read_attribute('Image').value.astype(np.double)
        self.img.setImage(self.image[525:625,600:700])
        self.view.update()
        #self.cameraWindow.update()
        self.scanTimer.start(100)

    def myLayout(self):
        self.layout = QtGui.QVBoxLayout(self) #the whole window, main layout
        self.inputLayout = QtGui.QHBoxLayout() 
        self.plotLayout = QtGui.QHBoxLayout() 
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
        self.numberOfGenesSpinbox.valueChanged.connect(self.setNumberOfGenes)
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
        self.cameraWindow.setMaximumSize(350,350)
        #self.cameraWindow.setSizePolicy(QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Expanding)
        self.view = self.cameraWindow.addViewBox()
        self.view.setAspectLocked(True)
        self.img = pq.ImageItem(border='w')
        self.view.addItem(self.img)
        self.img.setImage(self.image)
        self.updateImage()
                        
        self.plotLayout = QtGui.QHBoxLayout()      
        self.plotLayout.addWidget(self.plotWidget2)
        self.plotWidget1.setVisible(True)
        self.plotWidget1.setMaximumHeight(400)
        self.plotLayout.addWidget(self.plotWidget1)
        self.plotWidget2.setVisible(True)
        self.plotWidget2.setMaximumHeight(400)
        self.plotLayout.addWidget(self.cameraWindow)
        
        self.layout.addLayout(self.inputLayout)
        self.layout.addLayout(self.plotLayout)
        self.inputLayout.addLayout(self.gridLayout1)
        self.inputLayout.addLayout(self.gridLayout2)
        self.inputLayout.addLayout(self.gridLayout3)
        #self.inputLayout.addSpacerItem(QtGui.QSpacerItem(10, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum))
        self.layout.addLayout(self.plotLayout)

        self.startButton = QtGui.QPushButton('Start')
        self.startButton.clicked.connect(self.startScan)
        self.stopButton = QtGui.QPushButton('Stop')
        self.stopButton.clicked.connect(self.stopScan)
    
        #running indicator
        self.runningIndicator = QtGui.QRadioButton('GA running')
        self.gridLayout3.addWidget(self.runningIndicator, 2, 1)
    
        #read out values from boxes
        self.setNumberOfGenes()
        self.setPopSize()
        self.setCrossOverCoef()
        self.setMutationCoef()
        self.setElitismCoef()
        
    def fwhm(self,data):
        data = np.abs(data)
        data /= np.max(data)
        a = [np.diff(np.sign(data-0.5))]
        nonzeros = np.nonzero(a[0])
        fwhm = np.max(nonzeros)-np.min(nonzeros)
        #print np.min(nonzeros), np.max(nonzeros)
        return fwhm   
     
    def setPopSize(self):
        self.populationSize = self.popSizeSpinbox.value()
        self.calcIdx()
    
    def setNumberOfGenes(self):
        self.numberOfGenes = self.numberOfGenesSpinbox.value()
        print 'setting no of genes to ' + str(self.numberOfGenes)
    
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
        # returns the indices of new generation individuals to be created through elitism, mutation and x-over 
        self.elitismIdx = np.arange(0, int(max(1,np.floor((self.populationSize-1) * self.elitismCoef))))
        self.crossOverIdx = np.arange(int(max(self.elitismIdx))+1, int(max(self.elitismIdx)+1  + np.floor((self.populationSize-1) * self.twoPtXoverCoef)))
        self.mutationIdx = np.arange(max(self.crossOverIdx)+1, self.populationSize)
        
        print 'elitismIdx=' + str(self.elitismIdx)+ 'mutationIdx='+str(self.mutationIdx)+ 'XoverIdx=' +str(self.crossOverIdx)
        
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
            print 'Generating initial population'
            print 'Current gen: ', self.generation
            self.fitness = 0.0  
            self.calcIdx()
            self.fitnessTrend = []
            self.generateInitialPopulation(self.populationSize, self.numberOfGenes)
        else:     
            print 'Current gen: ', self.generation
            self.genData = np.append(self.genData, self.generation)
            self.newGeneration()
            self.maxFitnessTrend = np.append(self.maxFitnessTrend,np.max(self.fitness))
            self.avgFitnessTrend = np.append(self.avgFitnessTrend,np.average(self.fitness))
            self.plot21.setData(self.genData,self.maxFitnessTrend)
            self.plot22.setData(self.genData,self.avgFitnessTrend, color='r')
            self.plot21.setPen('w')
            self.plot22.setPen('r')
               
            spots = []
            for i in range(0,self.numberOfGenes):
                #chName = 'channel'+ str(self.channelsList[i])
                #self.adDevice.write_attribute(chName,self.voltages[i]/self.ampFactor)       
                spots.append({'pos': self.electrodeCoordinates[i,:], 'data': 1, 'brush':pq.intColor(np.round(self.population[0,i])), 'symbol': 'o', 'size': 50})

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
