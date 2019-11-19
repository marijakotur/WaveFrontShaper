'''
Created on 16 mars 2017

@author: Marija
'''


import PyTango as pt
import time
import pyqtgraph as pq
import sys
import numpy as np
from PyQt4 import QtGui, QtCore # (the example applies equally well to PySide)
import matplotlib.pyplot as plt
from scipy import misc


class TangoDeviceClient(QtGui.QWidget):
    def __init__(self, parent = None):
        QtGui.QWidget.__init__(self, parent)
        
        #=======================================================================
        # self.adName = 'gunlaser/devices/ad5370dac'
        # self.adDevice = pt.DeviceProxy(self.adName)
        #=======================================================================
        
        #=======================================================================
        # self.cameraName = 'gunlaser/cameras/blackfly_test01'
        # self.cameraDevice = pt.DeviceProxy(self.cameraName)
        # self.cameraDevice.set_timeout_millis(3000)
        # self.cameraDevice.write_attribute('Gain',24)
        # self.cameraDevice.write_attribute('ExposureTime',4500)
        # self.image = self.cameraDevice.read_attribute('Image').value.astype(np.double)
        #=======================================================================
        
                
        self.ampFactor = 73.0
        self.maxVoltage = 350 #5 * self.ampFactor
        self.noChannels = 40
        self.voltages = [5]*self.noChannels #for 37 electrodes

        #time.sleep(0.)
        #voltages = self.adName.read_attribute('channel0')        

        a = 1.0
        h = a*np.sqrt(3)/2
        self.electrodeCoordinates = [[-3*a/2,3*h],[-a/2,3*h],[a/2,3*h],[3*a/2,3*h],[-2*a,2*h],[-a,2*h],[0,2*h],[a,2*h],[2*a,2*h],
[-5*a/2,h],[-3*a/2,h],[-a/2,h],[a/2,h],[3*a/2,h],[5*a/2,h],[-3*a,0],[-2*a,0],[-a,0],[0,0],[a,0],[2*a,0],[3*a,0],
[-5*a/2,-h],[-3*a/2,-h],[-a/2,-h],[a/2,-h],[3*a/2,-h],[5*a/2,-h],
[-2*a,-2*h],[-a,-2*h],[0,-2*h],[a,-2*h],[2*a,-2*h],[-3*a/2,-3*h],[-a/2,-3*h],[a/2,-3*h],[3*a/2,-3*h],[-9*a/2,-3*h],[-7*a/2,-3*h],[-4*a,-2*h]]
        self.electrodeCoordinates = np.array(self.electrodeCoordinates)
        
        self.channelsList = [23,21,0,2,
                             24,22,20,1,3,
                             27,26,25,4,5,6,
                             28,29,30,32,17,18,7,
                             39,38,36,13,15,16,
                             37,34,9,11,14,
                             35,33,10,12,8,19,31]
        self.channelsOuter = [23,21,0,2,24,3,27,6,28,7,39,16,37,14,35,33,10,12,8,19,31]
        self.channelsMiddle = [22,20,1,26,5,29,18,38,15,34,9,11]
        self.channelsInner = [25,4,30,17,36,13]
        self.channelsCenter = [32]
        self.channelsDisconnected = [8,19,31]
 
        self.lastChanged = []
        self.lastClicked = []
        self.lastClickedElectrodeNumber = 666   
        
        self.scanTimer = QtCore.QTimer()

        
        self.myLayout()
        self.setVoltages()
        
        #self.readVoltages()
        
        

    def resetAll(self):
        print 'resetting all voltages'
        self.voltages = [1]*self.noChannels
        brushes = [] 
        for i in range(0,40):
            brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))            
            currentSB = self.slidersLayout.itemAt(2*i).widget()
            currentSB.setValue(self.voltages[i])
        self.scatterPlotItem1.setBrush(brushes)
        self.plot11.addItem(self.scatterPlotItem1)

    def updateVoltage(self):                                    
        brushes = [] 
        for i in range(0,40):
            brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))
            currentSB = self.slidersLayout.itemAt(2*i).widget()
            currentSB.setValue(self.voltages[i])

        self.scatterPlotItem1.setBrush(brushes)
        self.plot11.addItem(self.scatterPlotItem1)      
        

    def setToMaxVoltage(self):
        if self.lastClickedElectrodeNumber < 50:
            self.voltages[self.lastClickedElectrodeNumber] = self.maxVoltage


        brushes = [] 
        for i in range(0,40):
            brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))
            currentSB = self.slidersLayout.itemAt(2*i).widget()
            currentSB.setValue(self.voltages[i])

        self.scatterPlotItem1.setBrush(brushes)
        self.plot11.addItem(self.scatterPlotItem1)    
        
        #=======================================================================
        # fwhm_x = self.fwhm(np.sum(self.image[300:700,400:800],axis=0))
        # print fwhm_x
        # fwhm_y = self.fwhm(np.sum(self.image[300:700,400:800],axis=1))
        # print fwhm_y
        #=======================================================================
        
    def clicked(self, plot, points):
        #global lastClicked
        for p in self.lastClicked:
            p.resetPen()
            #p.resetBrush()
        for p in points:
            p.setPen('g', width=2)
            point = p.pos()
            #print point.x()
            #print dir(p)
            for i in range(0,40):
                if (self.electrodeCoordinates[i][0] == point.x()) and (self.electrodeCoordinates[i][1] == point.y()):
                    print 'electrode # is ' + str(i) 
                    self.lastClickedElectrodeNumber = i
                    self.currentElectrodeSpinBox.setValue(self.voltages[self.lastClickedElectrodeNumber])
                    #self.numberLabel.setText("{:.2f}".format(self.lastClickedElectrodeNumber))
                    #currentVoltage = self.voltageSliders[i].value()
            #print 'the voltage is: ' + str(self.voltageSliders[self.lastClickedElectrodeNumber].value())
       
        self.lastClicked = points
        
    
    def setVoltages(self):
                   
        self.spot_plot_list = []
        #=======================================================================
        # for i in range(0,37):
        #     self.voltageIndicators[i].setText("{:.2f}".format(self.voltageSliders[i].value()))
        #     #spots.append({'pos': self.electrodeCoordinates[i,:], 'data': 1, 'brush':pq.intColor(np.round(self.voltageSliders[i].value())), 'symbol': 'o', 'size': 50})
        #     spot = {'pos': self.electrodeCoordinates[i,:],'data':1,'brush':pq.hsvColor(self.voltageSliders[i].value()/(self.maxVoltage*5.5),sat=1.0,val=1.0, alpha=1.0),'symbol':'o','size': 40,'pen':0}
        #     #spots.append(spot)
        #     #spots.append({'pos': self.electrodeCoordinates[i,:], 'data': 1, 'brush':self.map.map(self.voltageSliders[i].value(),mode='qcolor'), 'symbol': 'o', 'size': 40})           
        #     plot = pq.ScatterPlotItem(size=50, pxMode=True, symbol='o', name=str(i))
        #     plot.addPoints([spot])
        #     self.spot_plot_list.append(plot)
        #     self.plotWidget1.addItem(plot)
        #=======================================================================
        
        #self.scatterPlotItem1 = pq.ScatterPlotItem(size=30, pen=pq.mkPen(None), brush=pq.mkBrush(0, 0, 125+self.voltages[0]/2, 250))
        self.scatterPlotItem1.addPoints(x=self.electrodeCoordinates[:,0], y=self.electrodeCoordinates[:,1])
               
        
        brushes = []
        for i in range(0,self.noChannels):
            if i<=len(self.electrodeCoordinates):
                brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))
            #self.labelItem1 = pq.LabelItem()
            #self.labelItem1.setPos(self.electrodeCoordinates[0][0],self.electrodeCoordinates[0][1])
            #self.labelItem1.setPos(0,0)
            #self.labelItem1.setText('A', size = '8pt')
            #self.plotWidget1.addItem(self.labelItem1)
            #self.plot11.addItem(self.labelItem1)
            
        #for i in range(0,37)
        #print type(brushes)
        self.scatterPlotItem1.setBrush(brushes)
        self.plot11.addItem(self.scatterPlotItem1)
        self.scatterPlotItem1.sigClicked.connect(self.clicked)
        
        self.textItem = pq.TextItem()
        self.plot11.addItem(self.textItem)
        
    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)
    
    def xTiltCalc(self):
        xTiltCoef = self.xTiltSpinBox.value()
        brushes = [] 
        for i, electrode in enumerate(self.electrodeCoordinates):
            self.voltages[i] = self.clamp(self.maxVoltage/2 + electrode[0]*xTiltCoef, 0, self.maxVoltage)  
            brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))    
        print self.voltages         
        #self.scatterPlotItem1.setBrush(brushes)
        self.plot11.addItem(self.scatterPlotItem1)  
        
    
    def myLayout(self):
        
        self.layout = QtGui.QHBoxLayout(self) #the whole window, main layout
        self.leftLayout = QtGui.QVBoxLayout()
        self.middleLayout = QtGui.QVBoxLayout()
        self.rightLayout = QtGui.QVBoxLayout()
        self.numberLayout = QtGui.QGridLayout()
        self.slidersLayout = QtGui.QGridLayout()
        self.electrodesLayout = QtGui.QGridLayout()
        self.controlsLayout = QtGui.QGridLayout()
        #self.outlinesLayout = QtGui.QGridLayout()
        
        self.layout.addLayout(self.leftLayout)
        self.layout.addLayout(self.middleLayout)
        self.layout.addLayout(self.rightLayout)
        self.leftLayout.addLayout(self.slidersLayout)
        self.leftLayout.addLayout(self.numberLayout)
        self.middleLayout.addLayout(self.electrodesLayout)
        self.middleLayout.addLayout(self.controlsLayout)
        #self.middleLayout.addLayout(self.outlinesLayout)
                
        
#        self.plotWidget1 = pq.PlotWidget()
        self.plotWidget1 = pq.GraphicsLayoutWidget()
        self.plotWidget1.setAspectLocked(1)
        self.plotWidget1.setVisible(True)
        self.plotWidget1.setMaximumSize(350,350)
        self.plot11 = self.plotWidget1.addPlot()
        self.scatterPlotItem1 = pq.ScatterPlotItem(size=30, pen=pq.mkPen(None))
        self.electrodesLayout.addWidget(self.plotWidget1)
        
        self.plotWidget2 = pq.PlotWidget()
        self.plotWidget2.setMaximumSize(350,200)
        self.plot21 = self.plotWidget2.plot()
        self.plot22 = self.plotWidget2.plot()
        #self.plotWidget2.setAspectLocked(1)  
        #self.plot21.setData(np.sum(self.image,axis=0),pen='w',name='white plot')
        #self.plot22.setData(np.sum(self.image,axis=1),pen='r',name='red plot')
        #self.plot21.addLegend(size=None, offset=(30, 30))
        #print np.sum(self.image[300:700,400:800],axis=0).shape
        #print self.fwhm(self.image[300:700,400:800])
        #fwhm_x = self.fwhm(np.sum(self.image,axis=0))
        #print fwhm_x
        #fwhm_y = self.fwhm(np.sum(self.image,axis=1))
        #print fwhm_y
        self.leg = pq.LegendItem()
        self.leg.addItem(self.plot21, 'hor')
        self.leg.addItem(self.plot22, 'vert')
        self.plotWidget2.addItem(self.leg,offset=(30, 30),size=None)


        self.voltageSliders = []
        self.voltageIndicators = []
        self.labelPositions = []
        #spacer = QtGui.QSpacerItem(0,100)
        for i in range(0,40):
            
            self.slider = QtGui.QSpinBox()
            self.slider.setMaximum(self.maxVoltage)
            self.slider.setMinimum(0)
            self.slider.setValue(self.voltages[i])
            self.slidersLayout.addWidget(self.slider,i//7*2,i%7)
            

            self.voltageLabel = QtGui.QLabel()
            self.slidersLayout.addWidget(self.voltageLabel,i//7*2+1,i%7)
            self.voltageLabel.setText("{:.2f}".format(i))
            
            self.labelPositions.append([i//7*2, i%7])
        
        self.currentElectrodeSpinBox = QtGui.QDoubleSpinBox()
        self.currentElectrodeSpinBox.setMaximum(self.maxVoltage)
        self.currentElectrodeSpinBox.setMinimum(0)
        self.numberLayout.addWidget(self.currentElectrodeSpinBox)
        self.currentElectrodeSpinBox.valueChanged.connect(self.updateVoltage)
        
        self.resetAllVoltagesButton = QtGui.QPushButton('Reset all')
        self.resetAllVoltagesButton.clicked.connect(self.resetAll)
        self.numberLayout.addWidget(self.resetAllVoltagesButton)
        
        self.setToMaxButton = QtGui.QPushButton('Set to max')
        self.setToMaxButton.clicked.connect(self.setToMaxVoltage)
        self.numberLayout.addWidget(self.setToMaxButton)
        
        # control widgets        
        self.xTiltSpinBox = QtGui.QDoubleSpinBox()
        self.xTiltSpinBox.setMaximum(120)
        self.xTiltSpinBox.setValue(20)
        self.xTiltSpinBox.valueChanged.connect(self.xTiltCalc)
        self.xTiltLabel = QtGui.QLabel('x tilt')
        self.controlsLayout.addWidget(self.xTiltSpinBox,0,1)
        self.controlsLayout.addWidget(self.xTiltLabel,0,0)
        
        self.xTiltCalc()


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    myapp = TangoDeviceClient()
    myapp.show()
    sys.exit(app.exec_())
    