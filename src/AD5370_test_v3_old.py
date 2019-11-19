'''
Created on 3 feb. 2017

@author: Marija
'''


'''
Created on 28 sep 2016

@author: laser
'''

import PyTango as pt
import time
import pyqtgraph as pq
import sys
import numpy as np
from PyQt4 import QtGui, QtCore # (the example applies equally well to PySide)

#===============================================================================
# class MyPlotWidget(pq.PlotWidget):
#     def mousePressEvent(self, ev):
#         if ev.button() == QtCore.Qt.LeftButton:
#             print 'mouse button pressed'
#             item = self.itemAt(ev.pos())
#             print type(item)
#             item.resetPen(pq.intColor(2))
#             item.updateSpots
#         
# #        item.setColor(pq.Color(0,0,255))
#         #print self.itemAt(ev.pos())
#         return pq.PlotWidget.mousePressEvent(self, ev)        
#===============================================================================

#===============================================================================
# class MySpinBox(pq.PlotWidget,num):
#     def spinBoxName(self):
#           self.num = num
#===============================================================================

class TangoDeviceClient(QtGui.QWidget):
    def __init__(self, parent = None):
        QtGui.QWidget.__init__(self, parent)
        
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
        self.cameraDevice.write_attribute('ExposureTime',15000)
        self.image = self.cameraDevice.read_attribute('Image').value.astype(np.double)
        
        
        self.ampFactor = 73.0
        self.maxVoltage = 4.11 * self.ampFactor
        #self.voltages = np.linspace(0, self.maxVoltage, 37)
        #self.voltages = np.linspace(0, 36, 37)
        self.noChannels = 40
        self.voltages = [5]*self.noChannels #for 37 electrodes

        time.sleep(0.)
        #voltages = self.adName.read_attribute('channel0')        

        a = 1.0
        h = a*np.sqrt(3)/2
        self.electrodeCoordinates = [[-3*a/2,3*h],[-a/2,3*h],[a/2,3*h],[3*a/2,3*h],[-2*a,2*h],[-a,2*h],[0,2*h],[a,2*h],[2*a,2*h],
[-5*a/2,h],[-3*a/2,h],[-a/2,h],[a/2,h],[3*a/2,h],[5*a/2,h],[-3*a,0],[-2*a,0],[-a,0],[0,0],[a,0],[2*a,0],[3*a,0],
[-5*a/2,-h],[-3*a/2,-h],[-a/2,-h],[a/2,-h],[3*a/2,-h],[5*a/2,-h],
[-2*a,-2*h],[-a,-2*h],[0,-2*h],[a,-2*h],[2*a,-2*h],[-3*a/2,-3*h],[-a/2,-3*h],[a/2,-3*h],[3*a/2,-3*h]]
        self.electrodeCoordinates = np.array(self.electrodeCoordinates)
        
        #self.channelsList = [2,0,38,36,3,1,39,37,35,6,5,4,34,33,32,7,18,17,27,29,30,31,16,15,13,23,21,20,14,11,9,25,22,12,10,26,24,8,19,28] #old
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
        #outerCircelIndices = []
        #outerCirceleIndices = np.array(outerCirceleIndices)
        #self.outerCircle = 

        self.lastChanged = []
        self.lastClicked = []
        self.lastClickedElectrodeNumber = 666   
        
        self.scanTimer = QtCore.QTimer()
        self.scanTimer.timeout.connect(self.updateImage)
        
        self.myLayout()
        self.setVoltages()
        
        #self.readVoltages()
        
        self.updateImage()
        

    def updateImage(self):
        #print 'updating image'
        self.image = self.cameraDevice.read_attribute('Image').value.astype(np.double)
        self.img.setImage(self.image)
        self.view.update()
        #self.cameraWindow.update()
        self.scanTimer.start(100)

    def readVoltages(self):
        readVoltages = []
        for i in range(0,self.noChannels):
            chName = 'channel'+ str(self.channelsList[i])
            readVoltages.append(self.adDevice.read_attribute(chName).value*self.ampFactor)
        #print readVoltages
    
    def resetAll(self):
        print 'resetting all voltages'
        self.voltages = [5]*self.noChannels
        brushes = [] 
        for i in range(0,40):
            chName = 'channel'+ str(i)
            self.adDevice.write_attribute(chName,self.voltages[i]/self.ampFactor)
            brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))            
            currentSB = self.slidersLayout.itemAt(2*i).widget()
            currentSB.setValue(self.voltages[i])
        self.scatterPlotItem1.setBrush(brushes)
        self.plot11.addItem(self.scatterPlotItem1)

    def updateVoltage(self):
        #self.voltages = self.voltageSliders.value()
        #print self.voltages[self.lastClickedElectrodeNumber]
        #print 'voltage changed'
        if self.lastClickedElectrodeNumber < 50:
            self.voltages[self.lastClickedElectrodeNumber] = self.currentElectrodeSpinBox.value()
            chNumber = self.channelsList[self.lastClickedElectrodeNumber]
            chName = 'channel'+ str(chNumber)
            self.adDevice.write_attribute(chName,self.voltages[self.lastClickedElectrodeNumber]/self.ampFactor)
            currentVoltage = self.adDevice.read_attribute(chName).value*self.ampFactor
            print 'set voltage on electrode ' + str(self.lastClickedElectrodeNumber) + ' to ' + str(self.voltages[self.lastClickedElectrodeNumber]) + ' V' 
            print 'read voltage from electrode '+str(self.lastClickedElectrodeNumber)+'('+ chName +')'+ ' is '+str(currentVoltage)
            #print dir(currentVoltage)
                                    
        brushes = [] 
        for i in range(0,40):
            brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))
            currentSB = self.slidersLayout.itemAt(2*i).widget()
            currentSB.setValue(self.voltages[i])

        self.scatterPlotItem1.setBrush(brushes)
        self.plot11.addItem(self.scatterPlotItem1)
               
        
        #self.scatterPlotItem1.updateSpots(dataSet)
        #p.setBrush(pq.hsvColor(10/(self.maxVoltage*5.5), sat=1.0, val=1.0, alpha=1.0))

        
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
            for i in range(0,37):
                if (self.electrodeCoordinates[i][0] == point.x()) and (self.electrodeCoordinates[i][1] == point.y()):
                    print 'electrode # is ' + str(i) 
                    self.lastClickedElectrodeNumber = i
                    self.currentElectrodeSpinBox.setValue(self.voltages[self.lastClickedElectrodeNumber])
                    self.numberLabel.setText("{:.2f}".format(self.lastClickedElectrodeNumber))
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
        
        self.plot11 = self.plotWidget1.addPlot()
        #self.scatterPlotItem1 = pq.ScatterPlotItem(size=30, pen=pq.mkPen(None), brush=pq.mkBrush(0, 0, 125+self.voltages[0]/2, 250))
        self.scatterPlotItem1 = pq.ScatterPlotItem(size=30, pen=pq.mkPen(None))
        self.scatterPlotItem1.addPoints(x=self.electrodeCoordinates[:,0], y=self.electrodeCoordinates[:,1])
        
        
        
        brushes = []
        for i in range(0,self.noChannels):
            chName = 'channel'+ str(self.channelsList[i])
            self.adDevice.write_attribute(chName,self.voltages[i]/self.ampFactor)
            if i<=36:
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
        
    
    def myLayout(self):
        
        self.layout = QtGui.QHBoxLayout(self) #the whole window, main layout
        self.leftLayout = QtGui.QVBoxLayout()
        self.rightLayout = QtGui.QVBoxLayout()
        self.numberLayout = QtGui.QGridLayout()
        self.slidersLayout = QtGui.QGridLayout()
        self.electrodesLayout = QtGui.QGridLayout()
        self.cameraLayout = QtGui.QVBoxLayout()
        
        self.layout.addLayout(self.leftLayout)
        self.layout.addLayout(self.rightLayout)
        self.leftLayout.addLayout(self.slidersLayout)
        self.leftLayout.addLayout(self.numberLayout)
        self.rightLayout.addLayout(self.electrodesLayout)
        self.rightLayout.addLayout(self.cameraLayout)
        
        self.numberLabel = QtGui.QLabel()
        self.numberLayout.addWidget(self.numberLabel,0,0)
        self.numberLabel.setText("{:.2f}".format(self.lastClickedElectrodeNumber))
        self.numberLabel.setFont(QtGui.QFont('SansSerif', 25))
        
#        self.plotWidget1 = pq.PlotWidget()
        self.plotWidget1 = pq.GraphicsLayoutWidget()
        self.plotWidget1.setAspectLocked(1)
        self.plotWidget1.setVisible(True)
        self.plotWidget1.setMaximumSize(350,350)
        self.electrodesLayout.addWidget(self.plotWidget1)
        

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
            
            #===================================================================
            # self.slider = QtGui.QSlider()
            # self.slider.setMinimumWidth(40)
            # self.slider.setSliderPosition(0.0)
            # self.slider.setTickPosition(QtGui.QSlider.TicksLeft) 
            # self.slider.setTickInterval(10)
            # self.slider.setMaximum(self.maxVoltage)
            # self.slider.setMinimum(0)
            # self.slidersLayout.addWidget(self.slider,i//10*2,i%10)
            # self.slider.valueChanged.connect(self.updateVoltage)
            #===================================================================
            
            #self.voltageSliders.append(self.slider)
            
            #===================================================================
            # self.indicator = QtGui.QLabel()
            # self.indicator.setText("{:.2f}".format(self.voltages[i]))
            # self.slidersLayout.addWidget(self.indicator,i//8*2,i%8)
            #===================================================================

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
                    
            
            #self.slidersLayout.addItem(QtGui.QSpacerItem(0, 50, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum))
            #if i%10 == 0:
            
        self.cameraWindow = pq.GraphicsLayoutWidget()
        self.cameraWindow.setMaximumSize(350,350)
        #self.cameraWindow.setSizePolicy(QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Expanding)
        self.cameraLayout.addWidget(self.cameraWindow)
        self.view = self.cameraWindow.addViewBox()
        self.view.setAspectLocked(True)
        self.img = pq.ImageItem(border='w')
        self.view.addItem(self.img)
        self.img.setImage(self.image)
        self.updateImage()

        #print type(self.image[0][0])

        #testImage=np.zeros((50, 50))
        



if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    myapp = TangoDeviceClient()
    myapp.show()
    sys.exit(app.exec_())
    