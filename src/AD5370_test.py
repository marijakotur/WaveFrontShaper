'''
Created on 28 sep 2016

@author: laser
'''

import PyTango as pt
import time
import pyqtgraph as pq
from PyQt4 import QtGui
import sys
import numpy as np
from PyQt4 import QtGui, QtCore # (the example applies equally well to PySide)



class TangoDeviceClient(QtGui.QWidget):
    def __init__(self, parent = None):
        QtGui.QWidget.__init__(self, parent)
        
        self.cameraName = 'gunlaser/thg/camera'
        self.cameraDevice = pt.DeviceProxy(self.cameraName)
        self.image = self.cameraDevice.read_attribute('Image').value.astype(np.double)
        
        self.adName = 'gunlaser/devices/ad5370dac'
        self.adDevice = pt.DeviceProxy(self.adName)
        #self.ad = self.adName.read_attribute('channel0')
        
        self.maxVoltage = 3.0 * 73;

        self.adDevice.write_attribute('channel0',1)
        time.sleep(0.)
        #voltages = self.adName.read_attribute('channel0')
        

        self.myLayout()
        self.setVoltages()
        
        #=======================================================================
        # self.scanTimer = QtCore.QTimer()
        # self.scanTimer.timeout.connect(self.setVoltages)
        #=======================================================================

    
    def setVoltages(self):
        a = 1.0
        h = a*np.sqrt(3)/2
        self.electrodeCoordinates = [[-3*a/2,3*h],[-a/2,3*h],[a/2,3*h],[3*a/2,3*h],[-2*a,2*h],[-a,2*h],[0,2*h],[a,2*h],[2*a,2*h],
[-5*a/2,h],[-3*a/2,h],[-a/2,h],[a/2,h],[3*a/2,h],[5*a/2,h],[-3*a,0],[-2*a,0],[-a,0],[0,0],[a,0],[2*a,0],[3*a,0],
[-5*a/2,-h],[-3*a/2,-h],[-a/2,-h],[a/2,-h],[3*a/2,-h],[5*a/2,-h],
[-2*a,-2*h],[-a,-2*h],[0,-2*h],[a,-2*h],[2*a,-2*h],[-3*a/2,-3*h],[-a/2,-3*h],[a/2,-3*h],[3*a/2,-3*h]]
        self.electrodeCoordinates = np.array(self.electrodeCoordinates)
        
        spots = []
        for i in range(0,37):
            attr_name = 'channel'+str(i)
            self.adDevice.write_attribute(attr_name,self.voltageSliders[i].value()/73.0)
            self.voltageIndicators[i].setText("{:.2f}".format(self.voltageSliders[i].value()))
            #spots.append({'pos': self.electrodeCoordinates[i,:], 'data': 1, 'brush':pq.intColor(np.round(self.voltageSliders[i].value())), 'symbol': 'o', 'size': 50})
            spots.append({'pos': self.electrodeCoordinates[i,:], 'data': 1, 'brush':pq.hsvColor(self.voltageSliders[i].value()/(self.maxVoltage*5.5), sat=1.0, val=1.0, alpha=1.0), 'symbol': 'o', 'size': 40})
            #spots.append({'pos': self.electrodeCoordinates[i,:], 'data': 1, 'brush':self.map.map(self.voltageSliders[i].value(),mode='qcolor'), 'symbol': 'o', 'size': 40})
            
            
        self.plot11.addPoints(spots)
        #print self.voltageSliders
    
    #===========================================================================
    # def readVoltages(self):
    #     print 'reading voltages'
    #     for i in range(0,len(self.voltageSliders)):
    #         #print self.slider.value()
    #         #self.voltageLabel.setText("{:.2f}".format(self.slider.value()))
    #         self.voltageIndicators[i].setText("{:.2f}".format(self.voltageSliders[i].value()))
    #         print self.voltageSliders[i].value()
    #===========================================================================


    
    def myLayout(self):
        
        self.layout = QtGui.QHBoxLayout(self) #the whole window, main layout
        self.slidersLayout = QtGui.QGridLayout()
        self.otherLayout = QtGui.QVBoxLayout()
        self.electrodesLayout = QtGui.QGridLayout()
        self.cameraLayout = QtGui.QVBoxLayout()
        
        self.layout.addLayout(self.slidersLayout)
        self.layout.addLayout(self.otherLayout)
        self.otherLayout.addLayout(self.electrodesLayout)
        self.otherLayout.addLayout(self.cameraLayout)
        
        self.plotWidget1 = pq.PlotWidget()
        self.plot11 = pq.ScatterPlotItem(size=50, pen=pq.mkPen('w'), pxMode=True, symbol='o')
        self.plotWidget1.addItem(self.plot11)
        self.plotWidget1.setAspectLocked(1)
        self.plotWidget1.setVisible(True)
        self.plotWidget1.setMaximumSize(350,350)
        self.electrodesLayout.addWidget(self.plotWidget1)
        
#===============================================================================
#         self.ch0slider = QtGui.QSlider()
#         self.gridLayout1.addWidget(self.ch0slider,0,0)
#         #self.gridLayout1.addWidget(QtGui.QLabel(str(0),0,1))
#
#         self.ch1slider = QtGui.QSlider()
#         self.gridLayout1.addWidget(self.ch0slider,0,1) 
#===============================================================================

        self.voltageSliders = []
        self.voltageIndicators = []
        for i in range(0,37):
            
            self.slider = QtGui.QSlider()
            self.slider.setMinimumWidth(40)
            self.slider.setSliderPosition(0.0)
            self.slider.setTickPosition(QtGui.QSlider.TicksLeft)
            self.slider.setTickInterval(10)
            self.slider.setMaximum(self.maxVoltage)
            self.slider.setMinimum(0)
            self.voltageSliders.append(self.slider)
            self.slidersLayout.addWidget(self.slider,i//10*2,i%10)
            self.slider.valueChanged.connect(self.setVoltages)
            
            self.indicator = QtGui.QLabel() 
            self.voltageIndicators.append(self.indicator)
            self.slidersLayout.addWidget(self.indicator,i//10*2+1,i%10)
            
            self.voltageLabel = QtGui.QLabel()
            #self.voltageLabel.setText("{:.2f}".format(-1))

            self.slidersLayout.addWidget(self.voltageLabel)
        
        self.cameraWindow = pq.GraphicsLayoutWidget()
        self.cameraWindow.setMaximumSize(350,350)
        #self.cameraWindow.setSizePolicy(QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Expanding)
        self.cameraLayout.addWidget(self.cameraWindow)
        view = self.cameraWindow.addViewBox()
        view.setAspectLocked(True)
        img = pq.ImageItem(border='w')
        view.addItem(img)
        print type(self.image[0][0])

        #testImage=np.zeros((50, 50))
        #img.setImage(self.image)



if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    myapp = TangoDeviceClient()
    myapp.show()
    sys.exit(app.exec_())