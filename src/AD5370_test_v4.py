'''
Created on 28 sep 2016

@author: laser
'''

import pyqtgraph as pq
import matplotlib.pyplot as plt
import PyTango as pt
import time
import sys
import numpy as np
from PyQt4 import QtGui, QtCore # (the example applies equally well to PySide)
#from scipy import misc

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
        self.cameraDevice.write_attribute('Gain',24)
        self.cameraDevice.write_attribute('ExposureTime',4500)
        self.image = self.cameraDevice.read_attribute('Image').value.astype(np.double)
        
        #bkgd_image_path = 'C:/Users/Marija/Documents/workspace/simpleGA/src/background_image.png'
        #self.backgroundImage = misc.imread(bkgd_image_path)
        #self.backgroundImage = np.sum(self.backgroundImage,axis=2)
        #print self.backgroundImage.shape
        #self.image = self.backgroundImage
                
        self.ampFactor = 73.0
        self.maxVoltage = 350 #5 * self.ampFactor
        #self.voltages = np.linspace(0, self.maxVoltage, 37)
        #self.voltages = np.linspace(0, 36, 37)
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
                             35,33,10,12,
                             8,19,31]
        self.channelsOuter = [23,21,0,2,24,3,27,6,28,7,39,16,37,14,35,33,10,12,8,19,31]
        self.channelsMiddle = [22,20,1,26,5,29,18,38,15,34,9,11]
        self.channelsInner = [25,4,30,17,36,13]
        self.channelsCenter = [32]
        self.channelsDisconnected = [8,19,31]
 
        self.lastChanged = []
        self.lastClicked = []
        self.lastClickedElectrodeNumber = 666   
        
        self.scanTimer = QtCore.QTimer()
        self.scanTimer.timeout.connect(self.updateImage)
        
        self.myLayout()
        self.setVoltages()
        
        #self.readVoltages()
        
        self.updateImage()
        
    def fwhm(self,data):
        data = np.abs(data)
        data /= np.max(data)
        a = [np.diff(np.sign(data-0.5))]
        nonzeros = np.nonzero(a[0])
        fwhm = np.max(nonzeros)-np.min(nonzeros)
        #print np.min(nonzeros), np.max(nonzeros)
        return fwhm     

    def updateImage(self):
        #print 'updating image'
        self.image = self.cameraDevice.read_attribute('Image').value.astype(np.double)
        self.img.setImage(self.image) #[425:525,500:600]
        self.plot21.setData(np.sum(self.image,axis=0))
        self.plot22.setData(np.sum(self.image,axis=1))
        #=======================================================================
        # self.plot21.setData(np.sum(self.image,axis=0))
        # self.plot22.setData(np.sum(self.image,axis=1))
        #=======================================================================
        self.plot22.setPen('r')   
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
        self.voltages = [1]*self.noChannels
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
        
        #=======================================================================
        # fwhm_x = self.fwhm(np.sum(self.image[300:700,400:800],axis=0))
        # print fwhm_x
        # fwhm_y = self.fwhm(np.sum(self.image[300:700,400:800],axis=1))
        # print fwhm_y
        #=======================================================================
    
    def saveImage(self):
        chNumber = self.channelsList[self.lastClickedElectrodeNumber]
        fileName = 'electrode' + str(self.lastClickedElectrodeNumber) + 'channel' + str(chNumber)
        image = self.image
        plt.imsave(fileName,image)   
        
    def saveImages(self):
        for i in range(0,40):
            self.resetAll()
            self.lastClickedElectrodeNumber = i
            self.setToMaxVoltage()
            time.sleep(0.5)
            self.saveImage()       
                    
    def setToMaxVoltage(self):
        if self.lastClickedElectrodeNumber < 50:
            self.voltages[self.lastClickedElectrodeNumber] = self.maxVoltage
            chNumber = self.channelsList[self.lastClickedElectrodeNumber]
            chName = 'channel'+ str(chNumber)
            self.adDevice.write_attribute(chName,self.voltages[self.lastClickedElectrodeNumber]/self.ampFactor)
            currentVoltage = self.adDevice.read_attribute(chName).value*self.ampFactor
            print 'set voltage on electrode ' + str(self.lastClickedElectrodeNumber) + ' to ' + str(self.voltages[self.lastClickedElectrodeNumber]) + ' V' 
            print 'read voltage from electrode '+str(self.lastClickedElectrodeNumber)+'('+ chName +')'+ ' is '+str(currentVoltage)

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
        self.scatterPlotItem1 = pq.ScatterPlotItem(size=30, pen=pq.mkPen(None))
        self.scatterPlotItem1.addPoints(x=self.electrodeCoordinates[:,0], y=self.electrodeCoordinates[:,1])
        
        
        
        brushes = []
        for i in range(0,self.noChannels):
            chName = 'channel'+ str(self.channelsList[i])
            try:
                self.adDevice.write_attribute(chName,self.voltages[i]/self.ampFactor)
            except:
                pass
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
        print 'setting x tilt'
        xTiltCoef = self.xTiltSpinBox.value()
        brushes = [] 
        for i, electrode in enumerate(self.electrodeCoordinates):
            self.voltages[i] = self.clamp(self.maxVoltage/2 + electrode[0]*xTiltCoef, 0, self.maxVoltage)  
            brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))   
            currentSB = self.slidersLayout.itemAt(2*i).widget()
            currentSB.setValue(self.voltages[i]) 
        print self.voltages         
        self.scatterPlotItem1.setBrush(brushes)
        self.plot11.addItem(self.scatterPlotItem1)     

    def yTiltCalc(self):
        print 'setting y tilt'
        xTiltCoef = self.xTiltSpinBox.value()
        brushes = [] 
        for i,electrode in enumerate(self.electrodeCoordinates):
            self.voltages[i] = self.clamp(self.maxVoltage/2 + electrode[1]*xTiltCoef, 0, self.maxVoltage)  
            brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))   
            currentSB = self.slidersLayout.itemAt(2*i).widget()
            currentSB.setValue(self.voltages[i]) 
        print self.voltages         
        self.scatterPlotItem1.setBrush(brushes)
        self.plot11.addItem(self.scatterPlotItem1)   

    def radial(self,n,m,rho):
        if (n < 0 or abs(m) > n):
            raise ValueError
        if ((n-m) % 2):
            return rho*0.0        
        rad = 0
        m = np.abs(m)
        for k in range((n-m)/2+1):
            rad += (-1.0)**k*np.math.factorial(n-k) / (np.math.factorial(k)*np.math.factorial((n+m)/2.0-k)*np.math.factorial((n-m)/2.0-k)) *rho**(n-2.0*k)        
        return rad                


    def calc_zernike(self):
        print 'calculating Zernike'
        n = self.nSpinBox.value()
        m = self.mSpinBox.value()
        self.zernike_multiplier = self.ZernikeMultiplierSpinBox.value()
        brushes = []
        zernike_component = []
        for i,electrode in enumerate(self.electrodeCoordinates):
            rho = np.sqrt(electrode[0]**2 + electrode[1]**2)
            if rho > 3:
                zernike_component.append(0)
                brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))   
                currentSB = self.slidersLayout.itemAt(2*i).widget()
                currentSB.setValue(self.voltages[i]) 
            elif m>=0:
                phi = np.arctan2(electrode[1],electrode[0])
                zernike_component.append(self.radial(n,m,rho) * np.cos(phi*m))
                brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))   
                currentSB = self.slidersLayout.itemAt(2*i).widget()
                currentSB.setValue(self.voltages[i]) 
            else:
                phi = np.arctan2(electrode[1],electrode[0])
                zernike_component.append(self.radial(n,m,rho) * np.sin(phi*m)) 
                brushes.append(QtGui.QColor(255, self.voltages[i]/self.maxVoltage*255, 0, 220))   
                currentSB = self.slidersLayout.itemAt(2*i).widget()
                currentSB.setValue(self.voltages[i])
        np.array(zernike_component)
        self.voltages = self.maxVoltage/2.0 + zernike_component * self.zernike_multiplier
        print self.voltages         

    #===========================================================================
    # def worker_calc_zernike(self):
    #     t1 = threading.Thread(name='calc_zernike', target=self.calc_zernike)
    #     t1.start()
    #     #t2 = threading.Thread(name='2d_interp', target=self.calc_zernike)
    #===========================================================================

        
 
    def myLayout(self):        
        self.layout = QtGui.QHBoxLayout(self) #the whole window, main layout
        self.leftLayout = QtGui.QVBoxLayout()
        self.middleLayout = QtGui.QVBoxLayout()
        self.rightLayout = QtGui.QVBoxLayout()
        self.controlsLayout = QtGui.QGridLayout()
        self.slidersLayout = QtGui.QGridLayout()
        self.electrodesLayout = QtGui.QGridLayout()
        self.cameraLayout = QtGui.QVBoxLayout()
        self.outlinesLayout = QtGui.QGridLayout()
        
        self.layout.addLayout(self.leftLayout)
        self.layout.addLayout(self.middleLayout)
        self.layout.addLayout(self.rightLayout)
        self.leftLayout.addLayout(self.slidersLayout)
        self.leftLayout.addLayout(self.controlsLayout)
        self.middleLayout.addLayout(self.electrodesLayout)
        self.middleLayout.addLayout(self.cameraLayout)
        self.middleLayout.addLayout(self.outlinesLayout)
                
        
#        self.plotWidget1 = pq.PlotWidget()
        self.plotWidget1 = pq.GraphicsLayoutWidget()
        self.plotWidget1.setAspectLocked(1)
        self.plotWidget1.setVisible(True)
        self.plotWidget1.setMaximumSize(350,350)
        self.plot11 = self.plotWidget1.addPlot()
        self.electrodesLayout.addWidget(self.plotWidget1)
        
        self.plotWidget2 = pq.PlotWidget()
        self.plotWidget2.setMaximumSize(350,200)
        self.plot21 = self.plotWidget2.plot()
        self.plot22 = self.plotWidget2.plot()
        #self.plotWidget2.setAspectLocked(1)  
        self.plot21.setData(np.sum(self.image,axis=0),pen='w',name='white plot')
        self.plot22.setData(np.sum(self.image,axis=1),pen='r',name='red plot')
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
        self.outlinesLayout.addWidget(self.plotWidget2)

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
        self.controlsLayout.addWidget(self.currentElectrodeSpinBox)
        self.currentElectrodeSpinBox.valueChanged.connect(self.updateVoltage)
        
        self.resetAllVoltagesButton = QtGui.QPushButton('Reset all')
        self.resetAllVoltagesButton.clicked.connect(self.resetAll)
        self.controlsLayout.addWidget(self.resetAllVoltagesButton)
        
        self.setToMaxButton = QtGui.QPushButton('Set to max')
        self.setToMaxButton.clicked.connect(self.setToMaxVoltage)
        self.controlsLayout.addWidget(self.setToMaxButton)
                    
        self.saveImageButton = QtGui.QPushButton('Save image')
        self.saveImageButton.clicked.connect(self.saveImage)
        self.controlsLayout.addWidget(self.saveImageButton)
        
        self.saveAllButton = QtGui.QPushButton('Save all images')
        self.saveAllButton.clicked.connect(self.saveImages)
        self.controlsLayout.addWidget(self.saveAllButton)
        
        # control widgets
        self.tiltLayout = QtGui.QGridLayout()
        self.controlsLayout.addLayout(self.tiltLayout,5,0)   
        self.xTiltSpinBox = QtGui.QDoubleSpinBox()
        self.xTiltSpinBox.setMaximum(120)
        self.xTiltSpinBox.setValue(20)
        self.xTiltSpinBox.valueChanged.connect(self.xTiltCalc)
        self.xTiltLabel = QtGui.QLabel('x tilt')
        self.tiltLayout.addWidget(self.xTiltSpinBox,0,1)
        self.tiltLayout.addWidget(self.xTiltLabel,0,0)
        
        self.zernikeLayout = QtGui.QGridLayout()
        self.controlsLayout.addLayout(self.zernikeLayout,6,0)
        self.nSpinBox = QtGui.QSpinBox()
        self.nSpinBox.valueChanged.connect(self.calc_zernike)
        self.nLabel = QtGui.QLabel('Zernike n')        
        self.mSpinBox = QtGui.QSpinBox()
        self.mSpinBox.valueChanged.connect(self.calc_zernike)
        self.mLabel = QtGui.QLabel('Zernike m')
        self.zernikeLayout.addWidget(self.nSpinBox,0,1)
        self.zernikeLayout.addWidget(self.nLabel,0,0)     
        self.zernikeLayout.addWidget(self.mSpinBox,1,1)
        self.zernikeLayout.addWidget(self.mLabel,1,0)
        self.ZernikeMultiplierSpinBox = QtGui.QDoubleSpinBox()
        self.ZernikeMultiplierSpinBox.setValue(10.0)
        self.zernikeLayout.addWidget(self.ZernikeMultiplierSpinBox,2,0)

        
            
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
    