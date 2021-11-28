from piplates.DAQCplate import getADC,toggleDOUTbit
from math import log
from time import time
from time import sleep
from kivy.clock import Clock
from functools import partial
from qwiic_icm20948 import QwiicIcm20948 as q9dof
import os


class Sensor(object):
    def __init__(self,
            pin=10,
            resistor1=500.0,
            resistor2=500.0,
            rtrim=0.0,
            fittype='rpolylog', #or 'vlinear'
            curvefit=[1,0,0],
            units='F'):
        self.pin = pin
        self.resistor1 = resistor1
        self.resistor2 = resistor2
        self.rtrim = rtrim
        self.fittype = fittype
        self.curvefit = curvefit
        self.units = units
        self.display = None
    
    def getVal(self):
        va = getADC(0,self.pin)
        #toggleDOUTbit(0,self.pin)
        if self.fittype == 'rpolylog':
            vcc = getADC(0,8)
            r2 = self.resistor1*va/(vcc-va)+self.rtrim
            val = log(r2)
            val = 1.0/(self.curvefit[0]
                       +val*(self.curvefit[1]
                       +self.curvefit[2]*val*val))
            val = val*9.0/5.0 - 459.67
            return val
        elif self.fittype == 'vlinear':
            vactual = va*(self.resistor1+self.resistor2)/self.resistor2
            val = self.curvefit[0]+self.curvefit[1]*vactual
            return val

def setupSensors():
    sensors={}
    waterTempSensor = Sensor(
                        pin=0,
                        resistor1=328.5, #333
                        curvefit=[0.00185492,
                                  0.000216673,
                                  1.62792e-7],
                        )
    sensors['waterT'] = waterTempSensor
    oilTempSensor = Sensor(
                      pin=1,
                      resistor1=328.5, #333
                      curvefit=[0.00175353,
                                0.000196018,
                                3.10344e-7],
                      )
    sensors['oilT'] = oilTempSensor
    oilPresSensor = Sensor(
                      pin=2,
                      resistor1=0.1,
                      resistor2=10000.0,
                      fittype='vlinear',
                      curvefit=[-12.5,25],
                      units='PSI'
                      )
    sensors['oilP'] = oilPresSensor

    return sensors

def monitorsensors(dash):
    sensors = setupSensors()

    last_read = 0
    period = 0.25 #seconds
    while True:
       curtime = time()
       if curtime-last_read < period:
           continue 
       last_read = curtime
       for name,sensor in sensors.items():
           val = sensor.getVal()
           if name == 'waterT':
               val = round(val,0)
               Clock.schedule_once(partial(dash.set_watert, val),0)
           elif name == 'oilT':
               val = round(val,0)
               Clock.schedule_once(partial(dash.set_oilt, val),0)
           elif name == 'oilP':
               val = round(val,1)
               Clock.schedule_once(partial(dash.set_oilp, val),0)

def monitor9dof(dash):
    last_read = 0
    init_check_period = 1.0
    main_log_start = False
    sensor_period = 0.25

    while not main_log_start:
        curtime = time()
        if curtime-last_read < init_check_period:
            continue
        last_read = curtime
        if dash.log_folder[-1] == "s":
            continue
        main_log_start = True

    imu_log_file = os.path.join(dash.log_folder, 'imu.log')

    imu = q9dof()
    if not imu.connected:
        print('IMU not connected')
        return

    imu.begin()

    while True:
        if dash.start_time < 0:
            continue
        curtime = time()
        if curtime-last_read < sensor_period:
            continue
        last_read = curtime
        if imu.dataReady():
            imu.getAgmt()
            with open(imu_log_file, 'a') as f:
                f.write('{:.2f} {:06d} {:06d} {:06d} {:06d} {:06d} {:06d} {:06d} {:06d} {:06d}\n'.format(
                    curtime-dash.start_time,
                    imu.axRaw,
                    imu.ayRaw,
                    imu.azRaw,
                    imu.gxRaw,
                    imu.gyRaw,
                    imu.gzRaw,
                    imu.mxRaw,
                    imu.myRaw,
                    imu.mzRaw))
