from piplates.DAQCplate import getADC,toggleDOUTbit
from math import log
from time import time
from time import sleep
from kivy.clock import Clock
from functools import partial
from qwiic_icm20948 import QwiicIcm20948 as q9dof
import os
import smbus
import board
import adafruit_vl53l1x


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
            try:
                r2 = self.resistor1*va/(vcc-va)+self.rtrim
            except:
                r2 = self.resistor1
            try:
                val = log(r2)
            except:
                val = 0.0
            try:
                val = 1.0/(self.curvefit[0]
                           +val*(self.curvefit[1]
                           +self.curvefit[2]*val*val))
            except:
                val = 273.0
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

def monitori2csensors(dash):
    last_read = 0
    init_check_period = 1.0
    main_log_start = False
    sensor_period = 0.25
    left_bus = 0
    right_bus = 1
    temp_add = 0x10
    left_height_offset = 0.0
    right_height_offset = 0.0

    while not main_log_start:
        curtime = time()
        if curtime-last_read < init_check_period:
            continue
        last_read = curtime
        if dash.log_folder[-1] == "s":
            continue
        main_log_start = True

    i2c_log_file = os.path.join(dash.log_folder, 'i2c.log')
    i2c_debug_file = os.path.join(dash.log_folder, 'i2c_debug.log')

    imu = q9dof()
    imufound = True
    if not imu.connected:
        with open(i2c_debug_file, 'a') as f:
            f.write('imu not detected\n')
        imufound = False

    if imufound:
        for i in range(20):
            try:
                imu.begin()
            except OSError:
                if i==19:
                    with open(i2c_debug_file, 'a') as f:
                        f.write('imu not started\n')
                continue
            break

    #setup i2c comm for simple sensors
    bus = smbus.SMBus(1)
    def read_temp(add):
        try:
            bus.write_byte(add,0x80)
        except OSError:
            return (0.0, 0.0)
        data = []
        for i in range(6):
            data.append(bus.read_byte(add))
        amb = (data[2]*2**16 + data[1]*2**8 + data[0])/200.0
        obj = (data[5]*2**16 + data[4]*2**8 + data[3])/200.0
        return ((amb+273.15)*9/5-459.67, (obj+273.15)*9/5-459.67)

    #function to toggle the bus for i2c multiplexers
    def select_i2c_bus(multibus):
        try:
            bus.write_byte(0x70,1<<multibus)
        except OSError:
            print('I2C Multiplexers not found.')

    #setup i2c for distance sensors
    i2c = board.I2C()
    select_i2c_bus(left_bus)
    try:
        vl53_left = adafruit_vl53l1x.VL53L1X(i2c)
    except:
        vl53_left = False
    if vl53_left:
        vl53_left.start_ranging()
    select_i2c_bus(right_bus)
    try:
        vl53_right = adafruit_vl53l1x.VL53L1X(i2c)
    except:
        vl53_right = False
    if vl53_right:
        vl53_right.start_ranging()

    temps_left = (0,0)
    temps_right = (0,0)
    ax = 0
    ay = 0
    az = 0
    gx = 0
    gy = 0
    gz = 0
    mx = 0
    my = 0
    mz = 0
    height_left = 0
    height_right = 0

    while True:
        if dash.start_time < 0:
            continue
        curtime = time()
        if curtime-last_read < sensor_period:
            continue
        last_read = curtime

        #read internal 9dof sensor
        if imu.dataReady():
            imu.getAgmt()
            ax = imu.axRaw
            ay = imu.ayRaw
            az = imu.azRaw
            gx = imu.gxRaw
            gy = imu.gyRaw
            gz = imu.gzRaw
            mx = imu.mxRaw
            my = imu.myRaw
            mz = imu.mzRaw

        #read left and right corner sensors
        select_i2c_bus(left_bus)
        temps_left = read_temp(temp_add)
        try:
            if vl53_left.data_ready:
                height_left = vl53_left.distance
                vl53_left.clear_interrupt()
        except OSError:
            height_left = 0.0
        if not height_left:
            height_left = 0.0
        height_left = height_left #* 10.0

        select_i2c_bus(right_bus)
        temps_right = read_temp(temp_add)
        try:
            if vl53_right.data_ready:
                height_right = vl53_right.distance
                vl53_right.clear_interrupt()
        except OSError:
            height_right = 0.0
        if not height_right:
            height_right = 0.0
        height_right = height_right #* 10.0

        # write tire temps back to dashboard
        val = '{:5.1f} F'.format(temps_left[1])
        Clock.schedule_once(partial(dash.set_tire_fl, val),0)
        val = '{:5.1f} F'.format(temps_right[1])
        Clock.schedule_once(partial(dash.set_tire_fr, val),0)
        with open(i2c_log_file, 'a') as f:
            f.write('{:.2f} {:06d} {:06d} {:06d} {:06d} {:06d} {:06d} {:06d} {:06d} {:06d} {:6.1f} {:6.1f} {:6.1f} {:6.1f} {:4.1f} {:4.1f}\n'.format(
                curtime-dash.start_time,
                ax,
                ay,
                az,
                gx,
                gy,
                gz,
                mx,
                my,
                mz,
                temps_left[0],
                temps_left[1],
                temps_right[0],
                temps_right[1],
                height_left*10.0+left_height_offset,
                height_right*10.0+right_height_offset,))
