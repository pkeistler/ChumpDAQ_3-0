from time import time
from kivy.clock import Clock
from functools import partial
from serial import Serial
from pyubx2.ubxreader import UBXReader
import os
from datetime import datetime
from datetime import timedelta
import numpy as np

class lap_tracker(object):
    def __init__(
            self,
            gpsstream,
            log_folder):
        self.gpsstream = gpsstream
        self.log_folder = log_folder
        self.gps_log_file = os.path.join(self.log_folder,'gps_ubx.log')
        self.starttime = None
        self.tzd = timedelta(hours=-4)
        with open(self.gps_log_file, 'a') as f:
            f.write('date time longitude latitude speed\n')

    def update(self):
        (raw, self.pdata) = self.gpsstream.read()
        if self.pdata.gnssFixOk != 1:
            return
        self.gtime = datetime(
                self.pdata.year,
                self.pdata.month,
                self.pdata.day,
                self.pdata.hour,
                self.pdata.min,
                self.pdata.second,
                max(0,min(999999,int(self.pdata.nano*1e-3))))
        self.gtime += self.tzd
        if self.starttime is None:
            self.starttime = self.gtime
        with open(self.gps_log_file, 'a') as f:
            f.write('{} {} {} {}\n'.format(
                self.gtime,
                self.pdata.lon,
                self.pdata.lat,
                self.pdata.gSpeed*0.00223694))

def monitorgps(dash):

    last_read = 0
    init_check_period = 1.0
    main_log_start = False

    port = Serial('/dev/ttyS0', 115200, timeout=1)
    gps = UBXReader(port)

    starttime = None
    while True:
        if not main_log_start:
            curtime = time()
            if curtime-last_read < init_check_period:
                continue
            last_read = curtime
            if dash.log_folder[-1] == "s":
                continue
            main_log_start = True

            lapper = lap_tracker(gps,dash.log_folder)

        # MAIN LOOP
        lapper.update()
