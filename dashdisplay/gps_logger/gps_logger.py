from time import time
from kivy.clock import Clock
from functools import partial
from serial import Serial
from pyubx2.ubxreader import UBXReader
import os
from datetime import datetime
from datetime import timedelta
import numpy as np

def monitorgps(dash):

    last_read = 0
    init_check_period = 1.0
    main_log_start = False

    port = Serial('/dev/ttyS0', 115200, timeout=1)
    gps = UBXReader(port)

    timezonedelta = timedelta(hours=-4)
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
            gps_log_file = os.path.join(dash.log_folder,'gps_ubx.log')
            with open(gps_log_file, 'a') as f:
                f.write('datetime longitude latitude speed\n')
        (raw_data, pdata) = gps.read()
        with open(gps_log_file, 'a') as f:
            if pdata.gnssFixOk != 1:
                continue
            gtime = datetime(
                    pdata.year,
                    pdata.month,
                    pdata.day,
                    pdata.hour,
                    pdata.min,
                    pdata.second,
                    max(0,min(999999,int(pdata.nano*1e-3))))
            gtime += timezonedelta
            f.write('{} {} {} {}\n'.format(
                #pdata.hour,pdata.min,pdata.second,max(0,min(999999,int(pdata.nano*1e-3))),
                gtime,
                pdata.lon,
                pdata.lat,
                pdata.gSpeed*0.00223694))
