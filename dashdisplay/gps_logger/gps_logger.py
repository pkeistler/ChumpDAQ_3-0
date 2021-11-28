from time import time
from kivy.clock import Clock
from functools import partial
from serial import Serial
from ublox_gps import UbloxGps
import os

def monitorgps(dash):

    last_read = 0
    init_check_period = 1.0
    main_log_start = False

    port = Serial('/dev/serial0', 38400, timeout=1)
    gps = UbloxGps(port)

    while True:
        if not main_log_start:
            curtime = time()
            if curtime-last_read < init_check_period:
                continue
            last_read = curtime
            if dash.log_folder[-1] == "s":
                continue
            main_log_start = True
            gps_log_file = os.path.join(dash.log_folder,'gps_nmea.log')
        with open(gps_log_file, 'a') as f:
            f.write(gps.stream_nmea())
