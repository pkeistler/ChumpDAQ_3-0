from time import time
from kivy.clock import Clock
from functools import partial
from serial import Serial
from pyubx2.ubxreader import UBXReader
import os
from datetime import datetime
from datetime import timedelta
import numpy as np
from scipy.spatial import KDTree
import subprocess
#import matplotlib.pyplot as plt
#from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
##from kivy_garden.graph import MeshLinePlot

class lap_tracker(object):
    def __init__(
            self,
            gpsstream,
            dash_obj,
            gpsformat='ubx'):
        self.gpsstream = gpsstream
        self.log_folder = dash_obj.log_folder
        log_folder_short = self.log_folder.split('/')[-1]
        self.lap_command = [
                "/home/pi/chumpdaq_v3/dashdisplay/gps_logger/plot_lap_deltav.sh",
                log_folder_short
                ]
        self.gpsformat = gpsformat
        self.dash = dash_obj
        if gpsformat == 'ubx':
            gfile = 'gps_ubx.log'
        else:
            gfile = 'gps_nmea.log'
        self.speed_samples = 0
        self.speed_running_avg = 0.0
        self.gps_log_file = os.path.join(self.log_folder,gfile)
        self.starttime = None
        self.last_delta_update = None
        self.delta_update_period = timedelta(seconds=0.5)
        self.last_plot_update = None
        self.plot_update_period = timedelta(seconds=1.0)
        self.tzd = timedelta(hours=-4)
        with open(self.gps_log_file, 'a') as f:
            f.write('date time longitude latitude speed\n')
        self.stream_valid = True
        self.feet_per_lat = None
        self.trackname = None
        self.tracksfline = None
        self.sfvecnorm = None
        self.sfradius = 0.0
        self.currentlaptrace = []
        self.bestKDT = None
        self.bestTIM = None
        self.bestSPD = None
        self.deltav_trace_file = os.path.join(self.log_folder,'deltav_trace.dat')
        with open(self.deltav_trace_file, 'a') as f:
            f.write('time xfeet yfeet deltav deltat\n')
        self.currentlapnum = 0
        self.currentlapstart = None
        self.laphistory = []
        with open(os.path.join(self.log_folder,'all_laps.dat'), 'a') as f:
            f.write('lap_number lap_time(sec)\n')
        self.bestlaptime = 9999.0
        self.bestlapnum = 0
        self.debug = False
        ##self.plot = MeshLinePlot()
        ##self.dash.graph.add_plot(self.plot)

    ##def plot_track(self):
    ##    xy = [x[1] for x in self.currentlaptrace]
    ##    self.plot.points = xy
        
    def debug_write(self,statement,force=False):
        if self.debug or force:
            with open(os.path.join(self.log_folder,'debug.log'), 'a') as f:
                f.write('{}\n'.format(statement))

    def set_feet_per_degree(self):
        if self.lat is not None and self.lon is not None:
            self.feet_per_lat = 364000.0
            self.feet_per_lon = self.feet_per_lat*np.cos(self.lat*np.pi/180.0)

    def find_track(self):
        track_folder = '/home/pi/chumpdaq_v3/tracks'
        available_tracks = next(os.walk(track_folder))[2]
        alltracks = {}
        close_dist = 9999.0 #miles
        close_track = None
        for track in available_tracks:
            with open(os.path.join(track_folder,track), 'r') as f:
                alltracks[track] = {}
                alltracks[track]['name'] = track.split('.')[0]
                alltracks[track]['sfline'] = [float(x) for x in f.readline().split()]
                dlat = self.lat - alltracks[track]['sfline'][0]
                dlon = self.lon - alltracks[track]['sfline'][1]
                dlat = dlat * self.feet_per_lat/5280.0 #miles
                dlon = dlon * self.feet_per_lon/5820.0 #miles
                dist = np.sqrt(dlat**2 + dlon**2)
                if dist < close_dist:
                    close_dist = dist
                    close_track = track
        if close_dist < 50.0:
            #track found
            self.trackname = alltracks[close_track]['name']
            self.tracksfline = alltracks[close_track]['sfline']
            #with open(self.gps_log_file, 'a') as f:
            #    f.write('Track {}, distance {}\n'.format(self.trackname,close_dist))
            Clock.schedule_once(partial(self.dash.set_track, self.trackname),0)
            sfl = self.tracksfline
            sfvector = np.array([(sfl[3]-sfl[1])*self.feet_per_lon, (sfl[2]-sfl[0])*self.feet_per_lat])
            self.sfradius = np.linalg.norm(sfvector)
            self.sfvecnorm = sfvector/self.sfradius
            return True
        else:
            self.debug_write('No track found within 50 miles.',force=True)
            return False
        
    # this is the main public method
    def update(self):
        if not self.stream_valid:
            return
        if self.gpsformat == 'ubx':
            succ = self.process_ubx()
        else:
            succ = self.process_nmea()
        if succ:
            succ2 = True
            if self.feet_per_lat is None:
                self.set_feet_per_degree()
            if self.trackname is None:
                succ2 = self.find_track()
                self.debug_write('track found: {}'.format(self.trackname))
            if self.currentlapstart is None:
                self.currentlapstart = self.gtime
            if succ2:
                self.add_to_lap()

    def process_ubx(self):
        (raw, pdata) = self.gpsstream.read()
        if pdata.gnssFixOk != 1:
            return
        self.gtime = datetime(
                pdata.year,
                pdata.month,
                pdata.day,
                pdata.hour,
                pdata.min,
                pdata.second,
                max(0,min(999999,int(pdata.nano*1e-3))))
        self.gtime += self.tzd
        if self.starttime is None:
            self.starttime = self.gtime
        with open(self.gps_log_file, 'a') as f:
            f.write('{} {} {} {}\n'.format(
                self.gtime,
                pdata.lon,
                pdata.lat,
                pdata.gSpeed*0.00223694))
        self.lon = pdata.lon
        self.lat = pdata.lat
        self.speed = pdata.gSpeed*0.00223695
        return True
            
    def process_nmea(self):
        rmcfound = False
        while not rmcfound:
            try:
                ndata = self.gpsstream.readline()
            except UnicodeDecodeError:
                self.debug_write('gps nmea file complete',force=True)
                self.stream_valid = False
                return False
            if 'GNRMC' in ndata:
                rmcfound = True
        fixfields = ndata.split(',')

        day = int(fixfields[9][0:2])
        month = int(fixfields[9][2:4])
        year = 2000+int(fixfields[9][4:])
        hour = int(fixfields[1][0:2])
        minute = int(fixfields[1][2:4])
        second = int(fixfields[1][4:6])
        microsecond = int(float('.{}'.format(fixfields[1].split('.')[-1]))*1000000)
        gtime = datetime(year,month,day,hour,minute,second,microsecond)
        gtime += self.tzd
        if self.starttime is None:
            self.starttime = gtime
        #if lapstart is None:
        #  lapstart = time - timedelta(seconds=100)
      
        #timed = time - lapstart
      
        gSpeed = float(fixfields[7])*1.15078 #mph
      
        xpos = float(fixfields[5][0:3]) + float(fixfields[5][3:])/60.0
        if fixfields[6] == 'W':
          xpos *= -1.0
        ypos = float(fixfields[3][0:2]) + float(fixfields[3][2:])/60.0
        if fixfields[4] == 'S':
          ypos *= -1.0
      
        with open(self.gps_log_file, 'a') as f:
            f.write('{} {} {} {}\n'.format(
                gtime,
                xpos,
                ypos,
                gSpeed))
        self.gtime = gtime
        self.lon = xpos
        self.lat = ypos
        self.speed = gSpeed
        return True
        #xfeet = (xpos-sfline[1])*feet_per_lon
        #yfeet = (ypos-sfline[0])*feet_per_lat
      
        #if fixindex < 599:
        #  laps[thislap][fixindex,:] = [timed.total_seconds(), speed, xfeet, yfeet]
        #  fixindex += 1
      
        #print(xfeet,yfeet)
      #  if np.linalg.norm(laps[thislap][fixindex-1,2:]) < sfradius and fixindex > 1:
      #    fix1 = laps[thislap][fixindex-2,:]
      #    fix2 = laps[thislap][fixindex-1,:]
      #    v1d = np.dot(sfvecnorm,fix1[2:])
      #    v2d = np.dot(sfvecnorm,fix2[2:])
      #    #print(fix1[0],v1d,v2d)
      #    if v1d < 0 and v2d >= 0:
      #      w1 = v2d/(v2d-v1d)
      #      w2 = -v1d/(v2d-v1d)
      #      laptime = w1*fix1[0] + w2*fix2[0]
      #      lapstart = time-timedelta(seconds=fix2[0]-laptime)
      #      print('lap! ',thislap,laptime)
      #      print(fastlaptime)
      #      if laptime < fastlaptime and thislap > 0:
      #        fastlaptime = laptime
      #        fastlap = thislap
      #      thislap += 1
      #    
      ##clean all laps
      #for lapnum in laps.keys():
      #  laps[lapnum] = clean_lap(laps[lapnum])
      #
      #for lapnum in range(thislap):
      #  with open('{}/lap{:04d}.dat'.format(folderout,lapnum), 'w') as f:
      #    for fix in laps[lapnum]:
      #      f.write('{} {} {} {}\n'.format(*fix))
      #print('Best lap: ',fastlap)
      #print('Best lap time: ',fastlaptime)

    def update_deltas(self,sample,time,speed):
        if self.speed_samples > 0:
            avg_spd = self.speed_running_avg/self.speed_samples
            Clock.schedule_once(partial(self.dash.set_speed, avg_spd),0)
            self.speed_samples = 0
            self.speed_running_avg = 0
        if self.bestKDT is None:
            return
        dists, inds = self.bestKDT.query(sample, k=2)
        dists = np.flip(dists/np.sum(dists))
        deltat = max(-20.0,min(20.0,time - np.sum(dists*self.bestTIM[inds])))
        deltav = max(-20.0,min(20.0,speed - np.sum(dists*self.bestSPD[inds])))
        with open(self.deltav_trace_file, 'a') as f:
            f.write('{} {} {} {} {}\n'.format(
                time,
                *sample,
                deltav,
                deltat
                ))
        if self.gtime - self.last_delta_update >= self.delta_update_period:
            #update the plot using subprocess
            plot_lap = subprocess.Popen(self.lap_command)
            plot_lap.wait()
            Clock.schedule_once(partial(self.dash.update_lap_image),0)
        if deltat < 0.0:
            sdeltat = '[color=#00FF00]{:5.2f}[/color]'.format(deltat)
        else:
            sdeltat = '[color=#FF0000]+{:5.2f}[/color]'.format(deltat)
        if deltav < 0.0:
            sdeltav = '[color=#FF0000]{:5.2f}[/color]'.format(deltav)
        else:
            sdeltav = '[color=#00FF00]+{:5.2f}[/color]'.format(deltav)
        Clock.schedule_once(partial(self.dash.set_deltat, sdeltat),0)
        Clock.schedule_once(partial(self.dash.set_deltav, sdeltav),0)

    def add_to_lap(self):
        #First convert to feet and check if we've crossed the start finish line
        xfeet = (self.lon - self.tracksfline[1])*self.feet_per_lon
        yfeet = (self.lat - self.tracksfline[0])*self.feet_per_lat
        self.debug_write('new point {} {}'.format(xfeet,yfeet))
        # update speed calcs
        self.speed_samples += 1
        self.speed_running_avg += self.speed
        current_radius = np.linalg.norm([xfeet,yfeet])
        #self.debug_write('radius: {} {}'.format(current_radius,self.sfradius))
        if current_radius < self.sfradius:
            if len(self.currentlaptrace) > 1:
                fix1 = self.currentlaptrace[-1][1]
                fix2 = np.array([xfeet,yfeet])
                v1d = np.dot(self.sfvecnorm,fix1)
                v2d = np.dot(self.sfvecnorm,fix2)
                self.debug_write('in radius: {} {}'.format(v1d,v2d))
                self.debug_write('           {} {}'.format(fix1,fix2))
                if v1d < 0 and v2d >= 0:
                    # Start finish line is crossed! Update stuff accordingly.
                    w1 = v2d/(v2d-v1d)
                    w2 = -v1d/(v2d-v1d)
                    time2d = self.gtime - self.currentlapstart
                    self.debug_write('calc lap time {} {} {} {}'.format(
                                       w1, w2, self.currentlaptrace[-1][0], time2d.total_seconds()))
                    laptime = w1*self.currentlaptrace[-1][0] + w2*time2d.total_seconds()
                    # finish up lap trace and write it out
                    self.currentlaptrace.append([
                                         laptime,
                                         np.array([fix1[0]*w1+fix2[0]*w2, fix1[1]*w1+fix2[1]*w2]),
                                         self.currentlaptrace[-1][2]*w1 + self.speed*w2
                                         ])
                    # make note of last point so it can be used as the first point of the next lap
                    nextlappoint = [0.0]+self.currentlaptrace[-1][1:]
                    self.debug_write('nextlapopint {}'.format(nextlappoint))
                    self.write_out_lap()
                    ##self.plot_track()
                    # Add this lap to the lap history
                    self.laphistory.append([self.currentlapnum,laptime])
                    with open(os.path.join(self.log_folder,'all_laps.dat'), 'a') as f:
                        f.write('{} {}\n'.format(
                                self.laphistory[-1][0],
                                self.laphistory[-1][1],
                                ))
                    llaptime = '{}:{:05.2f}'.format(
                            int(laptime/60.0),
                            laptime%60,
                            )
                    Clock.schedule_once(partial(self.dash.set_lastlap, llaptime),0)
                    blaptime = '{}:{:05.2f}'.format(
                            int(self.bestlaptime/60.0),
                            self.bestlaptime%60,
                            )
                    Clock.schedule_once(partial(self.dash.set_bestlap, blaptime),0)
                    if laptime < self.bestlaptime:
                        Clock.schedule_once(partial(self.dash.set_lastlap,
                                '[color=#00FF00]{}[/color]'.format(llaptime)),0)
                        Clock.schedule_once(partial(self.dash.set_bestlap,
                                '[color=#00FF00]{}[/color]'.format(llaptime)),0)
                        self.bestlaptime = laptime
                        self.bestlapnum = self.currentlapnum
                        # build k-d tree for sampling use later
                        self.bestKDT = KDTree(np.array([x[1] for x in self.currentlaptrace]))
                        self.bestTIM = np.array([x[0] for x in self.currentlaptrace])
                        self.bestSPD = np.array([x[2] for x in self.currentlaptrace])
                    # now increment lap
                    self.currentlapnum = self.currentlapnum + 1
                    self.currentlaptrace = []
                    self.currentlaptrace.append(nextlappoint)
                    # store the time when sf line was crossed (a litte bit before now)
                    self.currentlapstart = self.gtime-timedelta(seconds=time2d.total_seconds()-laptime)
                    return
        # Didn't cross start finish line, so we just add it to the trace.
        timed = self.gtime - self.currentlapstart
        self.currentlaptrace.append([
                             timed.total_seconds(),
                             np.array([xfeet,yfeet]),
                             self.speed
                             ])
        self.debug_write('current lap trace: {}'.format(self.currentlaptrace[-1]))
        # Update deltas if the period has passed
        if self.last_delta_update is None:
            self.last_delta_update = self.gtime
        if self.last_plot_update is None:
            self.last_plot_udpate = self.gtime
        if self.gtime - self.last_delta_update >= self.delta_update_period:
            self.update_deltas(np.array([xfeet,yfeet]),timed.total_seconds(),self.speed)
            self.last_delta_update = self.gtime
                    
    def write_out_lap(self):
        with open(os.path.join(self.log_folder,'lap_trace_{:04d}.dat'.format(self.currentlapnum)), 'w') as f:
            f.write('time(sec) xpos(ft) ypos(ft) speed(mph)\n')
            for point in self.currentlaptrace:
                f.write('{} {} {} {}\n'.format(
                        point[0],
                        point[1][0],
                        point[1][1],
                        point[2]
                        ))

def monitorgps(dash):

    last_read = 0
    init_check_period = 1.0
    main_log_start = False

    port = Serial('/dev/ttyS0', 115200, timeout=1)
    gps = UBXReader(port)

    #temporary nmea file read for testing
    nmeagps = open('/media/chump_thumb/chump_log_processing/road_atlanta_and_mid_ohio_2022/0074/gps_nmea.log', 'r')
    for i in range(500):
        nmeagps.readline()
    nmea_gps_period = 1.0

    while True:
        if not main_log_start:
            curtime = time()
            if curtime-last_read < init_check_period:
                continue
            last_read = curtime
            if dash.log_folder[-1] == "s":
                continue
            main_log_start = True

            #lapper = lap_tracker(gps,dash)
            lapper = lap_tracker(nmeagps, dash, gpsformat='nmea')

        # MAIN LOOP
        # make it wait 1 secon between updates
        curtime = time()
        if curtime-last_read < nmea_gps_period:
            continue
        last_read = curtime

        lapper.update()
