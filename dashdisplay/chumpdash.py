#!/usr/bin/env python3

from kivy.app import App
from kivy.core.window import Window
from kivy.uix.gridlayout import GridLayout
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.properties import ObjectProperty, NumericProperty, StringProperty
from threading import Thread
from .sensors import monitorsensors
from .sensors import monitor9dof
from .gps_logger import monitorgps
from kivy.clock import Clock
from time import time
import os

class MainDashScreen(Widget):
    # Kivy properties that need to be updated by Clock or other threads
    watert = NumericProperty(200)
    oilt = NumericProperty(200)
    oilp = NumericProperty(50)
    seat_time = NumericProperty(0)
    start_time = NumericProperty(-1)
    log_folder = StringProperty("/media/chump_thumb/chump_logs")
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_once(self.initdatalogger, 0.0)
        Clock.schedule_once(self.seattimecallback, 0.1)
        Clock.schedule_interval(self.seattimecallback, 10.0)

    def set_watert(self, value, *largs):
        self.watert = value
    def set_oilt(self, value, *largs):
        self.oilt = value
    def set_oilp(self, value, *largs):
        self.oilp = value

    def seattimecallback(self, dt):
        current_time = time()
        if self.start_time < 0:
            self.start_time = current_time
            Clock.schedule_interval(self.datalogger, 0.5)
        else:
            self.seat_time = current_time - self.start_time
    def initdatalogger(self, dt):
        old_dirs = sorted(next(os.walk(self.log_folder))[1], key=lambda x: int(x))
        if len(old_dirs) == 0:
            new_folder = "0000"
        else:
            new_folder = "{:04d}".format(int(old_dirs[-1])+1)
        self.log_folder = os.path.join(self.log_folder,new_folder)
        os.makedirs(self.log_folder)
        with open(os.path.join(self.log_folder,"data.log"), 'a') as f:
            f.write('seat_time water_temp oil_temp oil_pres\n')
    def datalogger(self, dt):
        with open(os.path.join(self.log_folder,"data.log"), 'a') as f:
            f.write('{:.2f} {} {} {}\n'.format(time()-self.start_time,
                                           self.watert,
                                           self.oilt,
                                           self.oilp,
                                           ))

class ChumpDashApp(App):
    def build(self):
        self.MDS = MainDashScreen()
        return self.MDS
    def on_start(self, *args):
        print('Starting sensor thread.')
        Thread(target=monitorsensors, args=(self.MDS,)).start()
        print('Starting gps thread.')
        Thread(target=monitorgps, args=(self.MDS,)).start()
        print('Starting imu thread.')
        Thread(target=monitor9dof, args=(self.MDS,)).start()

if __name__ == '__main__':
    # Setup the window position
    Window.size = (1020,700)
    Window.top = 0
    Window.left = 0
    ChumpDashApp().run()
