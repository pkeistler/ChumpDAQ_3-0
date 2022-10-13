#!/usr/bin/env python3

from kivy.app import App
from kivy.core.window import Window
from kivy.uix.gridlayout import GridLayout
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.uix.image import Image
from kivy.properties import (
        ObjectProperty,
        NumericProperty,
        StringProperty,
        #ListProperty,
        )
##from kivy_garden.graph import Graph, LinePlot
#from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
#import matplotlib.pyplot as plt
from threading import Thread
from .sensors import monitorsensors
from .sensors import monitori2csensors
from .gps_logger import monitorgps
from kivy.clock import Clock
from time import time
import os

x = [1,2,3,4,5]
y = [5,4,3,2,1]

#plt.plot(x,y)

class MainDashScreen(Widget):
    # Kivy properties that need to be updated by Clock or other threads
    watert = NumericProperty(200)
    oilt = NumericProperty(200)
    oilp = NumericProperty(50)
    seat_time = NumericProperty(0)
    start_time = NumericProperty(-1)
    cur_speed = NumericProperty(0)
    delta_time = StringProperty('+0.00')
    delta_velocity = StringProperty('+0.00')
    last_lap = StringProperty('0: 9.59')
    best_lap = StringProperty('0: 9.59')
    log_folder = StringProperty("/media/chump_thumb/chump_logs")
    current_track = StringProperty("Finding track.")
    lap_image_init = False
#    current_lap_xtrace = ListProperty([0.0])
#    current_lap_ytrace = ListProperty([0.0])
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_once(self.initdatalogger, 0.0)
        Clock.schedule_once(self.seattimecallback, 0.1)
        Clock.schedule_interval(self.seattimecallback, 10.0)

##        self.graph = Graph()
##        self.ids.plot_area.add_widget(self.graph)
#    def set_track_xtrace(self, value, *largs):
#        self.current_lap_xtrace = value
#    def set_track_ytrace(self, value, *largs):
#        self.current_lap_ytrace = value
#        self.ids.plot_area.clear_widgets()
#        plt.cla()
#        plt.plot(self.current_lap_xtrace,self.current_lap_ytrace)
#        self.ids.plot_area.add_widget(FigureCanvasKivyAgg(plt.gcf()))

    def set_watert(self, value, *largs):
        self.watert = value
    def set_oilt(self, value, *largs):
        self.oilt = value
    def set_oilp(self, value, *largs):
        self.oilp = value
    def set_track(self, value, *largs):
        self.current_track = value
    def set_deltat(self, value, *largs):
        self.delta_time = value
    def set_deltav(self, value, *largs):
        self.delta_velocity = value
    def set_lastlap(self, value, *largs):
        self.last_lap = value
    def set_bestlap(self, value, *largs):
        self.best_lap = value
    def update_lap_image(self, *largs):
        if not self.lap_image_init:
            self.image = Image(source='{}/lap_plot.png'.format(self.log_folder))
            self.ids.plot_area.add_widget(self.image)
            self.lap_image_init = True
        else:
            self.image.reload()
    def set_speed(self, value, *largs):
        self.cur_speed = value

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
        Thread(target=monitori2csensors, args=(self.MDS,)).start()

if __name__ == '__main__':
    # Setup the window position
    Window.size = (1020,700)
    Window.top = 0
    Window.left = 0
    ChumpDashApp().run()
