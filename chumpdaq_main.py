#!/usr/bin/env python3

from dashdisplay import ChumpDashApp
from kivy.clock import Clock
from kivy.core.window import Window

# Setup the window position
Window.size = (1020,700)
Window.top = 0
Window.left = 0

# Create the "app" and run
ChumpDashApp().run()
