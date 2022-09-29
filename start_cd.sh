#!/bin/bash

#sleep 10h
cd /home/pi/chumpdaq_v3
python3 ./chumpdaq_main.py &> /media/chump_thumb/daq.log
