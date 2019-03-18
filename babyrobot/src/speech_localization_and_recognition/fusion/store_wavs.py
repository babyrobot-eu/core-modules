#!/usr/bin/env python
__author__ = 'irodoma'

import datetime
import sys
import re
import glob
import os


log = '/home/mobot/irodoma/mobot-visualisation/patients.txt'
wavdir = '/home/nick/catkin_ws/src/cvsp_audio_gesture/src/audio_rec/kws/recorder/wavs'


# read sessions
sessions = {}
f = open(log,'r')
i = 0 
for ln in f:
    ln = ln.rstrip()
    m = re.search('{patient:(.*),time:(.*)}',ln)
    sessions[m.group(1)] = m.group(2)

#print sessions

# read wavs
files = glob.glob(wavdir + "/*.wav")
for f in files:
    f = os.path.basename(f)
    filename, file_extension = os.path.splitext(f)
    tmp = filename.split('_')
    date = tmp[3]
    time = tmp[4]


print ('20160421_091623' > '20160421_091623')
