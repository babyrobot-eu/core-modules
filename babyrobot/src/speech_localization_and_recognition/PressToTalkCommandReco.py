#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import numpy as np
import sys
import os
import re
import os.path
import time
import wave
import subprocess
import codecs
import scipy.io.wavfile
from  kws.recorder.cvsp_recorder_on import *
from  kws.recorder.delay_and_sum_beamformer import *
import time
import threading
from collections import deque
from vad_functions import *
from std_msgs.msg import String
import random
import yaml
import rospy
#import atexit
#import signal

class PressToTalkCommandReco():
    def __init__(self, recorder):
        self.currdir = os.path.dirname(os.path.abspath(__file__))
        #self.currdir = '/home/nick/catkin_ws/src/cvsp_audio_gesture/src/audio_rec/kws/recorder'
        self.configYaml = self.currdir  + '/PressToTalkCommandReco.yaml'
        head, tail = os.path.split(self.currdir)
        self.basepath, tail = os.path.split(head)
        self.started_talking = False
        self.wav_dir = self.currdir + '/wavs'
        self.mfcc_dir = self.currdir + '/mfcc'
        self.rec_dir = self.currdir + '/rec'
        self.bin_dir = self.basepath + '/kws/bin/linux_x86_64'
        self.hcopy_cfg = self.basepath + '/kws/configs/hcopy_voxforge.cfg'
        self.models = self.basepath + '/kws/models/german/voxforge_german/hmmdefs'
        self.models_rate = 16000
        self.dictionary = self.basepath + '/kws/dict/mobot_demo_commands.dict'
        self.model_list = self.basepath + '/kws/models/german/voxforge_german/tiedlist'
        self.wav_file = ''
        # Vite params
        self.hvite_cfg = self.basepath + '/kws/configs/HVite.config'
        self.asr_wordnet = self.basepath + '/kws/lm/mobot_demo_german/mobot_demo_commands.wdnet'
        self.classes = self.basepath + '/kws/models/german/mobot_demo_global_mllr_adaptation_loo_2_adapt/classes_32'
        self.xforms = self.basepath + '/kws/models/german/mobot_demo_global_mllr_adaptation_loo_2_adapt/xforms'
        self.nhyps = 1
        self.ntokens = 1000
        # Framing parameters
        #self.asr_frame_dur = 2.5
        self.asr_frame_dur = 2.5
        self.asr_frame_shift_time = 0.6
        #self.asr_frame_shift_time = 0.6
        self.vad_frame_time = 0.5
        self.threads_die_now = False
        self.CR = recorder
        self.max_recent_rec_results = 3
        self.recent_rec_results = deque(self.max_recent_rec_results*' ', self.max_recent_rec_results)
        self.rec_output = ''
        self.msg = ''
        self.file_queue = deque()
        self.times_queue = deque()
        self.rec_output_changed = False
        self.read_output = threading.Event()
        self.read_output.set()
	self.use_beamforming = True
	
	# Increase robustness by requesting two continuous segments to give the same result
	# Makes sense when no vad is applied
        self.increased_robustness = True
	# Apply voice activity detection
	self.use_vad = False
        self.s_thread = None
	# enable press to talk
	self.ptt = False
	# recognition output parameters
	#self.output_enc = 'iso-8859-7'
	self.output_enc = 'utf8'
	# ROS topic to communicate with other recognizers (e.g. gesture recognition)
	#self.publisher = rospy.Publisher('gesture_segmentation', String)
	# enable MLLR adaptation
	self.adaptation= True
        self.clear()

    def exit_handler(self):
       print 'Bye Bye!!'
       self.end_recording()
       time.sleep(5)
       #sys.exit()

    def config(self):
       with open(self.configYaml, 'r') as stream:
          try:
             params = yaml.load(stream)
             params['segmentation'] = 'pressToTalk'
             if params['segmentation'] == 'slidingWindow':
	        self.use_vad = False
                self.increased_robustness = True
                self.ptt = False
             elif params['segmentation'] == 'vad':
	        self.use_vad = True
                self.increased_robustness = False
                self.ptt = False
             elif params['segmentation'] == 'pressToTalk':
	        self.use_vad = False
                self.increased_robustness = False
                self.ptt = True
	     else:
                self.use_vad = False
                self.increased_robustness = True
                self.ptt = False
          except yaml.YAMLError as exc:
             print(exc)

	
    def clear(self):
        if False:
            for d in [self.wav_dir, self.rec_dir, self.mfcc_dir]:
                if os.path.exists(d):
                    for f in os.listdir(d):
                        os.remove(os.path.join(d, f))

    def save_wave(self, start_buffer, end_buffer):
        if not os.path.exists(self.wav_dir):
            os.mkdir(self.wav_dir)
        wav_file_bname = os.path.join(self.wav_dir,'cvsp_recorder_{}_{}_{}'.format(time.strftime('%Y%m%d_%H%M%S'),start_buffer,end_buffer))
        self.wav_file = wav_file_bname + '.wav'
	if self.CR.n_channels>1 and self.use_beamforming:
	    xs, ys, ymin, ymax = self.CR.wave(start_buffer, end_buffer, beamforming=True)
	    y_out = ys
	else:
	    xs, ys, ymin, ymax = self.CR.wave(start_buffer, end_buffer)
	    if self.CR.n_channels>1:
	        y_out = ys[:, 0]
	    else:
	    	y_out = ys
	
        scipy.io.wavfile.write(self.wav_file,self.CR.rate,y_out)
	if self.CR.rate>self.models_rate:
	    subprocess.check_output(['sox', self.wav_file, '-r', str(self.models_rate), wav_file_bname+'_16k.wav'])
	    self.wav_file = wav_file_bname + '_16k.wav';

        return self.wav_file

    def set_rec_output(self, rec_output, msg):
	if not self.increased_robustness:
	    self.rec_output = rec_output
            self.msg = msg
	    print rec_output
            print msg
	else:
  	    if self.recent_rec_results[-1]==rec_output:
		self.rec_output = rec_output
		self.file_queue = deque()
		self.times_queue = deque()
                #print '----------------------- previous: <' + ' >< '.join(self.recent_rec_results) + ' ' + '>'
                #print '----------------------- previous-1: <' + self.recent_rec_results[-1] + ' ' + '>'
		self.recent_rec_results = deque(self.max_recent_rec_results*' ', self.max_recent_rec_results)
                print 'Rec output: ' + rec_output
                self.msg = msg
                #print 'Message: ' + msg
	    else:
	        self.recent_rec_results.append(rec_output)

    def get_msg(self):
        #self.read_output.set()
        return self.msg
    
    def get_rec_output(self):
        #self.read_output.set()
        return self.rec_output

    def empty_rec_output(self):
        self.rec_output = ''
    
    def empty_msg(self):
        self.msg = ''

    def read_recognition_output(self, output):
	tmp = re.search('(SENT-START)? (.+) (SENT-END)?  ==  \[\d+ frames\] (-?\d+\.\d+) \[Ac',output)
	hyps=[]
	scores=[]
	hyps.append(tmp.group(2))
	scores.append(tmp.group(4))
	return hyps,scores 

    def read_nbest_recognition_output(self, wav_file):
        rec_file = wav_file.replace('.wav','.rec').replace(self.wav_dir, self.rec_dir)
        rec = open(rec_file, 'r')
	hyps = []
	scores = []
        words = []
	score = 0
        for ln in rec:
            ln = ln.rstrip('\r\n')
            #print ln
            ln_info = ln.split(' ')
            if len(ln_info) > 0:
	      if ln_info[0] == '///':
		hyps.append(" ".join(words).decode(self.output_enc))
		scores.append(score / len(words))
		words = []
		score = 0
	      else:
		words.append(ln_info[0])
		score = score + float(ln_info[1])
		#print ln_info[0],score
	if len(words) > 0:
	  hyps.append(" ".join(words).decode(self.output_enc))
	  scores.append(score / len(words))

	#sorted_scores_index = sorted(range(len(hyps)), reverse=True, key=lambda k:scores[k])	
	#hyps_sorted = []
	#scores_sorted = []
	#for i in sorted_scores_index:
	#  hyps_sorted.append(hyps[i])
	#  scores_sorted.append(scores[i])
        return hyps,scores

    def recognize_wave(self,rec_type,wav_file):
        t = time.time()
        if not os.path.exists(self.mfcc_dir):
            os.mkdir(self.mfcc_dir)
        if not os.path.exists(self.rec_dir):
            os.mkdir(self.rec_dir)
        mfc_file = wav_file.replace('.wav','.mfc').replace(self.wav_dir, self.mfcc_dir)
	cmd = [os.path.join(self.bin_dir,'HCopy'), '-C', self.hcopy_cfg, wav_file, mfc_file]
        #print " ".join(cmd)
        output = subprocess.check_output(cmd)

        if rec_type == 0:
            subprocess.check_output([os.path.join(self.bin_dir,'HVite'), '-C', self.hvite_cfg,'-l', self.rec_dir,'-k','-J', self.classes, '-t', '200.0',
                                 '-J', self.xforms,'mllr1','-h','''*.mfc''','-H',self.models,'-w', self.kws_wordnet, '-p',
                                 '0.0', '-o','TM',self.dictionary, self.model_list, mfc_file])
        elif rec_type == 1:
	   if self.nhyps > 1:
	      #print "n-Best hypothesis recognition"
	      cmd=[os.path.join(self.bin_dir,'HVite'), '-n', '%s' % (self.nhyps), '%s' % (self.ntokens),'-T', '1', 
		  '-C', self.hvite_cfg,'-l', self.rec_dir,'-k','-J', self.classes, '-t', '220.0',
                  '-J', self.xforms,'mllr1','-h','''*.mfc''','-H',self.models,'-w', self.asr_wordnet, '-p',
                  '0.0', '-o','TM',self.dictionary, self.model_list, mfc_file]
	      #print " ************************* "
	      #print " Output 2 "
	      #print " ------------------------- "
	      #print " ".join(cmd)
	      #output = subprocess.check_output(cmd)
	      #print output 
	      #print " ------------------------- "
	      #print " Output 1 "
	      #print " ------------------------- "
	   else:
	      cmd=[os.path.join(self.bin_dir,'HVite'),'-T', '1', 
		  '-C', self.hvite_cfg,'-l', self.rec_dir,'-k','-J', self.classes, '-t', '220.0',
                  '-J', self.xforms,'mllr1','-h','''*.mfc''','-H',self.models,'-w', self.asr_wordnet, '-p',
                  '0.0', '-o','TM',self.dictionary, self.model_list, mfc_file]
	      #print " ".join(cmd)
	      #output = subprocess.check_output(cmd)
	      #print output1
	   #print " \n ************************* \n "
	   #print " ".join(cmd)
           output = subprocess.check_output(cmd)
	   #print output
        else:
            subprocess.check_output([os.path.join(self.bin_dir,'HVite'), '-C', self.hvite_cfg,'-l', self.rec_dir,'-k','-J', self.classes, '-t', '300.0',
                                 '-J', self.xforms,'mllr1','-h','''*.mfc''','-H',self.models,'-w', self.kws_wordnet, '-p',
                                 '0.0', '-o','TM',self.dictionary, self.model_list, mfc_file])
	#print '1-Best Recognition took %.3f seconds' % (time.time() - t)

        if output.find('No tokens survived')<0:
	    if self.nhyps > 1:
	      [hyps,scores] = self.read_nbest_recognition_output(wav_file)
	      #print 'BEFORE rescoring'
              #print hyps
	      #print scores
              #if len(hyps) > 1:
              #   if float(scores[1])>float(scores[0]):
              #       print '!!!! 2nd hyp has better score !!!!'
	      [hyps,scores] = self.score_hyps(wav_file, hyps)
	      #print 'AFTER rescoring'
              #print hyps
	      #print scores
              #if len(hyps) > 1:
              #   if float(scores[1])>float(scores[0]):
              #       print '!!!! 2nd hyp has better score !!!!'
	    else:
	      [hyps,scores] = self.read_recognition_output(output)
	else:
	    hyps=[]
	    scores=[]
	    print 'No tokens survived'

	return hyps,scores

    def score_hyps(self,wav_file, hyps_in):
	#print 'Scoring n-Best hypothesis...'
        t = time.time()
        mfc_file = wav_file.replace('.wav','.mfc').replace(self.wav_dir, self.mfcc_dir)
	hyps_out = []
	scores = []
	for hyp in hyps_in:
	  label=hyp.replace(' ','')
          lab_file = wav_file.replace('.wav','.lab').replace(self.wav_dir, self.mfcc_dir)
	  f = open(lab_file,'w')
	  #print hyp.replace(' ','\n')
	  f.write('SENT-START\n')
	  f.write(hyp.replace(' ','\n'))
	  f.write('\nSENT-END\n')
	  f.close()
	  cmd=[os.path.join(self.bin_dir,'HVite'),'-T', '1','-a', 
		  '-C', self.hvite_cfg,'-k','-J', self.classes, '-t', '200.0',
                  '-J', self.xforms,'mllr1','-h','''*.mfc''','-H',self.models, '-p',
                  '0.0', '-o','N',self.dictionary, self.model_list, mfc_file]
	  #print " ".join(cmd)
	  output = subprocess.check_output(cmd)
          if output.find('No tokens survived')<0:
	    score = re.search('frames\] (-?\d+\.\d+) \[Ac',output).group(1)
	    hyps_out.append(hyp)
	    scores.append(score)
	#print 'Scoring n-Best hypotheses took %.3f seconds' % (time.time() - t)
	return hyps_out,scores

    def start_recording(self):
        if not self.started_talking:
            self.CR.continuous_start_alsa()
            self.started_talking = True
	if self.ptt:
	    self.s_thread = PressToTalkThread(self)
       	    self.s_thread.start()
	elif self.use_vad:
	    self.v_thread = VadThread(self)
       	    self.v_thread.start()
	else:
	    self.s_thread = SpeechThread(self)
       	    self.s_thread.start()

        self.a_thread = AsrThread(self)
        self.a_thread.start()
        self.threads_die_now = False

    def end_recording(self):
        print 'Ending recording'
        self.CR.continuous_end()
        self.started_talking = False
        self.threads_die_now = True

    def push_file(self,wav_file,end_frame):
        with threading.Lock():
            self.file_queue.append(wav_file)
            self.times_queue.append(end_frame)

class VadThread(threading.Thread):
    def __init__(self, rt_audio):
	threading.Thread.__init__(self)
	self.rt = rt_audio
	self.vad_start_frame = 0
	self.speech_start_frame = 0
	self.speech_end_frame = 0
	self.n_speech_frames = 0
	self.first_time = True

    def run_vad(self):
        while True:
            time.sleep(2*self.rt.vad_frame_time)
	    #print 'Starting VAD: ' +  str(time.clock())
            if self.rt.threads_die_now:
                break
            curr_start = self.rt.CR.get_current_start()
	    
            curr_frame = int(curr_start / self.rt.CR.buffersize)

	    if curr_frame>100 and self.first_time:
		curr_frame = 100

	    xs, ys, ymin, ymax = self.rt.CR.wave(self.vad_start_frame, curr_frame, beamforming=True)


	    if self.first_time:
		
	        vout, zo = vadsohn(ys, self.rt.CR.rate, 'a')
		self.first_time = False
	    else:
		vout, zo = vadsohn(ys, zo, 'a')

	    #print 'Ending VAD: ' +  str(time.clock())
	    if any(vout):
		if self.n_speech_frames==0:
		    #self.rt.publisher.publish('0 ' + '%s' % rospy.get_rostime().to_nsec()) # START
		    self.speech_start_frame = self.vad_start_frame
		self.n_speech_frames += 1
		print self.n_speech_frames
		self.speech_end_frame = curr_frame
	    else:
		if self.n_speech_frames >= 3 and self.n_speech_frames < 7:
		    #self.rt.publisher.publish('1 ' + '%s' % rospy.get_rostime().to_nsec()) # END 
            	    wav_file = self.rt.save_wave(self.speech_start_frame, curr_frame)
                    self.rt.push_file(wav_file, curr_frame)
		#elif self.n_speech_frames > 0:
		 #   self.rt.publisher.publish('2 ' + '%s' % rospy.get_rostime().to_nsec()) # REJECT
		self.n_speech_frames = 0

	    self.vad_start_frame = curr_frame + 1

    def run(self):
	self.run_vad()

class PressToTalkThread(threading.Thread):
    def __init__(self, rt_audio):
        threading.Thread.__init__(self)
        self.rt = rt_audio
        self.sub = rospy.Subscriber('gesture_segmentation',String, self.callback)

    def callback(self,data):
        print data
        #code.interact(local=locals())
        press = int(str(data).split()[1])
        print press
        if press == 0:
            curr_start = self.rt.CR.get_current_start()
            curr_frame = int(curr_start / self.rt.CR.buffersize)
            f_start = curr_frame 
            self.rt.CR.frame_start = f_start
        elif press == 1:
            curr_start = self.rt.CR.get_current_start()
            curr_frame = int(curr_start / self.rt.CR.buffersize)
            wav_file = self.rt.save_wave(self.rt.CR.frame_start, curr_frame)
            self.rt.push_file(wav_file, curr_frame)
        
    #def save(self):
        #while True:
	  #if self.rt.threads_die_now:
            #break
          #time.sleep(1.5)
	  #press = raw_input('Press ENTER to START talking')
	  #if press == '':
            #curr_start = self.rt.CR.get_current_start()
            #curr_frame = int(curr_start / self.rt.CR.buffersize)
	    #f_start = curr_frame 
	    #self.rt.CR.frame_start = f_start
	  #press = raw_input('Press ENTER to STOP talking')
	  #if press == '' :
            #curr_start = self.rt.CR.get_current_start()
            #curr_frame = int(curr_start / self.rt.CR.buffersize)
            #wav_file = self.rt.save_wave(self.rt.CR.frame_start, curr_frame)
            #self.rt.push_file(wav_file, curr_frame)
    
    def run(self):
        rospy.spin()


class SpeechThread(threading.Thread):
    def __init__(self, rt_audio):
        threading.Thread.__init__(self)
        self.rt = rt_audio

    def save(self):
        window_length = int(self.rt.asr_frame_dur * self.rt.CR.rate / self.rt.CR.buffersize)
        self.rt.CR.frame_start = 0
        self.rt.CR.frame_end = self.rt.CR.frame_start + window_length - 1

        while True:
            time.sleep(self.rt.asr_frame_shift_time)
            if self.rt.threads_die_now:
                break
            #self.rt.read_output.wait(5)
            curr_start = self.rt.CR.get_current_start()

            #if curr_start < 0.01*self.rt.CR.rate:
            #    continue
            curr_frame = int(curr_start / self.rt.CR.buffersize)
	    f_start = curr_frame - window_length
  	    if f_start<0:
		if self.rt.CR.frame_start == 0:
		    continue
		else:
		    f_start += self.rt.CR.max_n_buffers
	    
	    self.rt.CR.frame_start = f_start
            wav_file = self.rt.save_wave(self.rt.CR.frame_start, curr_frame)
            self.rt.push_file(wav_file, curr_frame)
    def run(self):
        self.save()


class AsrThread(threading.Thread):
    def __init__(self, rt_audio):
        threading.Thread.__init__(self)
        self.rt = rt_audio
        self.paused = False

    def get_file(self):
        if self.rt.file_queue:
            return self.rt.file_queue.popleft(), self.rt.times_queue.popleft()
        else:
            return None, None

    def rec2lab(self,rec_output):
        
        if rec_output.find('komm n')>=0:
           lab='ComeCloser'
        elif rec_output.find('hilfe')>=0:
           lab='Help'
        elif rec_output.find('ich will aufstehen')>=0:
           lab='WantStandUp'
        #elif rec_output.find('gehe weg')>=0:
        #   lab='GoAway'
        #elif rec_output.find('gehe links')>=0:
        #   lab='TurnLeft'
        #elif rec_output.find('gehe rechts')>=0:
        #   lab='TurnRight'
        elif rec_output.find('parke')>=0:
           lab='Park'
        elif rec_output.find('wo bin ich')>=0:
           lab='WhereAmI'
        else:
           lab = 'BG'
           
    def filterOutput(self, rec_output):
	filtered_output = 'BG'
        #print rec_output.find('mobot') >= 0
        if rec_output.find('mobot') >= 0:
            if rec_output.find('komm n')>=0 or rec_output.find('gehe links')>=0 or rec_output.find('gehe rechts')>=0 or rec_output.find('ich will aufstehen')>=0 or rec_output.find('hilfe')>=0 or rec_output.find('gehe weg')>=0 or rec_output.find('park')>=0:
                filtered_output = rec_output
        return filtered_output
	   
    def create_msg(self,hyps,scores): 
       hyps_scores=''
       for i in range(0,len(hyps),1):
           hyps_scores = hyps_scores + ' %s %s' % (self.rec2lab(hyps[i]),scores[i])
           msg = '0 0 0' + hyps_scores
       return msg
    
    def run(self):
        while True:
            if self.rt.threads_die_now:
                break
            wav_file, end_frame = self.get_file()
            if wav_file:
		#print '\n' + wav_file
                [hyps,scores]  = self.rt.recognize_wave(1, wav_file)
		if len(hyps) > 0:
                  #print hyps
                  #print scores
                  rec_output = hyps[0]
		  #print 'HVite output: ' + rec_output
		  rec_output = self.filterOutput(rec_output)
                  msg = self.create_msg(hyps,scores)	
		  #print 'Recognition after filtering: ' + rec_output
		else:
 		  rec_output = ''
                  msg = ''
		#if rec_output == '':
		 #   self.rt.publisher.publish('2 ' + '%s' % rospy.get_rostime().to_nsec()) # REJECT
            
		self.rt.set_rec_output(rec_output,msg)
            	with threading.Lock():
                    self.files = deque()
                    self.times = deque()
            time.sleep(0.04)

if __name__ == "__main__":
    print 'TEST!!!!!!!!!!!!!!'
    currpath = os.path.dirname(os.path.abspath(__file__))
    import ConfigParser
    Config = ConfigParser.ConfigParser()
    Config.read(currpath+ '/../../fusion.ini')
    topic = Config.get('Settings','Topic')
    rospy.init_node('speech_p2t', anonymous=True)
    pub = rospy.Publisher(topic, String)
    CR = CvspRecorder()
    CR.config()
    CR.setup_alsa()

    commandReco = PressToTalkCommandReco(CR)
    commandReco.config()
    commandReco.start_recording()
    rospy.on_shutdown(commandReco.exit_handler)

    #atexit.register(commandReco.exit_handler)
    #signal.signal(signal.SIGINT, commandReco.exit_handler)

    counter = 0
    try:
       while not rospy.is_shutdown():
           time.sleep(0.2)
           output = commandReco.get_msg()
           #print "counter: {}, output: {}".format(counter, output)
           commandReco.empty_rec_output()
           commandReco.empty_msg()
           fields = output.split()
           #print fields
           if len(fields)>0 and fields[3] != 'BG':
                output = re.sub('BG \d+(\.\d+)?','',output)
                output = re.sub('\s+',' ',output)
                rospy.loginfo(output)
                pub.publish(output)
           counter += 1
    except KeyboardInterrupt:
       commandReco.exit_handler()

    commandReco.exit_handler()
   
