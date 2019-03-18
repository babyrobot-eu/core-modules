#!/usr/bin/env python
__author__ = 'irodoma'

import sys
import re
import os.path
import rospy
from std_msgs.msg import String
import ConfigParser
import threading
import json
import time

class audioGestCmdRec():
    def __init__(self):
        self.use_sensor_times = True
        self.curdir = os.path.dirname(os.path.abspath(__file__))
        self.recoTopicCFG = self.curdir + '/fusion.ini'
        self.promptsTopicCFG = self.curdir + '/wavplayer.ini'
        self.navigationTopicCFG = self.curdir + '/navigationCmds.ini'
        self.visualizationTopicCFG = self.curdir + '/visualization.ini'
        self.ovlp_thres = 0.5
        self.arrival_time_thres = 5000000
        self.wa = 0.8
        self.wv = 0.2
        self.N = 2
        self.MAX_AUDIO_GLOB_SCORE = -80.6511
        self.MIN_AUDIO_GLOB_SCORE = -101.5445
        self.audioMsgQueue1 = []
        self.audioMsgQueue2 = []
        self.audioMsgQueue3 = []
        self.audioMsgQueue = []
        rospy.init_node('fusion')
        self.clockTicks = 0
        self.speechArrivalTick1 = 0
        self.speechArrivalTick2 = 0
        self.speechArrivalTick3 = 0
        self.speechArrivalTick = 0
        self.waitingTicks = 0
        self.publisher_to_prompts = self.create_publisher(self.promptsTopicCFG)
        self.publisher_to_navigation = self.create_publisher(self.navigationTopicCFG)
        self.publisher_to_visualization = self.create_publisher(self.visualizationTopicCFG)
        self.logfile = self.curdir + '/logs/AudioGestRecResults_%s' % (time.strftime('%Y%m%d_%H%M%S'))
        self.prev_rec = ''
        self.reset_vis_msg = {'modalityID':5}
        self.publisher_to_visualization.publish(json.dumps(self.reset_vis_msg))
        #self.wavdir = self.read_wavdir(self.promptsTopicCFG)
        self.wavdir = rospy.get_param('wavdir','/home/nick/catkin_ws/src/cvsp_audio_gesture/src/audio_rec/prompts/validation/en')
        self.nTicksReset = rospy.get_param('visualization_reset_clock_ticks',7) 
        self.waiting_thres = rospy.get_param('waiting_clock_ticks',11) 
        self.single_modality_mode = rospy.get_param('single_modality_mode',0)


    def filter_recognition(self,rec):
        far2near=['comeNear']
        near2near=['help', 'wantStandUp', 'whatTimeIsIt']
        near2far=['goAway']
        far=['park']

	output = rec + ' rejected' 
        if self.prev_rec == '' and re.search('('+"|".join(far2near)+')',rec):
            output = rec
        elif re.search('('+'|'.join(far2near)+'|'+'|'.join(near2near)+')',self.prev_rec) and re.search('('+'|'.join(far2near)+')',rec) is None:
            output = rec 
        elif re.search('('+'|'.join(far2near)+'|'+'|'.join(near2near)+')',self.prev_rec) and re.search('('+'|'.join(far2near)+')',rec) is None:
            output = rec 
        elif re.search('('+'|'.join(near2far)+')',self.prev_rec) and re.search('('+'|'.join(far)+')',rec):
            output = rec 
        print output 
 
    def read_wavdir(self,cfg):
        config = ConfigParser.ConfigParser(cfg)
        config.read(cfg)
        return config.get('Settings','wav_dir')

    def create_publisher(self,cfg):
        config = ConfigParser.ConfigParser()
        config.read(cfg)
        pub = rospy.Publisher(config.get('Settings','Topic'), String)
        return pub

    def rec_listener(self):
        config = ConfigParser.ConfigParser()
        config.read(self.recoTopicCFG)
        self.sub = rospy.Subscriber(config.get('Settings','Topic'),String, self.rec_callback)
        rospy.spin()

    
    def rec_callback(self, data):
        #rospy.loginfo('Fusion: I heard %s', data.data)
        msg = self.read_message(data.data)

        # reset visualization
        if self.clockTicks == self.nTicksReset:
            self.publisher_to_visualization.publish(json.dumps(self.reset_vis_msg))
            
        # append message in the corresponding queue
        if 'clockTime' in msg.keys():
            self.clockTicks = self.clockTicks + 1
            #print 'Clock Tick: %d' % self.clockTicks
        elif msg['modalityID'] == 1:
            #self.publisher_to_visualization.publish(json.dumps(msg))
            #print 'Gesture1 Message to visualization:'
            #print json.dumps(msg)
            #print '---Message from Gesture Recognition---'
            #print msg
            self.audioMsgQueue1.append(msg)
            self.speechArrivalTick1 = self.clockTicks
        elif msg['modalityID'] == 2:
            #self.publisher_to_visualization.publish(json.dumps(msg))
            #print 'Gesture2 Message to visualization:'
            #print json.dumps(msg)
            #print '---Message from Gesture Recognition---'
            #print msg
            self.audioMsgQueue2.append(msg)
            self.speechArrivalTick2 = self.clockTicks
        elif msg['modalityID'] == 3:
            #self.publisher_to_visualization.publish(json.dumps(msg))
            #print 'Gesture3 Message to visualization:'
            #print json.dumps(msg)
            #print '---Message from Gesture Recognition---'
            #print msg
            self.audioMsgQueue3.append(msg)
            self.speechArrivalTick3 = self.clockTicks
        else:
            print 'Wrong value for the 1st tupple of the message'
       
        # search in queues for close segments to combine
        T = self.waiting_thres
        speechEndsWaiting1 = (self.speechArrivalTick1 > 0 and (self.clockTicks - self.speechArrivalTick1 >= T or self.single_modality_mode))
        speechEndsWaiting2 = (self.speechArrivalTick2 > 0 and (self.clockTicks - self.speechArrivalTick2 >= T or self.single_modality_mode))
        speechEndsWaiting3 = (self.speechArrivalTick3 > 0 and (self.clockTicks - self.speechArrivalTick3 >= T or self.single_modality_mode))
        allArrived = (self.speechArrivalTick1 > 0 and self.speechArrivalTick2 > 0 and self.speechArrivalTick3 > 0 and abs(self.speechArrivalTick1-self.speechArrivalTick2)<T and abs(self.speechArrivalTick1-self.speechArrivalTick3)<T)
        if speechEndsWaiting1 or speechEndsWaiting2 or speechEndsWaiting3 or allArrived:
            if len(self.audioMsgQueue1)>0 and len(self.audioMsgQueue2)>0 and len(self.audioMsgQueue3)>0: 
                self.printResult(self.audioMsgQueue1[0])
                self.printResult(self.audioMsgQueue2[0])
                self.printResult(self.audioMsgQueue3[0])
                msg = self.fusion_three_speech(self.audioMsgQueue1[0],self.audioMsgQueue2[0],self.audioMsgQueue3[0])
                if msg != None:
                    self.printResult(msg)
                    #print '------------------------------------------------------Fusion Message to visualization:'
                    #print json.dumps(msg)
                    self.publisher_to_visualization.publish(json.dumps(msg))
                else:
                    self.printResult({'modalityID':4,'segStartTime':0,'segEndTime':0,'arrivalTime':0,'hypotheses':['REJECTED'],'scores':[0]})
            
            elif len(self.audioMsgQueue1)>0 and len(self.audioMsgQueue2)>0:
                self.printResult(self.audioMsgQueue1[0])
                self.printResult(self.audioMsgQueue2[0])
                msg = self.fusion_two_speech(self.audioMsgQueue1[0],self.audioMsgQueue2[0])
                if msg != None:
                    self.printResult(msg)
                    #print '------------------------------------------------------Fusion Message to visualization:'
                    #print json.dumps(msg)
                    self.publisher_to_visualization.publish(json.dumps(msg))
                else:
                    self.printResult({'modalityID':4,'segStartTime':0,'segEndTime':0,'arrivalTime':0,'hypotheses':['REJECTED'],'scores':[0]})
            elif len(self.audioMsgQueue1)>0 and len(self.audioMsgQueue3)>0:
                self.printResult(self.audioMsgQueue1[0])
                self.printResult(self.audioMsgQueue3[0])
                msg = self.fusion_two_speech(self.audioMsgQueue1[0],self.audioMsgQueue3[0])
                if msg != None:
                    self.printResult(msg)
                    #print '------------------------------------------------------Fusion Message to visualization:'
                    #print json.dumps(msg)
                    self.publisher_to_visualization.publish(json.dumps(msg))
                else:
                    self.printResult({'modalityID':4,'segStartTime':0,'segEndTime':0,'arrivalTime':0,'hypotheses':['REJECTED'],'scores':[0]})
            elif len(self.audioMsgQueue2)>0 and len(self.audioMsgQueue3)>0:
                self.printResult(self.audioMsgQueue2[0])
                self.printResult(self.audioMsgQueue3[0])
                msg = self.fusion_two_speech(self.audioMsgQueue2[0],self.audioMsgQueue3[0])
                if msg != None:
                    self.printResult(msg)
                    #print '------------------------------------------------------Fusion Message to visualization:'
                    #print json.dumps(msg)
                    self.publisher_to_visualization.publish(json.dumps(msg))
                else:
                    self.printResult({'modalityID':4,'segStartTime':0,'segEndTime':0,'arrivalTime':0,'hypotheses':['REJECTED'],'scores':[0]})
            elif len(self.audioMsgQueue1)>0:
                self.printResult(self.audioMsgQueue1[0])
                self.audioMsgQueue1[0]['modalityID'] = 4
                self.printResult(self.audioMsgQueue1[0])
                self.publisher_to_visualization.publish(json.dumps(self.audioMsgQueue1[0]))
            elif len(self.audioMsgQueue2)>0:
                self.printResult(self.audioMsgQueue2[0])
                self.audioMsgQueue2[0]['modalityID'] = 4
                self.printResult(self.audioMsgQueue2[0])
                self.publisher_to_visualization.publish(json.dumps(self.audioMsgQueue2[0]))
            elif len(self.audioMsgQueue3)>0:
                self.printResult(self.audioMsgQueue3[0])
                self.audioMsgQueue3[0]['modalityID'] = 4
                self.printResult(self.audioMsgQueue3[0])
                self.publisher_to_visualization.publish(json.dumps(self.audioMsgQueue3[0]))
                
            self.audioMsgQueue1 = []
            self.audioMsgQueue2 = []
            self.audioMsgQueue3 = []
            self.speechArrivalTick1 = 0
            self.speechArrivalTick2 = 0
            self.speechArrivalTick3 = 0
            self.clockTicks = 1
    

    def printResult(self,msg,dif=0):
        modalityIDs = ['GestureRecognition','SpokenCommandRecognition1','SpokenCommandRecognition2','SpokenCommandRecognition3','FUSION']
        print '####  ' + modalityIDs[msg['modalityID']] + ' ####'
        print '%f %s: %s %s\n' % (msg['arrivalTime'],modalityIDs[msg['modalityID']],' '.join(msg['hypotheses']),' '.join(str(n) for n in msg['scores']))
        #print msg
        with open(self.logfile,'a') as f:
            #f.write('%s %f %f %s %s\n' % (modalityIDs[msg['modalityID']],msg['arrivalTime'],dif,' '.join(msg['hypotheses']),' '.join(str(n) for n in msg['scores'])))
            if (modalityIDs[msg['modalityID']] == 'GestureRecognition') or (modalityIDs[msg['modalityID']] == 'FUSION'):
                f.write('%s %.2f %s %s\n' % (modalityIDs[msg['modalityID']],msg['arrivalTime'],' '.join(msg['hypotheses']),' '.join(str(n) for n in msg['scores'])))
            else:
               f.write('%s %.2f %s %s %s\n' % (modalityIDs[msg['modalityID']],msg['arrivalTime'],' '.join(msg['hypotheses']),' '.join(str(n) for n in msg['scores']),' '.join(str(n) for n in msg['raw_scores'])))
               
    def find_close_arrived_segments(self):
        tdoa_min = 100000000000
        i_sel = 0
        j_sel = 0
        for i in range(0,len(self.audioMsgQueue),1):
            for j in range(0,len(self.audioMsgQueue),1):
                tdoa = abs(self.audioMsgQueue[j]['arrivalTime'] - self.audioMsgQueue[i]['arrivalTime'])
                if tdoa < tdoa_min:
                    i_sel = i
                    j_sel = j
                    tdoa_min = tdoa
        return i_sel,j_sel,tdoa_min

    def find_close_recorded_segments(self):
        ovlp_max = 0
        i_sel = 0
        j_sel = 0
        for i in range(0,len(self.audioMsgQueue),1):
            for j in range(0,len(self.audioMsgQueue),1):
                amsg = self.audioMsgQueue[i]
                vmsg = self.audioMsgQueue[j]
                ovlp = self.segment_ovlp(amsg['segStartTime'],amsg['segEndTime'],vmsg['segStartTime'],vmsg['segEndTime'])
                if ovlp > ovlp_max:
                    i_sel = i
                    j_sel = j
                    ovlp_max = ovlp
        return i_sel,j_sel,ovlp_max

    def segment_ovlp(self,ref_start,ref_end,det_start,det_end):
        ref_dur = ref_end-ref_start
        det_dur = det_end-det_start
        # Find overlap(segment1,segment2)/ duration(segment1)%
        overlap = 0

        if (det_start<=ref_start) and (ref_start<det_end) and (det_end<=ref_end):
            overlap = det_end-ref_start
        elif (ref_start<=det_start) and (det_start<ref_end) and (ref_start<det_end) and (det_end<=ref_end):
            overlap = det_dur
        elif (ref_start<=det_start) and (det_start<ref_end) and (det_end>=ref_end):
            overlap = ref_end-det_start
        elif (det_start<=ref_start) and (det_end>=ref_end):
            overlap = ref_dur
        return float(overlap) / ref_dur


    def read_message(self,msg):
        hyps=[]
        scores=[]
        fields = msg.split()
        #print fields
        modalityID = int(fields.pop(0))
        if modalityID == 5:
            m = {'clockTime':int(fields.pop(0))}
            return m
        segStartTime = int(fields.pop(0))
        segEndTime = int(fields.pop(0))
        for i in range(0,len(fields),1):
            if i % 2 == 0:
                hyps.append(fields[i])
            else:
                scores.append(float(fields[i]))
        arrivalTime = rospy.get_time()
        if modalityID == 0:
            m = {'modalityID':modalityID,'segStartTime':0,'segEndTime':0,'arrivalTime':0,'hypotheses':0,'scores':0,'raw_scores':0}
        if modalityID == 1:
            norm_scores = self.normalize_speech_scores(scores)
            m = {'modalityID':modalityID,'segStartTime':segStartTime,'segEndTime':segEndTime,'arrivalTime':arrivalTime,'hypotheses':hyps,'scores':norm_scores,'raw_scores':scores}
        if modalityID == 2:
            norm_scores = self.normalize_speech_scores(scores)
            m = {'modalityID':modalityID,'segStartTime':segStartTime,'segEndTime':segEndTime,'arrivalTime':arrivalTime,'hypotheses':hyps,'scores':norm_scores,'raw_scores':scores}
        if modalityID == 3:
            norm_scores = self.normalize_speech_scores(scores)
            m = {'modalityID':modalityID,'segStartTime':segStartTime,'segEndTime':segEndTime,'arrivalTime':arrivalTime,'hypotheses':hyps,'scores':norm_scores,'raw_scores':scores}
        return m

    def normalize_speech_scores(self,scores):
        scores_norm = []
        #print 'scoressss',scores
        for i in range(0,len(scores),1):
            scores_norm.append((scores[i] - self.MIN_AUDIO_GLOB_SCORE) / (self.MAX_AUDIO_GLOB_SCORE - self.MIN_AUDIO_GLOB_SCORE))
        return scores_norm 
                 
    def sort_hyps(self,hyps,scores):
        sorted_scores_index = sorted(range(len(hyps)), reverse=True, key=lambda k:scores[k])
        hyps_sorted = []
        scores_sorted = []
        for i in sorted_scores_index:
            hyps_sorted.append(hyps[i])
            scores_sorted.append(scores[i])
        return hyps_sorted,scores_sorted
    
    def fusion_1best_speech(self,audioMsg,visualMsg):
        gest_max_score_thres = 0.8
        gest_max_scores_diff_thres = 0.2
        top_rank_window = 2
        down_rank_window = 2

        # check modality confidence using hypotseses scores measures
        gest_confident_max = (visualMsg['scores'][0] > gest_max_score_thres) 
        #print gest_confident_max
        gest_close_besthyps = (visualMsg['scores'][0] - visualMsg['scores'][1] < gest_max_scores_diff_thres) 
        #print gest_close_besthyps

        # ranking comparison checks 
        modalities_agree = (re.search('('+'|'.join(visualMsg['hypotheses'][0:top_rank_window])+')',audioMsg['hypotheses'][0]))
        modalities_disagree = (re.search('('+'|'.join(visualMsg['hypotheses'][-down_rank_window:])+')',audioMsg['hypotheses'][0]))

        # fusion logic
        output = None
        if modalities_agree:
            audioMsg['modalityID']=2
            output = audioMsg
        
        return output	 
        
        
    def fusion_two_speech(self,audioMsg1,audioMsg2):
        gest_max_score_thres = 0.8
        gest_max_scores_diff_thres = 0.2
        top_rank_window = 0
        down_rank_window = 2


        # ranking comparison checks 
        modalities_agree = (re.search('('+'|'.join(audioMsg2['hypotheses'][0:top_rank_window])+')',audioMsg1['hypotheses'][0]))
        modalities_disagree = (re.search('('+'|'.join(audioMsg2['hypotheses'][-down_rank_window:])+')',audioMsg1['hypotheses'][0]))

        # fusion logic
        output = None
        if modalities_agree:
            audioMsg1['modalityID']=4
            output = audioMsg1
        
        return output  
    
    def fusion_three_speech(self,audioMsg1,audioMsg2,audioMsg3):
        gest_max_score_thres = 0.8
        gest_max_scores_diff_thres = 0.2
        top_rank_window = 0
        down_rank_window = 2


        # ranking comparison checks 
        modalities_agree1 = (re.search('('+'|'.join(audioMsg2['hypotheses'][0:top_rank_window])+')',audioMsg1['hypotheses'][0]))
        modalities_agree2 = (re.search('('+'|'.join(audioMsg3['hypotheses'][0:top_rank_window])+')',audioMsg1['hypotheses'][0]))
        modalities_agree3 = (re.search('('+'|'.join(audioMsg3['hypotheses'][0:top_rank_window])+')',audioMsg2['hypotheses'][0]))
        #modalities_disagree = (re.search('('+'|'.join(visualMsg['hypotheses'][-down_rank_window:])+')',audioMsg['hypotheses'][0]))

        # fusion logic
        output = None
        if modalities_agree1:
            audioMsg1['modalityID']=4
            output = audioMsg1
        elif modalities_agree2:
            audioMsg1['modalityID']=4
            output = audioMsg1
        elif modalities_agree3:
            audioMsg2['modalityID']=4
            output = audioMsg2
        
        return output     

    def hypRescoring(self,audioMsg,visualMsg):
        # the audio n-best list is supposed to be sorted
        scores = []
	n = min([len(audioMsg['hypotheses']), self.N])
        for i in range(0,n,1):
            for j in range(0,len(visualMsg['hypotheses']),1):
                if visualMsg['hypotheses'][j] == audioMsg['hypotheses'][i]:
                    break
            scores.append(self.wa * visualMsg['scores'][i] + self.wv *  visualMsg['scores'][j])
        ix = sorted(range(n), reverse=True, key=lambda k: scores[k])
        hyps_sorted = []
        scores_sorted = []
        for i in ix:
            hyps_sorted.append(audioMsg['hypotheses'][i])
            scores_sorted.append(scores[i])

        segStartTime=min(audioMsg['segStartTime'],visualMsg['segStartTime'])
        segEndTime=max(audioMsg['segEndTime'],visualMsg['segEndTime'])
        m = {'modalityID':2,'segStartTime':segStartTime,'segEndTime':segEndTime,'hypotheses':hyps_sorted,'scores':scores_sorted,'arrivalTime':rospy.get_time()}
        return m

    def readCSV(self, csv, tierName):
        scores = dict()
        labs = dict()
        for line in open(csv):
            if not re.search(tierName, line): continue
            line = line.rstrip('\n\r')
            col = line.split('\t')
            times = "%2.3f %2.3f" % (float(col[1]), float(col[2]))
            hypnum = re.search("H(\d+)", col[0], re.IGNORECASE).group(1)
            tmp = col[3].split(" ")
            lab = tmp[0]
            if tierName is 'AUDIO':
                score = (float(tmp[1]) - self.MIN_AUDIO_GLOB_SCORE) / (
                    self.MAX_AUDIO_GLOB_SCORE - self.MIN_AUDIO_GLOB_SCORE)
            else:
                score = float(tmp[1])

            if times in scores:
                scores[times][lab] = score
                labs[times].append(lab)
            else:
                labs[times] = []
                scores[times] = dict()
                scores[times][lab] = score
                labs[times].append(lab)

        return labs, scores


    def batchScoreComb(self, alabs, ascores, vscores):

        for times in alabs:
            av_nbest_labs = alabs[times][0:self.N]
            av_scores = []
            for lab in av_nbest_labs:
                av_scores.append(self.wa * ascores[times][lab] + self.wv * vscores[times][lab])

            ix = sorted(range(self.N), reverse=True, key=lambda k: av_scores[k])
            times = times.split(' ')
            print "AV\t%f\t%f\t%s %f" % (float(times[0]), float(times[1]), av_nbest_labs[ix[0]], av_scores[ix[0]])

    def test_rec_filtering(self):
        self.prev_rec = ''
        self.filter_recognition('help')
        self.filter_recognition('comeNear')
        self.prev_rec = 'comeNear'
        self.filter_recognition('help')
        self.prev_rec = 'help'
        self.filter_recognition('comeNear')
        self.filter_recognition('goAway')
        self.prev_rec = 'goAway'
        self.filter_recognition('wantStandUp')
        self.filter_recognition('park')

if __name__ == "__main__":
    f = audioGestCmdRec()
    #[alabs, ascores] = f.readCSV('p1.csv', 'AUDIO')
    #[vlabs, vscores] = f.readCSV('NBest_p1.csv', 'combined')
    #f.scoreComb(alabs, ascores, vscores)
    #f.speech_listener()
    #f.gest_listener()
   
    f.rec_listener()
    #f.test_rec_filtering()
   
