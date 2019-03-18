#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, UInt8
import roslib
# roslib.load_manifest('store_stereo_image')
import sys
import cv2
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
from collections import deque
from math import ceil, sqrt, pi, exp
import matplotlib.pyplot as plt
from scipy.signal import medfilt, argrelextrema


class ActivityDetector:
    def __init__(self):

        self.DEBUG = False

        # ROS parameters
        rospy.init_node("press_to_gesture", anonymous=True)
        self.sub_topic = rospy.get_param('/kinect_topic', '/camera/rgb/image_color/')
        self.out_topic = rospy.get_param('/activity_det_topic', '/gesture_segmentation')
        print self.sub_topic, self.out_topic
        self.rgb_sub = rospy.Subscriber(self.sub_topic, sensor_msgs.msg.Image, self.rgb_callback)
        # self.depth_sub = rospy.Subscriber("/camera1/depth_registered/image_rect/", sensor_msgs.msg.Image, self.depth_callback)
        rospy.on_shutdown(self.shutdown)
        self.key_publisher = rospy.Publisher(self.out_topic, String)
        self.bridge = CvBridge()

        # On-line parameters
        self.frame_cnt = 0
        self.real_frame_cnt = 0
        self.state = "Rest"
        self.nonrest_cnt = 0
        self.oflow_frames_buffer = deque(maxlen=2)
        self.scores_buffer = []
        self.scores_filtered_buffer = []
        self.timestamps_buffer = []
        self.gaussWinL = 5
        self.medianWinL = 13
        self.gauss_win = self.gausswin(self.gaussWinL, 6)
        if self.medianWinL % 2 ==0 or self.medianWinL % 2 == 0:
            print("Windows length must be odd")
        self.thres = 0
        self.min_nonrest_frames = 8
        self.max_nonrest_frames = 50
        self.history_size = 800
        self.min_history_size = 100
        self.ref_threshold = 0

        cv2.namedWindow("Kinect - RGB", 1)
        if self.DEBUG:
            # plt.figure(1)
            cv2.namedWindow("Optical Flow", 1)
            plt.ion()
            plt.show()
        # cv2.namedWindow("Kinect - depth", 1)

        # Optical flow parameters
        self.decimate = True
        self.pyr_scale = 0.5
        self.levels = 3
        self.winsize = 11
        self.iterations = 3
        self.poly_n = 5
        self.poly_sigma = 1.2
        self.flags = cv2.OPTFLOW_FARNEBACK_GAUSSIAN
        self.str_el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (14, 14))

    def rgb_callback(self, imageMessage):
        # rospy.loginfo(rospy.get_caller_id() + "rgb callback was triggered (frame %d)" % self.frameCounter_rgb)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="bgr8")
            if self.frame_cnt <= self.min_history_size:
                cv2.putText(cv_image, str(self.min_history_size-self.frame_cnt), (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (50, 50, 255), 2, cv2.CV_AA)
            cv2.imshow("Kinect - RGB", cv_image)
            key = cv2.waitKey(1)
        except CvBridgeError, e:
            rospy.loginfo("Conversion failed")
            print("Conversion failed")
            return

        self.real_frame_cnt += 1
        if self.real_frame_cnt % 2 != 0:
            return
        self.frame_cnt += 1

        if self.decimate:
            size = (int(ceil(cv_image.shape[1] / 2)), int(ceil(cv_image.shape[0] / 2)))
            cv_image = cv2.pyrDown(cv_image, dstsize=size)

        self.oflow_frames_buffer.append(cv_image)
        if self.frame_cnt < 2:
            return


        frame_prev = cv2.cvtColor(self.oflow_frames_buffer[0], cv2.COLOR_BGR2GRAY)
        frame_next = cv2.cvtColor(self.oflow_frames_buffer[1], cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(frame_prev, frame_next, self.pyr_scale, self.levels, self.winsize,
                                            self.iterations,
                                            self.poly_n, self.poly_sigma, self.flags)

        flow_x = cv2.morphologyEx(np.absolute(flow[..., 0]), cv2.MORPH_OPEN,
                                  self.str_el)  # = cv2.normalize(flow[...,0], None, 0, 1, cv2.NORM_MINMAX)
        flow_y = cv2.morphologyEx(np.absolute(flow[..., 1]), cv2.MORPH_OPEN,
                                  self.str_el)  # = cv2.normalize(flow[..., 1], None, 0, 1, cv2.NORM_MINMAX)
        magnitude = np.sqrt(np.square(flow_x) + np.square(flow_y))
        # magnitude = cv2.normalize(magnitude, None, 0, 1, cv2.NORM_MINMAX)

        score = np.sum(magnitude, (0, 1)) / np.size(magnitude)
        self.scores_buffer.append(score)
        self.timestamps_buffer.append(imageMessage.header.stamp.to_nsec())
        if len(self.scores_buffer) > self.history_size:
            del self.scores_buffer[0]
            del self.timestamps_buffer[0]
            self.frame_cnt = self.history_size

        # self.scores_filtered_buffer = np.convolve(np.array(self.scores_buffer), np.array(self.gauss_win), 'same')
        self.scores_filtered_buffer = medfilt(self.scores_buffer, self.medianWinL)
        self.scores_filtered_buffer = np.convolve(np.array(self.scores_filtered_buffer), np.array(self.gauss_win),
                                                  'same')

        if self.frame_cnt <= self.min_history_size:
            if self.frame_cnt == self.min_history_size:
                self.ref_threshold = self.determine_threshold()
            return

        thres = self.determine_threshold()
        if thres> 1.2*self.ref_threshold:
            thres = 1.2 * self.ref_threshold
        elif thres < 0.8*self.ref_threshold:
            thres = 0.8 * self.ref_threshold

        idx = int(ceil(max(self.gaussWinL, self.medianWinL) / 2))

        if self.scores_filtered_buffer[-idx-1] >= thres:
            new_state = "Non Rest"
            if self.state == "Rest":
                if self.DEBUG:
                    print "Rest ---> Non Rest @ %d" % self.timestamps_buffer[-idx-1]
                msg = str(0) + " " + str(self.timestamps_buffer[-idx-1])
                self.key_publisher.publish(msg)
                # publish "start"
            self.nonrest_cnt += 1
        else:
            new_state = "Rest"
            if self.state == "Non Rest":
                if self.DEBUG:
                    print "Non Rest ---> Rest @ %d" % self.timestamps_buffer[-idx-1]
                if self.min_nonrest_frames <= self.nonrest_cnt <= self.max_nonrest_frames:
                    if self.DEBUG:
                        print "End (%d frames)" % self.nonrest_cnt
                    msg = str(1) + " " + str(self.timestamps_buffer[-idx - 1])
                else:
                    if self.DEBUG:
                        print "Reject (%d frames)" % self.nonrest_cnt
                    msg = str(2) + " " + str(self.timestamps_buffer[-idx - 1])
                self.key_publisher.publish(msg)
                self.nonrest_cnt = 0
        self.state = new_state

        if self.DEBUG:
            cv2.imshow('Optical Flow', magnitude)
        key = cv2.waitKey(1)
        if (key == 1048689) or (key == 113):  # 'q' or "Q"
            msg = "100" + " " + str(self.timestamps_buffer[-idx - 1])
            self.key_publisher.publish(msg)
            print "You pressed 'q'. Goodbye!"
            rospy.signal_shutdown("You pressed 'q'. Goodbye!")

        if self.DEBUG:
            # print "frame (real): %d\t frame: %d\t decision frame: %d\t decision: %s\n" % \
            #       (self.real_frame_cnt, self.frame_cnt, self.frame_cnt - idx, state)
            plt.figure(1)
            plt.plot(self.scores_buffer, color='b')
            plt.hold(True)
            plt.plot(self.scores_filtered_buffer, color='r')
            plt.plot(np.ones(len(self.scores_filtered_buffer))*thres, color='g')
            plt.plot((self.frame_cnt-idx, self.frame_cnt-idx), (0, 0), "r^", markersize=20)

            # scores_filtered = medfilt(scores_filtered, 11)
            # plt.plot(scores_filtered, color='g')
            plt.draw()
            plt.pause(0.00001)
            plt.hold(False)

    def depth_callback(self, imageMessage):
        self.frameCounter_depth = self.frameCounter_depth + 1
        rospy.loginfo(rospy.get_caller_id() + "depth callback was triggered (frame %d)" % self.frameCounter_depth)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(imageMessage, desired_encoding="16UC1")
            depth_array = np.array(cv_image, dtype=np.float32)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
            cv2.imshow("Kinect - depth", depth_array)
            cv2.waitKey(3)
        except CvBridgeError, e:
            rospy.loginfo("Conversion failed")
            print("Conversion failed")

    def shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("Shutting down node...")
        if self.DEBUG:
            plt.close("all")

    def gausswin(self, N=11, sigma=1):
        if N % 2 != 0:
            r = range(-int(N / 2), int(N / 2) + 1)
        else:
            r = range(-int(N / 2), int(N / 2))
        win = [exp(-(float(x) ** 2) / (2 * sigma ** 2)) for x in r]
        win = [x / sum(win) for x in win]
        # plt.plot(win)
        return win

    def determine_threshold(self):
        hist, bins = np.histogram(self.scores_filtered_buffer, 100)
        # hist = medfilt(hist, 5)
        max_ind = np.argmax(hist)
        hist_trunc = hist.copy().astype("float")
        hist_trunc[0:max_ind] = float('Inf')
        local_minima = argrelextrema(hist_trunc, np.less)[0]
        if not local_minima.size:
            thres = self.thres
        else:
            thres = bins[local_minima[0]]
            self.thres = thres

        if self.DEBUG:
            plt.figure(2)
            plt.ion()
            plt.plot(bins[:-1], hist, "k")
            plt.hold(True)
            plt.plot((thres, thres), (0, np.max(hist)), "r-")
            plt.draw()
            plt.hold(False)
            # if self.frame_cnt>100:
            #     pass
        return thres



def main(args):
    r = ActivityDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)


    # if key!=-1:
    #     print key
    # if (key == 1048689) or (key == 113): # 'q' or "Q"
    #     # self.key_publisher.publish(str(100))
    #     print "You pressed 'q'. Goodbye!"
    #     rospy.signal_shutdown("You pressed 'q'. Goodbye!")
    # if (key==1048673) or (key==97): # a
    #     msg = str(0) + " "  + str(timestamp)
    #     #time.sleep(7)
    #     self.key_publisher.publish(msg)
    #     print "I just published '0' (START) from " + str(timestamp) + "!"
    # if (key == 1048691) or (key==115):  # s
    #     msg = str(1) + " "  + str(timestamp)
    #     print "I just published '1' (STOP) from " + str(timestamp) + "!"
    #     self.key_publisher.publish(msg)
    # if (key == 1048676) or (key==100):  # d
    #     msg = str(2) + " "  + str(timestamp)
    #     self.key_publisher.publish(msg)
    #     print "I just published '2' (REJECT) from " + str(timestamp) + "!"
