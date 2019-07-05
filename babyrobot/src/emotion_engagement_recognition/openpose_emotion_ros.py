#!/usr/bin/env python
# -*- coding: utf-8 -*-

# From Python
# It requires OpenCV installed for Python
import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy
import cv2
import os
from sys import platform
import argparse
from cv_bridge import CvBridge, CvBridgeError
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages/')
import sensor_msgs
import torchvision.transforms as transforms
import numpy as np
import face_recognition
import socket
import time
import torch
import torch.nn as nn
import torchvision.models as models
from std_msgs.msg import String

class EmotionModelFace(nn.Module):
    """docstring for niki"""
    def __init__(self, regions=True):
        super(EmotionModelFace, self).__init__()

        resnet50 = models.resnet50(pretrained=False)

        last_layer_input_features = resnet50.fc.in_features

        modules = list(resnet50.children())[:-1]  # delete the last fc layer.

        self.features = nn.Sequential(*modules)

        self.emotion_classifier = nn.Linear(2048, 8)

    def forward(self, image):
            features = self.features(image)

            all_features = features.view(features.size(0),-1)

            return self.emotion_classifier(all_features)

# Import Openpose (Windows/Ubuntu/OSX)
dir_path = os.path.dirname(os.path.realpath(__file__))
try:
    # Windows Import
    if platform == "win32":
        # Change these variables to point to the correct folder (Release/x64 etc.) 
        sys.path.append(dir_path + '/../../python/openpose/Release');
        os.environ['PATH']  = os.environ['PATH'] + ';' + dir_path + '/../../x64/Release;' +  dir_path + '/../../bin;'
        import pyopenpose as op
    else:
        # Change these variables to point to the correct folder (Release/x64 etc.) 
        sys.path.append('/home/babyrobot/software/openpose/build/python');
        # If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
        # sys.path.append('/usr/local/python')
        from openpose import pyopenpose as op
except ImportError as e:
    print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
    raise e

# Flags
parser = argparse.ArgumentParser()
parser.add_argument("--image_path", default="../../../examples/media/COCO_val2014_000000000241.jpg", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
args = parser.parse_known_args()

# Custom Params (refer to include/openpose/flags.hpp for more parameters)
params = dict()
params["model_folder"] = "../../../models/"
params["face"] = True
params["hand"] = True
params["net_resolution"] = "128x96"
# Add others in path?
for i in range(0, len(args[1])):
    curr_item = args[1][i]
    if i != len(args[1])-1: next_item = args[1][i+1]
    else: next_item = "1"
    if "--" in curr_item and "--" in next_item:
        key = curr_item.replace('-','')
        if key not in params:  params[key] = "1"
    elif "--" in curr_item and "--" not in next_item:
        key = curr_item.replace('-','')
        if key not in params: params[key] = next_item

# Construct it from system arguments
# op.init_argv(args[1])
# oppython = op.OpenposePython()

affectnet_dic = {
    0: "Neutral",
    1: "Happiness",
    2: "Sadness",
    3: "Surprise",
    4: "Fear",
    5: "Disgust",
    6: "Anger",
    7: "Contempt"
}

affectnet_dic_greek = {
    0: "Ουδέτερο",
    1: "Χαρά",
    2: "Λύπη",
    3: "Έκπληξη",
    4: "Φόβο",
    5: "Αηδία",
    6: "Θυμό",
    7: "Contempt"
}

os.chdir("/home/babyrobot/software/openpose/build/examples/tutorial_api_python")
# Starting OpenPose

image_pub = rospy.Publisher("image_with_body_and_face",sensor_msgs.msg.Image)
# print("aaa")
# -------------------------- Emotion ------------------------ #
model = nn.DataParallel(EmotionModelFace()).cuda()

checkpoint = torch.load("/home/babyrobot/software/openpose/build/examples/tutorial_api_python/2")
# print(checkpoint['state_dict'])
model.load_state_dict(checkpoint['state_dict'])
print("Loaded Checkpoint %s with accuracy %f" % ("resnet50", checkpoint['acc']))

normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])

train_transforms = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((224,224)),
    transforms.ToTensor(),
   normalize,
])

fps = 3
counter = 0


# customizable parameters #####

ticket_name = 'furhat'
iristk_name = 'emotion_system' ###### for iristk only 
server_ip = '192.168.0.105' # ------------------- CHANGE THIS TO 145 for school
port = 1932

###################################

json_format = "{ \"class\": \"iristk.system.Event\", \"event_sender\": \"emotion_system\" , \"event_name\": \"athena.emotion.recognized_english\", \"text\": \"%s\" }\n"
json_format_greek = "{ \"class\": \"iristk.system.Event\", \"event_sender\": \"emotion_system\" , \"event_name\": \"athena.emotion.recognized_greek\", \"text\": \"%s\" }\n"

event_format = "EVENT athena.emotion.recognized_english %d\n"
event_format_greek = "EVENT athena.emotion.recognized_greek %d\n"
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

sock.connect((server_ip, port))

msg = 'CONNECT %s %s \n' % (ticket_name, iristk_name)

sock.send(msg)

# pub_emotion = rospy.Publisher("/emotion_topic", String, queue_size=10)

listening = False

def callback_listen(data):
    
    print 'gfkvvkjvbkrvrkvg'
    global listening, check_buffer
    if listening:
        return

    time.sleep(0.1)
    listening = True
    rospy.loginfo("Emotion Module Started Listening")

def callback(data):
    global fps, counter, listening

    if listening == False:
        counter = 0
        return

    datum = op.Datum()

    cv_image = bridge.imgmsg_to_cv2(data)

    datum.cvInputData = cv_image


    counter += 1
    if counter != fps:
        # image_pub.publish(bridge.cv2_to_imgmsg(datum.cvInputData, "bgr8"))
        return
    else:
        counter = 0

    opWrapper.emplaceAndPop([datum])


    face_locations = face_recognition.face_locations(cv_image)

    if len(face_locations) == 0:
        image_pub.publish(bridge.cv2_to_imgmsg(datum.cvOutputData, "bgr8"))
    else:
        face_location = face_locations[0]

        top, right, bottom, left = face_location

        face_image = datum.cvInputData[top:bottom, left:right]

        face_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2RGB)

        face_image = train_transforms(face_image).unsqueeze(0).cuda()

        with torch.no_grad():
            model.eval()
            result = nn.functional.softmax(model(face_image))


        if result.max(1)[0].item() * 100 > 30 and int(result.max(1)[1].item()) != 0 and int(result.max(1)[1].item()) != 7:
            # if result.max(1)[0].item() == 0:

            # print(affectnet_dic[result.max(1)[1].item()], result.max(1)[0].item(), result*100)

            a = datum.cvOutputData
            cv2.putText(a, '%s'%affectnet_dic[result.max(1)[1].item()], (500,20), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0,0,255), 1, cv2.LINE_AA)
            image_pub.publish(bridge.cv2_to_imgmsg(a, "bgr8"))

            rospy.loginfo("Recognized Emotion %s" % affectnet_dic[result.max(1)[1].item()])

            # -- send english -- #
            j =  json_format % affectnet_dic[result.max(1)[1].item()]
            full_event = event_format % len(j)
            sock.sendall(full_event)
            sock.sendall(j)


            # -- send greek -- #
            j =  json_format_greek % affectnet_dic_greek[result.max(1)[1].item()]
            full_event = event_format_greek % len(j)
            sock.sendall(full_event)
            sock.sendall(j)

            listening = False
            # pub_emotion.publish("%s-%s"%(affectnet_dic[result.max(1)[1].item()], affectnet_dic_greek[result.max(1)[1].item()])

        else:
            image_pub.publish(bridge.cv2_to_imgmsg(datum.cvOutputData, "bgr8"))


rospy.init_node('openpose_body', anonymous=True)
rospy.loginfo("Emotion Module is connected and ready")

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()


bridge = CvBridge()
rgb_sub = rospy.Subscriber('/kinect4/qhd/image_color', sensor_msgs.msg.Image, callback) # ------------------- CHANGE THIS TO kinect1 for school
print 'gjgfkjg'
rospy.Subscriber("/athena_emotion", String, callback_listen)

rospy.spin()
