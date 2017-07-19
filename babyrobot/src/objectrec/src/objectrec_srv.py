#!/usr/bin/env python
import os
import uuid

import numpy as np
import rospy
import tensorflow as tf
from PIL import Image
from babyrobot_msgs.msg import ObjectRecognitionResult, RecognizedObject, \
    BoundingBox, PixelPoint
from babyrobot_msgs.srv import ObjectRecognition, ObjectRecognitionResponse

from babyrobot.lib.utils import yaml2dict
from tf.utils import check_model, load_frozen_model, get_labels_map, \
    orec_image, extract_box_from_image, get_abs_box_from_image


def handle_objectrec(req):
    rospy.loginfo('Request metadata: {}'.format(req.metadata))

    with detect_graph.as_default():
        with tf.Session(graph=detect_graph) as sess:
            data = np.fromstring(req.frame.data, dtype=np.uint8)

            image = Image.fromarray(
                data.reshape((req.frame.height, req.frame.width, 3)))

            # show for debugging purposes
            # image.show()

            (boxes, classes, scores, num_detections) = orec_image(sess,
                                                                  detect_graph,
                                                                  image)
            objects = list(zip(boxes, scores, classes))
            frames = extract_box_from_image(image, objects,
                                            config["model"]["threshold"])

            # visualize recognized objects for debugging purposes
            # visualize_recognized_objects(image, boxes, classes, scores,
            #                              label_map,
            #                              config["model"]["threshold"])

            msg = ObjectRecognitionResult()
            msg.header.id = str(uuid.uuid4())
            msg.header.timestamp = rospy.Time.now()
            msg.related_frame_id = msg.header.id

            for (box, score, cls), frame in zip(objects, frames):
                rec_object = RecognizedObject()

                rec_object.proximity = 0
                rec_object.label = label_map[cls]["name"]
                rec_object.confidence = score

                xmin, ymin, xmax, ymax = get_abs_box_from_image(image, box)
                bbox = BoundingBox(PixelPoint(xmin, ymin),
                                   PixelPoint(xmax, ymax))
                rec_object.bounding_box = bbox

                msg.objects.append(rec_object)

            return ObjectRecognitionResponse(msg)


def objectrec_server():
    rospy.init_node('objectrec_server')
    rospy.Service('objectrec', ObjectRecognition, handle_objectrec)
    rospy.loginfo("Object Recognition server started.")
    rospy.spin()


if __name__ == "__main__":
    _script_loc = os.path.dirname(os.path.abspath(__file__))
    _conf_path = os.path.join(_script_loc, 'tf', 'config.yaml')
    config = yaml2dict(_conf_path)

    rospy.loginfo("Loading Object Recognition module...")

    # Download Model
    check_model(config["model"]["name"])

    # Load a (frozen) Tensorflow model into memory.
    detect_graph = load_frozen_model(config["model"]["name"])

    # Loading label map
    label_map = get_labels_map(config["model"]["labels"],
                               config["model"]["classes"])

    objectrec_server()
