#!/usr/bin/env python
import uuid

import numpy as np
import rospy
import tensorflow as tf
from PIL import Image
from babyrobot_msgs.msg import (
    ObjectRecognitionResult,
    RecognizedObject,
    BoundingBox,
    PixelPoint
)
from babyrobot_msgs.srv import ObjectRecognition, ObjectRecognitionResponse

from tf.config import OBJECTREC as CONFIG
from tf.utils import (
    check_model,
    load_frozen_model,
    get_labels_map,
    orec_image,
    extract_box_from_image,
    box_norm2abs
)


def handle_objectrec(req):
    rospy.loginfo('Request metadata: {}'.format(req.metadata))

    with detect_graph.as_default():
        with tf.Session(graph=detect_graph) as sess:

            # reconstruct image from buffer
            data = np.fromstring(req.frame.data, dtype=np.uint8)
            image = Image.fromarray(
                data.reshape((req.frame.height, req.frame.width, 3)))

            if CONFIG.debug:
                image.show()

            # object recognition
            (boxes, classes, scores, num_detections) = orec_image(sess,
                                                                  detect_graph,
                                                                  image)
            objects = np.asarray(list(zip(boxes, scores, classes)))
            frames, mask = extract_box_from_image(image, objects,
                                                  CONFIG.model.threshold)
            # filter: only images above threshold
            objects = objects[mask]

            # make response
            msg = ObjectRecognitionResult()
            msg.header.id = str(uuid.uuid1())
            msg.header.timestamp = rospy.Time.now()
            msg.related_frame_id = msg.header.id

            for (box, score, cls), frame in zip(objects, frames):
                rec_object = RecognizedObject()

                rec_object.proximity = 0
                rec_object.label = label_map[cls]["name"]
                rec_object.confidence = score

                ymin, xmin, ymax, xmax = box_norm2abs(image, box)
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
    rospy.loginfo("Loading Object Recognition module...")

    # Download Model
    check_model(CONFIG.model.name)

    # Load a (frozen) Tensorflow model into memory.
    detect_graph = load_frozen_model(CONFIG.model.name)

    # Loading label map
    label_map = get_labels_map(CONFIG.model.labels,
                               CONFIG.model.classes)

    objectrec_server()
