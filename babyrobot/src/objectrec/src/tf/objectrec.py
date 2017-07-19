import glob
import os

import tensorflow as tf
from PIL import Image

from babyrobot.lib.utils import yaml2dict
from utils import load_frozen_model, check_model, orec_image, \
    visualize_recognized_objects, debug_info_image, get_labels_map

config = yaml2dict("config.yaml")

# Images
SCRIPT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))
PATH_TO_TEST_IMAGES_DIR = os.path.join(SCRIPT_DIR_PATH, '..', 'images')
TEST_IMAGE_PATHS = glob.glob(PATH_TO_TEST_IMAGES_DIR + "/*.jpg")

# Download Model
check_model(config["model"]["name"])

# Load a (frozen) Tensorflow model into memory.
detect_graph = load_frozen_model(config["model"]["name"])

# Loading label map
label_map = get_labels_map(config["model"]["labels"],
                           config["model"]["classes"])

with detect_graph.as_default():
    with tf.Session(graph=detect_graph) as sess:
        for image_path in TEST_IMAGE_PATHS:
            image = Image.open(image_path)

            # objects = orec_image(sess, detection_graph, image)
            (boxes, scores, classes, num_detections) = orec_image(sess,
                                                                  detect_graph,
                                                                  image)
            # boxes = [ymin, xmin, ymax, xmax]

            # print debugging information
            debug_info_image(image, boxes, scores, classes, label_map,
                             config["model"]["threshold"])

            # Visualization of the results of a detection.
            if config["model"]["visualize"]:
                visualize_recognized_objects(image,
                                             boxes,
                                             classes,
                                             scores,
                                             label_map,
                                             config["model"]["threshold"])
