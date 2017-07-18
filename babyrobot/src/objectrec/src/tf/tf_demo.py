import glob
import os

import tensorflow as tf
from PIL import Image
from tensorflow.models.object_detection.utils import label_map_util

from tf_utils import load_frozen_model, check_model, orec_image, \
    visualize_recognized_objects, debug_info_image

THRESHOLD = 0.2
VISUALIZE = True
NUM_CLASSES = 90
MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'

##############################################################################
# PATHS & FILENAMES
##############################################################################
TF_MODELS_DIR = os.path.abspath(tf.models.object_detection.__file__).strip("__init__.py").strip("__init__.pyc")
SCRIPT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))

MODEL_FILE = MODEL_NAME + '.tar.gz'
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = os.path.join(SCRIPT_DIR_PATH, 'models', MODEL_NAME, 'frozen_inference_graph.pb')

# List of the strings that is used to add correct label for each box.
# PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')
PATH_TO_LABELS = os.path.join(TF_MODELS_DIR, 'data', 'mscoco_label_map.pbtxt')

# Images
PATH_TO_TEST_IMAGES_DIR = os.path.join(SCRIPT_DIR_PATH, '..', 'images')
TEST_IMAGE_PATHS = glob.glob(PATH_TO_TEST_IMAGES_DIR + "/*.jpg")

# Download Model
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
SAVE_LOC = os.path.join(SCRIPT_DIR_PATH, 'models', MODEL_FILE)
EXTR_LOC = os.path.join(SCRIPT_DIR_PATH, 'models')
check_model(DOWNLOAD_BASE, SAVE_LOC, EXTR_LOC)

# Load a (frozen) Tensorflow model into memory.
detection_graph = load_frozen_model(PATH_TO_CKPT)

# Loading label map
# ---
# Label maps map indices to category names, so that when our convolution network predicts 5,
# we know that this corresponds to airplane. Here we use internal utility functions,
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map,
                                                            max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
category_index = label_map_util.create_category_index(categories)

with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        for image_path in TEST_IMAGE_PATHS:
            image = Image.open(image_path)

            # objects = orec_image(sess, detection_graph, image)
            (boxes, scores, classes, num_detections) = orec_image(sess, detection_graph, image)
            # boxes = [ymin, xmin, ymax, xmax]

            # print debugging information
            debug_info_image(image, boxes, scores, classes, category_index, THRESHOLD)

            # Visualization of the results of a detection.
            if VISUALIZE:
                visualize_recognized_objects(image, boxes, classes, scores, category_index, THRESHOLD)
