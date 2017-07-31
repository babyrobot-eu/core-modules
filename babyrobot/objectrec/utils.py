"""
Utility functions for object recognition, using Tensorflow API
"""
import glob
import os
import random
import tarfile

import numpy as np
import rospy
import six.moves.urllib as urllib
import tensorflow as tf
from PIL import Image
from matplotlib import pyplot as plt
from scipy.spatial.distance import euclidean
from tensorflow.models.object_detection.utils import \
    visualization_utils as vis_util
from tensorflow.models.object_detection.utils.label_map_util import \
    convert_label_map_to_categories, load_labelmap, create_category_index

from config import OBJECTREC as CONFIG


def check_model(model_name):
    """
    Checks if the given model exists on disk.
    If it doesn't, then downloads it
    :param model_name: the name of the model
    :return:
    """

    _model_file = model_name + '.tar.gz'
    _save_loc = os.path.join(CONFIG.model.paths.models, _model_file)
    _extract_loc = CONFIG.model.paths.models

    if not os.path.exists(os.path.join(_extract_loc, model_name)):
        rospy.loginfo("Downloading Model...", )
        opener = urllib.request.URLopener()
        opener.retrieve(CONFIG.model.repo + _model_file, _save_loc)
        tar_file = tarfile.open(_save_loc)
        for _file in tar_file.getmembers():
            file_name = os.path.basename(_file.name)
            if 'frozen_inference_graph.pb' in file_name:
                tar_file.extract(_file, _extract_loc)
        rospy.loginfo("done!")

        # remove the tar file
        os.remove(_save_loc)


def load_frozen_model(model_name):
    """
    Loads a pre-trained model from the disk
    and returns a TF Graph object
    :param model_name: the name of the model
    :return: TF Graph
    """
    _model_path = os.path.join(CONFIG.model.paths.models, model_name,
                               'frozen_inference_graph.pb')

    rospy.loginfo("Loading a (frozen) Tensorflow model into memory...", )
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(_model_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    rospy.loginfo("done!")

    return detection_graph


def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape(
        (im_height, im_width, 3)).astype(np.uint8)


def distance_box_to_center(box):
    """
    Computes the distance from the center of a box,
    from the center of the image.
    The distances are normalized
    :param box: (ymin, xmin, ymax, xmax) tuple
    :return: (float) distance in range (0,1).
    """
    # (im_width, im_height) = image.size
    ymin, xmin, ymax, xmax = box
    center_box = [(ymin + ymax) / 2, (xmin + xmax) / 2]
    dist = euclidean([0.5, 0.5], center_box)
    return dist


def get_random_image():
    """
    Returns a random image from '/images' dir
    :return:
    """
    _image_paths = glob.glob(CONFIG.model.paths.images + "/*.jpg")
    random.shuffle(_image_paths)
    image = Image.open(_image_paths[0])
    return image


def capture_frame():
    """
    Captures a frame from the camera and returns it as an Image
    * Note: At the moment, it returns a random image from the '/images' dir
    :return:
    """
    # todo: add code to read data from camera
    return get_random_image()


def get_labels_map(labels, n_classes):
    """
    Mapping from class indices to class labels
    :return:
    """
    _tf_models_dir = os.path.abspath(tf.models.object_detection.__file__) \
        .strip("__init__.py").strip("__init__.pyc")

    # List of the strings that is used to add correct label for each box.
    _labels_path = os.path.join(_tf_models_dir, 'data', labels)

    label_map = load_labelmap(_labels_path)
    categories = convert_label_map_to_categories(label_map,
                                                 max_num_classes=n_classes,
                                                 use_display_name=True)
    category_index = create_category_index(categories)
    return category_index


def extract_box_from_image(image, objects, threshold=None):
    """
    Given an image and a collection of recognized objects,
    it extracts the parts of the image, that correspond to each object

    If a threshold is passed, the it extracts only the frames
    for those objects that have higher score than the threshold.
    Else (threshold is None) it extracts frames for all the objects.

    :param image: a PIL image
    :param objects: list of recognized objects (box, score, class)
    :param threshold: (float) omit images with score below a threshold
    :return: (list of PIL.Image) cropped images for each object
    """

    # image.show()
    # width, height = image.size

    _images = []
    _mask = []

    for box, score, cls in objects:
        _m = False

        condition = (
            (threshold is not None and score > threshold)
            or (threshold is None)
        )
        if condition:
            _abs_box = box_norm2abs(image, box)
            frame = image.crop(_abs_box)
            _images.append(frame)
            _m = True

        _mask.append(_m)

    return _images, _mask


def box_norm2abs(image, box):
    """
    Get the bounding box of an image, in pixels
    :return:
    """
    width, height = image.size
    ymin, xmin, ymax, xmax = box
    _abs_box = np.asarray([ymin * height, xmin * width,
                           ymax * height, xmax * width])
    return _abs_box.astype(np.uint32)


def ros_box_to_norm_box(image, rbox):
    """
    Convert a ROS msg BoundingBox to a TF (normalized) box
    :param image:
    :param rbox:
    :return:
    """
    width, height = image.size
    box = [rbox.upper_left.y, rbox.upper_left.x,
           rbox.lower_right.y, rbox.lower_right.x]
    box = np.array(box).astype(np.float32)
    ymin, xmin, ymax, xmax = box

    _norm_box = np.asarray([ymin / height, xmin / width,
                            ymax / height, xmax / width])
    return _norm_box


def orec_image(sess, comp_graph, image):
    """
    Performs object recognition on an image and returns
    for each object, it's bounding box, confidence and class label
    :param sess: the current tf session
    :param comp_graph: the computational graph (model)
    :param image: the image (python Image)
    :return:
        - boxes: The coordinates of the each bounding box in boxes are encoded
            as [y_min, x_min, y_max, x_max].
            The bounding box coordinates are floats in [0.0, 1.0] relative
            to the width and height of the underlying image.
        - scores: the confidence of the prediction.
            list of floats in [0.0, 1.0].
        - classes
        - num_detections
    """

    # the array based representation of the image will be used later
    # in order to prepare the result image with boxes and labels on it.
    image_np = load_image_into_numpy_array(image)

    # Expand dimensions since the model expects images to have
    # shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)
    image_tensor = comp_graph.get_tensor_by_name('image_tensor:0')

    # Each box represents a part of the image
    # where a particular object was detected.
    boxes = comp_graph.get_tensor_by_name('detection_boxes:0')

    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    scores = comp_graph.get_tensor_by_name('detection_scores:0')
    classes = comp_graph.get_tensor_by_name('detection_classes:0')
    num_detections = comp_graph.get_tensor_by_name('num_detections:0')

    # Actual detection.
    (boxes, scores, classes, num_detections) = sess.run(
        [boxes, scores, classes, num_detections],
        feed_dict={image_tensor: image_np_expanded})

    results = (np.squeeze(boxes),
               np.squeeze(classes).astype(np.int32),
               np.squeeze(scores),
               np.squeeze(num_detections).astype(np.int32))

    return results


def visualize_objectrec_response(image, res, label_map, threshold):
    objects = res.recognized.objects

    boxes = []
    classes = []
    scores = []
    for o in objects:
        box = ros_box_to_norm_box(image, o.bounding_box)
        boxes.append(box)

        scores.append(o.confidence)
        cls_id = [k for (k, v) in label_map.items() if v['name'] == o.label][0]
        classes.append(cls_id)

    visualize_recognized_objects(image,
                                 np.asarray(boxes), classes, scores,
                                 label_map,
                                 threshold)


def visualize_recognized_objects(image, boxes, classes, scores,
                                 category_index,
                                 threshold):
    """
    Show an image with bounding boxes around the recognized objects,
    along with the confidence of the model for each object
    :param image:
    :param boxes:
    :param classes:
    :param scores:
    :param category_index:
    :param threshold:
    :return:
    """
    # Size, in inches, of the output images. For visualization
    IMAGE_SIZE = (12, 8)

    # w, h = image.size
    # my_dpi = 96  # todo: find this programmatically

    image_np = load_image_into_numpy_array(image)
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        boxes,
        classes,
        scores,
        category_index,
        use_normalized_coordinates=True,
        line_thickness=5,
        min_score_thresh=threshold)
    plt.figure(figsize=IMAGE_SIZE)
    # plt.figure(figsize=(w / my_dpi, h / my_dpi), dpi=my_dpi)
    plt.imshow(image_np)
    plt.show()


def debug_info_image(objects, category_index, threshold):
    """
    print debugging information, for the object recognition on an image
    :return:
    """
    for box, score, cls in objects:
        if score > threshold:
            rospy.loginfo("label:{}, score:{}, dist:{}".format(
                category_index[cls]["name"],
                score,
                distance_box_to_center(box)))
    rospy.loginfo("")
