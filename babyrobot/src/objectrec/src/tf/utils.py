"""
Utility functions for object recognition, using Tensorflow API
"""
import os
import tarfile

from matplotlib import pyplot as plt
import numpy as np
import six.moves.urllib as urllib
import tensorflow as tf
from scipy.spatial.distance import euclidean
from tensorflow.models.object_detection.utils.label_map_util import \
    convert_label_map_to_categories, load_labelmap, create_category_index

from tensorflow.models.object_detection.utils import \
    visualization_utils as vis_util


def check_model(model_name):
    """
    Check if the given model exists on disk.
    If it doesn't, then download it
    :param model_name:
    :return:
    """
    _model_file = model_name + '.tar.gz'
    _url_base = 'http://download.tensorflow.org/models/object_detection/'
    _script_loc = os.path.dirname(os.path.abspath(__file__))
    _save_loc = os.path.join(_script_loc, 'models', _model_file)
    _extract_loc = os.path.join(_script_loc, 'models')

    if not os.path.isfile(_save_loc):
        print("Downloading Model...")
        opener = urllib.request.URLopener()
        opener.retrieve(_url_base + _model_file, _save_loc)
        tar_file = tarfile.open(_save_loc)
        for _file in tar_file.getmembers():
            file_name = os.path.basename(_file.name)
            if 'frozen_inference_graph.pb' in file_name:
                tar_file.extract(_file, _extract_loc)
        print("done!")


def load_frozen_model(model_name):
    """
    Loads a pre-trained model from the disk
    and returns a TF Graph object
    :param model_name:
    :return:
    """
    _script_loc = os.path.dirname(os.path.abspath(__file__))
    _model_path = os.path.join(_script_loc, 'models', model_name,
                               'frozen_inference_graph.pb')

    print("Loading a (frozen) Tensorflow model into memory...")
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(_model_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    print("done!")

    return detection_graph


def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape(
        (im_height, im_width, 3)).astype(np.uint8)


def distance_from_center(box, image):
    """
    Computes the distance from the center of a box,
    from the center of the image.
    The distances are normalized
    :param box: (ymin, xmin, ymax, xmax)
    :param image:
    :return: distance in range (0,1).
    """
    (im_width, im_height) = image.size
    ymin, xmin, ymax, xmax = box
    center_box = [(ymin + ymax) / 2, (xmin + xmax) / 2]
    dist = euclidean([0.5, 0.5], center_box)
    return dist


def capture_frame():
    """
    Captures a frame from the camera and returns it as an Image
    * Note:
    :return:
    """

    pass


def get_labels_map(labels, n_classes):
    """
    Label maps map indices to category names,
    so that when our convolution network predicts 5,
    we know that this corresponds to airplane.
    Here we use internal utility functions,
    but anything that returns a dictionary mapping integers
    to appropriate string labels would be fine
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


def orec_image(sess, comp_graph, image):
    """
    Performs object recognition on an image and returns
    for each object, it's bounding box, confidence and class label
    :param sess: the current tf session
    :param comp_graph: the computational graph (model)
    :param image: the image (python Image)
    :return: boxes, scores, classes, num_detections
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

    # results = []
    # for box, score, cls in zip(boxes, scores, classes):
    #     obj = {
    #         "box": box,
    #         "score": score,
    #         "label": cls
    #     }
    #     results.append(obj)
    #
    # return results

    return boxes, scores, classes, num_detections


def visualize_recognized_objects(image, boxes, classes, scores, category_index,
                                 threshold):
    """

    :param image:
    :param boxes:
    :param classes:
    :param scores:
    :param category_index:
    :return:
    """
    # Size, in inches, of the output images. For visualization
    IMAGE_SIZE = (12, 8)

    # w, h = image.size
    # my_dpi = 96  # todo: find this programmatically

    image_np = load_image_into_numpy_array(image)
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        np.squeeze(boxes),
        np.squeeze(classes).astype(np.int32),
        np.squeeze(scores),
        category_index,
        use_normalized_coordinates=True,
        line_thickness=10,
        min_score_thresh=threshold)
    plt.figure(figsize=IMAGE_SIZE)
    # plt.figure(figsize=(w / my_dpi, h / my_dpi), dpi=my_dpi)
    plt.imshow(image_np)
    plt.show()


def debug_info_image(image, boxes, scores, classes, category_index, threshold):
    """
    print debugging information, for the object recognition on an image
    :return:
    """
    # print("--------------------")
    # print(image_path)
    # print("--------------------")
    for x, y, b in list(zip(np.squeeze(scores),
                            np.squeeze(classes).astype(np.int32),
                            np.squeeze(boxes))):
        # for x, y, b in objects:
        if x > threshold:
            print(category_index[y]["name"], x, distance_from_center(b, image))
    print()
