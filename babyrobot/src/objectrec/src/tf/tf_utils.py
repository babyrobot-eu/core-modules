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

from tensorflow.models.object_detection.utils import visualization_utils as vis_util


def check_model(url, save_loc, extract_loc):
    if not os.path.isfile(save_loc):
        print("Downloading Model...")
        opener = urllib.request.URLopener()
        opener.retrieve(url)
        tar_file = tarfile.open(save_loc)
        for file in tar_file.getmembers():
            file_name = os.path.basename(file.name)
            if 'frozen_inference_graph.pb' in file_name:
                tar_file.extract(file, extract_loc)
        print("done!")


def load_frozen_model(model_path):
    print("Loading a (frozen) Tensorflow model into memory...")
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(model_path, 'rb') as fid:
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
    pass


def orec_image(sess, comp_graph, image):
    """
    Performs object recognition on an image and returns
    for each object, it's bounding box, confidence and class label
    :param sess: the current tf session
    :param comp_graph: the computational graph (model)
    :param image: the image (python Image)
    :return: boxes, scores, classes, num_detections
    """

    # the array based representation of the image will be used later in order to prepare the
    # result image with boxes and labels on it.
    image_np = load_image_into_numpy_array(image)
    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)
    image_tensor = comp_graph.get_tensor_by_name('image_tensor:0')
    # Each box represents a part of the image where a particular object was detected.
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


def visualize_recognized_objects(image, boxes, classes, scores, category_index, threshold):
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

    w, h = image.size
    my_dpi = 96  # todo: find this programmatically

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
    for x, y, b in list(zip(np.squeeze(scores), np.squeeze(classes).astype(np.int32), np.squeeze(boxes))):
        # for x, y, b in objects:
        if x > threshold:
            print(category_index[y]["name"], x, distance_from_center(b, image))
    print()
