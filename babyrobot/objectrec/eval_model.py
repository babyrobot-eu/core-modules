import os

import tensorflow as tf
from PIL import ImageOps
from config import OBJECTREC as CONFIG
from matplotlib.image import imsave
from tensorflow.models.object_detection.utils import (
    visualization_utils as vis_util
)
from utils import load_frozen_model, check_model, orec_image, \
    debug_info_image, get_labels_map, \
    extract_box_from_image, get_images_from_dir, \
    load_image_into_numpy_array

# Download Model
check_model(CONFIG.model.name)

# Load a (frozen) Tensorflow model into memory.
detect_graph = load_frozen_model(CONFIG.model.name)

# Loading label map
label_map = get_labels_map(CONFIG.model.labels,
                           CONFIG.model.classes)


def get_rec_objects(
        image, boxes, classes, scores, category_index, threshold):
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
    return image_np


def downscale_image(image, baseWidth=800):
    # Calculate the height using the same aspect ratio
    widthPercent = (baseWidth / float(image.size[0]))
    height = int((float(image.size[1]) * float(widthPercent)))
    image = image.resize((baseWidth, height))
    return image


def pad_image(image):
    # Calculate the height using the same aspect ratio
    (im_width, im_height) = image.size
    border = int(im_width * 0.1)
    image = ImageOps.expand(image, border=border, fill='white')
    return image


print("Processing images...")
with detect_graph.as_default():
    with tf.Session(graph=detect_graph) as sess:
        # capture image
        for image in get_images_from_dir("images/_test_images"):
            head, tail = os.path.split(image.filename)
            filename, ext = os.path.splitext(tail)
            print("processing image {} ...".format(tail))
            image = downscale_image(image)
            image = pad_image(image)

            (boxes, classes, scores, num_detections) = orec_image(sess,
                                                                  detect_graph,
                                                                  image)

            objects = list(zip(boxes, scores, classes))

            # extract object image frames
            frames, mask = extract_box_from_image(image, objects,
                                                  CONFIG.model.threshold)

            # print debugging information
            debug_info_image(objects, label_map, CONFIG.model.threshold)

            # Visualization of the results of a detection.
            img = get_rec_objects(image,
                                  boxes, classes, scores,
                                  label_map,
                                  CONFIG.model.threshold)
            # IMAGE_SIZE = (12, 8)
            # fig = plt.figure(figsize=IMAGE_SIZE)
            # fig.savefig("images/_test_results/" + filename + ".png")
            imsave("images/_test_results/" + filename + ".png",
                   img, dpi=100)
            # plt.figure(figsize=(w / my_dpi, h / my_dpi), dpi=my_dpi)
            # plt.imshow(img)
            # plt.show()
