import os

import tensorflow as tf
from config import OBJECTREC as CONFIG
from matplotlib.image import imsave
from utils import load_frozen_model, check_model, orec_image, \
    debug_info_image, get_labels_map, \
    extract_box_from_image, get_images_from_dir, \
    downscale_image, pad_image, get_rec_objects

# Download Model
check_model(CONFIG.model.name)

# Load a (frozen) Tensorflow model into memory.
detect_graph = load_frozen_model(CONFIG.model.name)

# Loading label map
label_map = get_labels_map(CONFIG.model.labels,
                           CONFIG.model.classes)

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

            imsave("images/_test_results/" + filename + ".png", img)
            # IMAGE_SIZE = (12, 8)
            # fig = plt.figure(figsize=IMAGE_SIZE)
            # plt.figure(figsize=(w / my_dpi, h / my_dpi), dpi=my_dpi)
            # plt.imshow(img)
            # plt.show()
