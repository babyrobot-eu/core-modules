import tensorflow as tf

from babyrobot.lib.utils import yaml2dict
from utils import load_frozen_model, check_model, orec_image, \
    visualize_recognized_objects, debug_info_image, get_labels_map, \
    capture_frame, extract_box_from_image

config = yaml2dict("config.yaml")

# Download Model
check_model(config["model"]["name"])

# Load a (frozen) Tensorflow model into memory.
detect_graph = load_frozen_model(config["model"]["name"])

# Loading label map
label_map = get_labels_map(config["model"]["labels"],
                           config["model"]["classes"])

with detect_graph.as_default():
    with tf.Session(graph=detect_graph) as sess:
        image = capture_frame()

        (boxes, classes, scores, num_detections) = orec_image(sess,
                                                              detect_graph,
                                                              image)

        objects = list(zip(boxes, scores, classes))

        # extract object image frames
        frames = extract_box_from_image(image, objects,
                                        config["model"]["threshold"])

        # print debugging information
        debug_info_image(image, objects, label_map,
                         config["model"]["threshold"])

        # Visualization of the results of a detection.
        if config["model"]["visualize"]:
            visualize_recognized_objects(image,
                                         boxes, classes, scores,
                                         label_map,
                                         config["model"]["threshold"])
