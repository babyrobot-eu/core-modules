import rospy
from PIL import Image

from babyrobot.objectrec.client import objectrec
from babyrobot.objectrec.config import OBJECTREC as CONFIG


def test_object_recognition_client_object_labels():
    """
    Test the labels of the top (highest confidence) 10 objects

    Note: Ensure that objectrec_srv is running!

    :return:
    :rtype:
    """
    rospy.init_node('objectrec_client')

    model_name = 'faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017'
    CONFIG.debug = False
    CONFIG.model.name = model_name

    im_name = CONFIG.model.paths.images + "/Copy_of_objects-000.jpg"
    image = Image.open(im_name)

    recognized = objectrec(image)

    top_10 = ['person', 'couch', 'sports ball', 'bed', 'couch', 'teddy bear',
              'person', 'banana', 'dining table', 'apple']
    assert [r.label for r in recognized.objects][:10] == top_10


def test_object_recognition_client_object_scores():
    """
    Test the scores of the top (highest confidence) 10 objects

    Note: Ensure that objectrec_srv is running!

    :return:
    :rtype:
    """
    rospy.init_node('objectrec_client')

    model_name = 'faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017'
    CONFIG.debug = False
    CONFIG.model.name = model_name

    im_name = CONFIG.model.paths.images + "/Copy_of_objects-000.jpg"
    image = Image.open(im_name)

    recognized = objectrec(image)

    top_10 = [0.57, 0.49, 0.35, 0.29, 0.21, 0.14, 0.13, 0.1, 0.09, 0.08]
    assert [round(r.confidence, 2)
            for r in recognized.objects][:10] == top_10
