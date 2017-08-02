import rospy
from PIL import Image

from babyrobot.objectrec.client import objectrec
from babyrobot.objectrec.config import OBJECTREC as CONFIG


def test_object_recognition_client_object_labels():
    """
    Test that for:
    * a given threshold
    * and a given model
    the model finds the correct objects
    :return:
    :rtype:
    """
    rospy.init_node('objectrec_client')

    model_name = 'faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017'
    CONFIG.debug = False
    CONFIG.model.threshold = 0.3
    CONFIG.model.name = model_name

    im_name = CONFIG.model.paths.images + "/Copy_of_objects-000.jpg"
    image = Image.open(im_name)

    recognized = objectrec(image)
    assert [r.label for r in recognized.objects] == ['person',
                                                     'couch',
                                                     'sports ball']


def test_object_recognition_client_object_threshold():
    """
    Test that changing the threshold affects the response
    :return:
    :rtype:
    """
    rospy.init_node('objectrec_client')

    model_name = 'faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017'
    CONFIG.debug = False
    CONFIG.model.threshold = 99.99
    CONFIG.model.name = model_name

    im_name = CONFIG.model.paths.images + "/Copy_of_objects-000.jpg"
    image = Image.open(im_name)

    recognized = objectrec(image)
    assert len(recognized.objects) == 0
