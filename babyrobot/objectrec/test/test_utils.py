import babyrobot.objectrec.utils as utils


def test_get_random_image():
    image = utils.get_random_image()
    assert image.width > 0 and image.height > 0


def test_capture_frame():
    frame = utils.capture_frame()
    assert frame.width > 0 and frame.height > 0
