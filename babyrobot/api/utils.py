import base64
from io import BytesIO

import cStringIO
from PIL import Image
import numpy as np


def base64_to_pil(base64_image):
    # Decode image from base64
    image = Image.open(BytesIO(base64.b64decode(base64_image)))
    # Convert to PIL Image
    img = np.array(image)
    pil_img = Image.fromarray(img)
    return pil_img


def np_to_pil(np_image):
    return Image.fromarray(np_image.astype('uint8'), 'RGB')


def pil_to_base64(pil_image):
    buffer = cStringIO.StringIO()
    pil_image.save(buffer, format="JPEG")
    return base64.b64encode(buffer.getvalue())


def np_to_base64(np_image):
    pil_img = np_to_pil(np_image)
    return pil_to_base64(pil_img)


def save_wav(base64_wav, save_dir):
    # Decode wav from base64
    wav = base64_wav.decode('base64')
    with open(save_dir, 'w') as output_file:
        output_file.write(wav)
