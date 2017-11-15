import base64
from io import BytesIO
from PIL import Image
import numpy as np


def json_to_image(base64_image):
    # Decode image from base64
    image = Image.open(BytesIO(base64.b64decode(base64_image)))
    # Convert to PIL Image
    img = np.array(image)
    pil_img = Image.fromarray(img)

    return pil_img


def save_wav(base64_wav, save_dir):
    # Decode wav from base64
    wav = base64_wav.decode('base64')
    with open(save_dir, 'w') as output_file:
        output_file.write(wav)
