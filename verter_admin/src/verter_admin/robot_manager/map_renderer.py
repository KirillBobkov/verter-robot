import numpy as np
from PIL import Image

import os


class MapRenderer:

    def __init__(self, output_path: str,):
        self.output_path = output_path

    def update(self, msg):
        width = msg.info.width
        height = msg.info.height

        occupancy = np.asarray(msg.data, dtype=np.int16,).reshape(height, width)

        image = np.full((height, width), 205, dtype=np.uint8)

        image[occupancy == 0] = 254

        image[occupancy >= 65] = 0

        image = np.flipud(image)

        result = Image.fromarray(image, mode="L")

        temporary = (self.output_path + ".tmp")

        result.save(temporary, format="PNG")

        os.replace(temporary, self.output_path,)