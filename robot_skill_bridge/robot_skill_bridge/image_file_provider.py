"""Proveedor de imágenes grabadas y codificación compartida de imágenes RGB."""

import time
from io import BytesIO
from pathlib import Path

from PIL import Image

from .perception import Observation


def png_bytes(image, max_dimension):
    image = image.convert('RGB')
    image.thumbnail((max_dimension, max_dimension))
    output = BytesIO()
    image.save(output, format='PNG')
    return output.getvalue()


class ImageFileProvider:
    def __init__(self, path, max_dimension=640):
        self.path = Path(path)
        self.max_dimension = max_dimension

    def observe(self):
        with Image.open(self.path) as image:
            data = png_bytes(image, self.max_dimension)
        # captured_at es la adquisición por el proveedor, no la fecha de la foto.
        return Observation(data, 'image/png', time.time(), 'image_file:' + self.path.name)
