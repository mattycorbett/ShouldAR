import io
import socket
import time

import numpy as np
from PIL import Image

serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# address '0.0.0.0' or '' work to allow connections from other machines.  'localhost' disallows external connections.
# see https://www.raspberrypi.org/forums/viewtopic.php?t=62108
serv.bind(('10.0.0.40', 6464))
serv.listen(5)
print("Ready to accept 5 connections")


def create_image_from_bytes(image_bytes) -> Image.Image:
    stream = io.BytesIO(image_bytes)
    return Image.open(stream)


while True:
    conn, addr = serv.accept()
    start = time.time()


    while True:
        # print('waiting for data')
        # Try 4096 if unsure what buffer size to use. Large transfer chunk sizes (which require large buffers) can cause corrupted result
        bufferSize = 65000
        array_from_client = bytearray()
        totalData = 0
        size = conn.recv(4)
        sizeInt = int.from_bytes(size, "little")
        print(sizeInt)

        while totalData < sizeInt:
            if (bufferSize > (sizeInt - totalData)):
                bufferSize = sizeInt - totalData;
            data = conn.recv(bufferSize)
            totalData += len(data)
            array_from_client.extend(data)

        img: Image.Image = create_image_from_bytes(array_from_client)
        ms = time.time() * 1000.0
        img.save(str(ms) + ".jpg")
    conn.close()
    print('client disconnected')