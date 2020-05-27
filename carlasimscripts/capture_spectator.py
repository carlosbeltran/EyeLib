
import glob
import sys
import os
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
from carla import ColorConverter as cc
import numpy as np

def process_img(image):
    image.convert(cc.Raw)
    image.save_to_disk('_out/%08d' % image.frame_number)

client    = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)

world        = client.get_world()
spectator = world.get_spectator()

bp_library = world.get_blueprint_library()
camera_bp = bp_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '576')
camera_bp.set_attribute('image_size_y', '432')
camera_bp.set_attribute('fov', '71')
camera = world.spawn_actor(camera_bp, spectator.get_transform())
ImageSizeX = 576;
ImageSizeY = 432;
cameraFOV = camera_bp.get_attribute('fov')
print(cameraFOV)
Focal_length = ImageSizeX /(2 * np.tan(71 * np.pi / 360))
Center_X = ImageSizeX / 2
Center_Y = ImageSizeY / 2
k = np.identity(3)
k[0, 2] = Center_X;
k[1, 2] = Center_Y;
k[0, 0] = k[1, 1] = Focal_length; 
print(k)
camera.listen(process_img)

counter = 0

while True:
    world.wait_for_tick()
    counter = counter + 1
    if counter == 10:
        counter = 0
        print(spectator.get_transform())
        print(camera.focal_distance)
        print(camera.K)
        exit()
