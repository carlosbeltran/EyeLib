
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
#from carla import ColorConverter as cc

#def process_img(image):
#    image.convert(cc.Raw)
#    image.save_to_disk('_out/%08d' % image.frame_number)

#camera
T_camera = carla.Transform(carla.Location(x=65.384514, y=-56.747856, z=12), carla.Rotation(pitch=-30, yaw=151, roll=0))
#walker
T_walker1 = carla.Transform(carla.Location(x=58.883602, y=-50.814964, z=8.829111), carla.Rotation(pitch=-13.618502, yaw=-26.884417, roll=0.000033))
T_walker2 = carla.Transform(carla.Location(x=63.062401, y=-54.493992, z=8.822564), carla.Rotation(pitch=-6.612490, yaw=-20.266428, roll=0.000036))
T_walker3 = carla.Transform(carla.Location(x=53.883602, y=-51.814964, z=8.829111), carla.Rotation(pitch=-13.618502, yaw=-28.884417, roll=0.000033))
T_walker4 = carla.Transform(carla.Location(x=56.062401, y=-55.493992, z=8.822564), carla.Rotation(pitch=-6.612490, yaw=-25.266428, roll=0.000036))
T_walker5 = carla.Transform(carla.Location(x=61.062401, y=-50.493992, z=8.822564), carla.Rotation(pitch=-6.612490, yaw=-5.266428, roll=0.000036))

client    = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)

world        = client.get_world()

blueprint    = random.choice(world.get_blueprint_library().filter('walker.*'))
mypedestrian = world.try_spawn_actor(blueprint, T_walker1)

blueprint    = random.choice(world.get_blueprint_library().filter('walker.*'))
mypedestrian = world.try_spawn_actor(blueprint, T_walker2)

blueprint    = random.choice(world.get_blueprint_library().filter('walker.*'))
mypedestrian = world.try_spawn_actor(blueprint, T_walker3)

blueprint    = random.choice(world.get_blueprint_library().filter('walker.*'))
mypedestrian = world.try_spawn_actor(blueprint, T_walker4)

blueprint    = random.choice(world.get_blueprint_library().filter('walker.*'))
mypedestrian = world.try_spawn_actor(blueprint, T_walker5)

spawn_points = world.get_map().get_spawn_points()
spawn_point  = random.choice(spawn_points) if spawn_points else carla.Transform()
mypedestrian = world.try_spawn_actor(blueprint, spawn_point)

spectator = world.get_spectator()
#transform = mypedestrian.get_transform()
#spectator.set_transform(carla.Transform(transform.location + carla.Location(z=5),
spectator.set_transform(T_camera)
counter = 0

#bp_library = world.get_blueprint_library()
#camera_bp = bp_library.find('sensor.camera.rgb')
#camera_bp.set_attribute('image_size_x', '640')
#camera_bp.set_attribute('image_size_y', '480')

#camera = world.spawn_actor(camera_bp, spectator.get_transform())
#camera.listen(process_img)

while True:
    world.wait_for_tick()
    counter = counter + 1
    if counter == 10:
        counter = 0
        print(spectator.get_transform())

#control = carla.WalkerBoneControl()
#first_tuple = ('crl_hand__R', carla.Transform(rotation=carla.Rotation(roll=90)))
#second_tuple = ('crl_hand__L', carla.Transform(rotation=carla.Rotation(roll=90)))
#control.bone_transforms = [first_tuple, second_tuple]
#world.player.apply_control(control)

