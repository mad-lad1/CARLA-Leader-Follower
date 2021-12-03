import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
actors = world.get_actors().filter('vehicle.*')
for actor in actors:
    actor.destroy()
    