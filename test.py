import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO


client = carla.Client('localhost', 2000)
client.set_timeout(10)

world = client.load_world('Town01')
map = world.get_map()
sampling_resolution = 2

dao = GlobalRoutePlannerDAO(map, sampling_resolution)
grp = GlobalRoutePlanner(dao)
grp.setup()


spawn_points = world.get_map().get_spawn_points()

a = carla.Location(spawn_points[50].location)
b = carla.Location(spawn_points[100].location)

w1 = grp.trace_route(a, b) 

for w in w1:
    world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
    color=carla.Color(r=255, g=0, b=0), life_time=120.0,
    persistent_lines=True)