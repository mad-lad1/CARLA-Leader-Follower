import carla
from time import sleep
from agents.navigation.controller import VehiclePIDController
import math 
#code to draw waypoints on a road
client = carla.Client('localhost', 2000)
client.set_timeout(20.0)
world = client.get_world()
def draw_waypoints(world, waypoints, road_id=None, life_time=50.0):

  for waypoint in waypoints:

    if(waypoint.road_id == road_id):
      world.debug.draw_string(waypoint.transform.location, '-', draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                   persistent_lines=True)
                                   
waypoints = world.get_map().generate_waypoints(distance=1.0)
draw_waypoints(world, waypoints, road_id=4, life_time=20)
print("Drew waypoints")

vehicle_blueprint = world.get_blueprint_library().filter('model3')[0]

filtered_waypoints = []
spawn_points = []
for waypoint in waypoints:
  if(waypoint.road_id == 2):
    filtered_waypoints.append(waypoint)


spawn_point1 = filtered_waypoints[100].transform
spawn_point1.rotation.yaw = 0
spawn_point1.location.z += 1
spawn_point1.location.x += 0.2

spawn_point2 = filtered_waypoints[120].transform
spawn_point2.rotation.yaw = 0
spawn_point2.location.z += 1
spawn_point2.location.x -= 1

spawn_point3 = filtered_waypoints[140].transform
spawn_point3.rotation.yaw = 0
spawn_point3.location.z += 1

vehicle1 = world.spawn_actor(vehicle_blueprint, spawn_point1)
vehicle2 = world.spawn_actor(vehicle_blueprint, spawn_point2)
vehicle3 = world.spawn_actor(vehicle_blueprint, spawn_point3)
# actor_list = world.get_actors()
# vehicle = actor_list.find(90)

print("Spawned 3 Tesla Model 2")

custom_controller = VehiclePIDController(vehicle1, args_lateral={'K_P': 0, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})

target_waypoint = filtered_waypoints[0]

client.get_world().debug.draw_string(target_waypoint.transform.location, 'O', draw_shadow=False,
                           color=carla.Color(r=255, g=0, b=0), life_time=20,
                           persistent_lines=True)

ticks_to_track = 5
for i in range(ticks_to_track):
	control_signal = custom_controller.run_step(1, target_waypoint)
	vehicle1.apply_control(control_signal)





while(math.sqrt((vehicle1.get_location().x - target_waypoint.transform.location.x) ** 2 + (vehicle1.get_location().y - target_waypoint.transform.location.y) ** 2) > 1):
  distance = math.sqrt((vehicle1.get_location().x - target_waypoint.transform.location.x) ** 2 + (vehicle1.get_location().y - target_waypoint.transform.location.y) ** 2)
  #print(distance)

stop_signal = carla.VehicleControl()
stop_signal.throttle = 0
stop_signal.brake = 2
vehicle1.apply_control(stop_signal)

sleep(3)


vehicle1.destroy()
vehicle2.destroy()
vehicle3.destroy()