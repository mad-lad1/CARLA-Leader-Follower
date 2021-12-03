import carla
from time import sleep
import numpy as np
from agents.navigation.local_planner import LocalPlanner
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
import matplotlib.pyplot as plt
import random
import time

def get_with_delay(attribute, packet_delay):
  sleep(packet_delay)
  return attribute


  
def send_with_rate(attribute, iterator, every, attribute_from_prev_step):
  if iterator % every == 0:
    return attribute
  else:
    return attribute_from_prev_step

def get_with_probability(attribute_from_prev_step, attribute, probability, i):
  random_number = random.random()
  
  if random_number > 1 - probability:
    print("Accepted")
    return send_with_rate(attribute, i, 1, attribute_from_prev_step), True
  else:
    print("Returned same value!")
    return send_with_rate(attribute_from_prev_step, i, 1, attribute_from_prev_step), False



def lateralController(waypoints,x,y,yaw,v):
  #Stanley Controller
  k_e = 4
  k_v = 8
  yaw_path = np.arctan2(waypoints[-1][0].transform.location.y-waypoints[0][0].transform.location.y,
                        waypoints[-1][0].transform.location.x-waypoints[0][0].transform.location.x)
  yaw_diff = yaw_path - yaw
  if yaw_diff > np.pi:
    yaw_diff -= 2*np.pi
  elif yaw_diff < -np.pi:
    yaw_diff += 2*np.pi
  current_xy = np.array([x,y])
  crosstrack_error = np.min(np.sum((current_xy -  np.array([waypoints[0][0].transform.location.x, waypoints[0][0].transform.location.y])**2)))
  yaw_cross_track = np.arctan2(y-waypoints[0][0].transform.location.y ,x-waypoints[0][0].transform.location.x)
  yaw_path2ct = yaw_path -yaw_cross_track
  
  if yaw_path2ct > np.pi:
    yaw_path2ct -= 2 *np.pi
  if yaw_path2ct < -np.pi:
    yaw_path2ct += 2* np.pi
  if yaw_path2ct > 0:
    crosstrack_error = abs(crosstrack_error)
  else:
    crosstrack_error = - abs(crosstrack_error)
  
  yaw_diff_crosstrack = np.arctan(k_e*crosstrack_error/(k_v+v))
  
  steer_expect = yaw_diff + yaw_diff_crosstrack
  if steer_expect > np.pi:
      steer_expect -= 2*np.pi
  if steer_expect < -np.pi:
      steer_expect += 2 *np.pi
  steer_expect = min(1.22,steer_expect)
  steer_expect = max(-1.22,steer_expect)

  #update
  return steer_expect




client = carla.Client('localhost', 2000)
client.set_timeout(20.0)




world = client.get_world()

waypoints = world.get_map().generate_waypoints(distance=1.0)
filtered_waypoints = []
spawn_points = []
for waypoint in waypoints:
  if(waypoint.road_id == 2):
    filtered_waypoints.append(waypoint)
    
    
vehicle_blueprint = world.get_blueprint_library().filter('model3')[0]
spawn_point1 = filtered_waypoints[100].transform
spawn_point1.rotation.yaw = 0
spawn_point1.location.z += 1
spawn_point1.location.x += 0.2


#code to draw waypoints on a road
client = carla.Client('localhost', 2000)
client.set_timeout(20.0)
world = client.get_world()
map = world.get_map()
sampling_resolution = 0.1



dao = GlobalRoutePlannerDAO(map, sampling_resolution)
grp = GlobalRoutePlanner(dao)
grp.setup()


                                   
waypoints = world.get_map().generate_waypoints(distance=1.0)

vehicle_blueprint = world.get_blueprint_library().filter('model3')[0]

filtered_waypoints = []
spawn_points = []
for waypoint in waypoints:
  if(waypoint.road_id == 2):
    filtered_waypoints.append(waypoint)


spawn_point1 = filtered_waypoints[10].transform
spawn_point1.rotation.yaw = 0
spawn_point1.location.z += 1
spawn_point1.location.x += 0.2

spawn_point2 = filtered_waypoints[50].transform
spawn_point2.rotation.yaw = 0
spawn_point2.location.z += 1
spawn_point2.location.x -= 1


vehicle1 = world.spawn_actor(vehicle_blueprint, spawn_point1)
vehicle2 = world.spawn_actor(vehicle_blueprint, spawn_point2)


#custom_controller = VehiclePIDController(vehicle2, args_lateral={'K_P': a0, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
custom_controller = LocalPlanner(vehicle2, opt_dict={
    #'lateral_control_dict' : {'K_P': 0, 'K_D': 0, 'K_I': 0, 'dt': 1/20.0}
})
#custom_controller._max_brake = 0.0
#compute path between vehicle and leader


vehicle1_velocities  = []
vehicle2_velocities = []


previous_location = vehicle1.get_location()
vehicle1.set_autopilot(True)
previous_velocity = vehicle1.get_velocity().x * 3.6
i = 0
print("Entering while loop")
while i < 350:
    time_start = time.perf_counter()
    goal, flag_pos = get_with_probability(previous_location, vehicle1.get_location(), 1, i) #get_location_with_delay(vehicle1, 2)
    
    start = vehicle2.get_location()
    
    
    #path = grp.trace_route(start, goal)
    
    custom_controller.set_global_plan(path)
    vehicle1_speed = vehicle1.get_velocity().x
    desired_speed, flag_speed = get_with_probability(previous_velocity, vehicle1_speed * 3.6, 1, i) 
    
    custom_controller.set_speed(desired_speed)
    custom_controller.target_waypoint = path[-1]
    output = custom_controller.run_step()
    
    
  
    # steering = lateralController(path, vehicle2.get_location().x, vehicle2.get_location().y, 
    #                             vehicle2.get_transform().rotation.yaw, 
    #                             vehicle2.get_velocity().x)
    
    control_input = carla.VehicleControl(
      throttle = output.throttle,
      steer = 0,
      brake = output.brake,
      hand_brake = output.hand_brake,
      manual_gear_shift = output.manual_gear_shift
    )
    
    # plt.plot(i, output.throttle, '-ok')
    # plt.pause(0.05)
    vehicle2.apply_control(control_input)
    
    vehicle1_velocities.append(vehicle1_speed)
    vehicle2_velocities.append(desired_speed)
    
    
    
    for w in path:
          world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
          color=carla.Color(r=255, g=0, b=0), life_time=0.2,
          persistent_lines=True)
    i = i + 1
    
    if flag_pos:
      previous_location = send_with_rate(vehicle1.get_location(), i,  1,  previous_location) #get_location_with_delay(vehicle1, 2)
    if flag_speed:
      previous_velocity = send_with_rate(vehicle1.get_velocity().x * 3.6,  i, 1, previous_velocity) 
    
    time_end = time.perf_counter()
    
    print(time_end - time_start)

plt.plot(list(range(i)), vehicle1_velocities, color='r', label='Leader Velocity')
plt.plot(list(range(i)), vehicle2_velocities, color='g', label='Follower Velocity')   
  
plt.xlabel("Time Steps")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.show()  
print("Done")




vehicle1.destroy()
vehicle2.destroy()


#except KeyboardInterrupt: 
    
