import carla
from time import sleep
import numpy as np
from agents.navigation.local_planner import LocalPlanner
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
import matplotlib.pyplot as plt
import random
import time



every = 3
p = 0.8

def get_with_delay(attribute, packet_delay):
  sleep(packet_delay)
  return attribute


  
def send_with_rate(attribute, iterator, kil, attribute_from_prev_step):
  if iterator % kil == 0:
    return attribute
  else:
    return attribute_from_prev_step

def get_with_probability(attribute_from_prev_step, attribute, probability, i):
  random_number = random.random()
  
  if random_number > 1 - probability:
    return send_with_rate(attribute, i, every, attribute_from_prev_step), True
  else:
    
    return send_with_rate(attribute_from_prev_step, i, every, attribute_from_prev_step), False



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
sampling_resolution = 0.3



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


spawn_point3 = filtered_waypoints[90].transform
spawn_point3.rotation.yaw = 0
spawn_point3.location.z += 1
spawn_point3.location.x -= 1


spawn_point4 = filtered_waypoints[130].transform
spawn_point4.rotation.yaw = 0
spawn_point4.location.z += 1
spawn_point4.location.x -= 1


vehicle1 = world.spawn_actor(vehicle_blueprint, spawn_point1)
vehicle2 = world.spawn_actor(vehicle_blueprint, spawn_point2)
vehicle3 = world.spawn_actor(vehicle_blueprint, spawn_point3)
vehicle4 = world.spawn_actor(vehicle_blueprint, spawn_point4)

#custom_controller = VehiclePIDController(vehicle2, args_lateral={'K_P': a0, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})


#compute path between vehicle and leader
custom_controller2 = LocalPlanner(vehicle2)
custom_controller3 = LocalPlanner(vehicle3)
custom_controller4 = LocalPlanner(vehicle4)


vehicle1_velocities  = []
vehicle2_velocities = []
vehicle3_velocities = []
vehicle4_velocities = []

previous_location2 = vehicle1.get_location()
previous_location3 = vehicle2.get_location()
previous_location4 = vehicle3.get_location()
vehicle1.set_autopilot(True)


previous_velocity2 = vehicle1.get_velocity().x * 3.6
previous_velocity3 = vehicle2.get_velocity().x * 3.6
previous_velocity4 = vehicle3.get_velocity().x * 3.6

i = 0
print("Entering while loop")
while i < 300:
    time_start = time.perf_counter()
    
    goal2, flag_pos2 = get_with_probability(previous_location2, vehicle1.get_location(), p, i) #get_location_with_delay(vehicle1, 2)
    start2 = vehicle2.get_location()
    
    goal3, flag_pos3 = get_with_probability(previous_location3, vehicle2.get_location(), p, i) #get_location_with_delay(vehicle1, 2)
    start3 = vehicle3.get_location()
    
    goal4, flag_pos4 = get_with_probability(previous_location4, vehicle3.get_location(), p, i) #get_location_with_delay(vehicle1, 2)
    start4 = vehicle4.get_location()
    
    
    path2 = grp.trace_route(start2, goal2)
    path3 = grp.trace_route(start3, goal3)
    path4 = grp.trace_route(start4, goal4)
    
    custom_controller2.set_global_plan(path2)
    custom_controller3.set_global_plan(path3)
    custom_controller4.set_global_plan(path4)
    
    vehicle1_speed = vehicle1.get_velocity().x
    desired_speed2, flag_speed2 = get_with_probability(previous_velocity2, vehicle1_speed * 3.6, p, i) 
    
    vehicle2_speed = vehicle2.get_velocity().x
    desired_speed3, flag_speed3 = get_with_probability(previous_velocity3, vehicle2_speed * 3.6, p, i) 
    
    vehicle3_speed = vehicle3.get_velocity().x
    desired_speed4, flag_speed4 = get_with_probability(previous_velocity4, vehicle3_speed * 3.6, p, i) 
    
    
    
    
    custom_controller2.set_speed(desired_speed2)
    custom_controller2.target_waypoint = path2[-1]
    output2 = custom_controller2.run_step()
    
    
    custom_controller3.set_speed(desired_speed3)
    custom_controller3.target_waypoint = path3[-1]
    output3 = custom_controller3.run_step()
    
    custom_controller4.set_speed(desired_speed4)
    custom_controller4.target_waypoint = path4[-1]
    output4 = custom_controller4.run_step()
    
    
    
    
  
    # steering = lateralController(path, vehicle2.get_location().x, vehicle2.get_location().y, 
    #                             vehicle2.get_transform().rotation.yaw, 
    #                             vehicle2.get_velocity().x)
    
    control_input2 = carla.VehicleControl(
      throttle = output2.throttle,
      steer = 0,
      brake = output2.brake,
      hand_brake = output2.hand_brake,
      manual_gear_shift = output2.manual_gear_shift
    )
    
    
    control_input3 = carla.VehicleControl(
      throttle = output3.throttle,
      steer = 0,
      brake = output3.brake,
      hand_brake = output3.hand_brake,
      manual_gear_shift = output3.manual_gear_shift
    )
    
    
    
    control_input4 = carla.VehicleControl(
      throttle = output4.throttle,
      steer = 0,
      brake = output4.brake,
      hand_brake = output4.hand_brake,
      manual_gear_shift = output4.manual_gear_shift
    )
    
    
    
    
    
    # plt.plot(i, output.throttle, '-ok')
    # plt.pause(0.05)
    vehicle2.apply_control(control_input2)
    vehicle3.apply_control(control_input3)
    vehicle4.apply_control(control_input4)
    
    
    
    vehicle1_velocities.append(vehicle1_speed * 3.6)
    vehicle2_velocities.append(desired_speed2)
    vehicle3_velocities.append(desired_speed3)
    vehicle4_velocities.append(desired_speed4)
    
    
    for w in path2:
          world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
          color=carla.Color(r=255, g=0, b=0), life_time=0.2,
          persistent_lines=True)
    for w in path3:
          world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
          color=carla.Color(r=255, g=0, b=0), life_time=0.2,
          persistent_lines=True)
    for w in path4:
          world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
          color=carla.Color(r=255, g=0, b=0), life_time=0.2,
          persistent_lines=True)
    
    
    if flag_pos2:
      previous_location2 = send_with_rate(vehicle1.get_location(), i,  every,  previous_location2) #get_location_with_delay(vehicle1, 2)
    if flag_speed2:
      previous_velocity2 = send_with_rate(vehicle1.get_velocity().x * 3.6,  i, every, previous_velocity3) 
    
    if flag_pos3:
      previous_location3 = send_with_rate(vehicle2.get_location(), i,  every,  previous_location3) #get_location_with_delay(vehicle1, 2)
    if flag_speed3:
      previous_velocity3 = send_with_rate(vehicle2.get_velocity().x * 3.6,  i, every, previous_velocity3) 
      
    if flag_pos4:
      previous_location4 = send_with_rate(vehicle3.get_location(), i,  every,  previous_location3) #get_location_with_delay(vehicle1, 2)
    if flag_speed4:
      previous_velocity4 = send_with_rate(vehicle3.get_velocity().x * 3.6,  i, every, previous_velocity4) 
    
    
    i = i + 1
    
    
    
    time_end = time.perf_counter()
    
    time_difference = time_end - time_start



plt.plot(list(range(i)), vehicle1_velocities, color='r', label='Leader Velocity')
plt.plot(list(range(i)), vehicle2_velocities, color='g', label='Follower 1 Velocity')
plt.plot(list(range(i)), vehicle3_velocities, color='b', label='Follower 2 Velocity')   
plt.plot(list(range(i)), vehicle4_velocities, color='c', label='Follower 3 Velocity')   
plt.title(f"Velocity Profile with Loss Probability: {p} - Message Sent every {every} iterations")#{(1.0/time_difference*every):.2f} Communication Rate")
plt.xlabel("Time Steps")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.show()  
print("Done")




vehicle1.destroy()
vehicle2.destroy()
vehicle3.destroy()



#except KeyboardInterrupt: 
    
