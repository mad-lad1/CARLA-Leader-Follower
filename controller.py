import cutils
import numpy as np

#not used in this project. Attempt to use Model Predictive Control
class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist &lt; min_dist:
                min_dist = dist
                min_idx = i
        if min_idx &lt; len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous',0.0)
        self.vars.create_var('throttle_previous',0.0)
        self.vars.create_var('int_val',0.0)
        self.vars.create_var('last_error',0.0)


        #Assigning gains for PID Controller.
        kp = 1
        ki = 1
        kd = 0.01


        # Skip the first frame to store previous values properly
        if self._start_control_loop:
    
            # Change these outputs with the longitudinal controller.
            throttle_output = 0
            brake_output    = 0

            #pid control
            st = t - self.vars.t_previous

            #error term
            delta_v = v_desired - v

            #Intergral
            integral = self.vars.int_val + delta_v *st

            #Derivative
            derivative = (delta_v-self.vars.last_error)/st

            #total

            rst = kp * delta_v + ki * integral + kd * derivative

            if rst &gt; 0 :
                throttle_output = np.tanh(rst)
                throttle_output = max(0.0,min(1.0,throttle_output))
                if throttle_output - self.vars.throttle_previous &gt; 0.1 :
                    throttle_output = self.vars.throttle_previous +0.1
            else :
                throttle_output = 0

            # Change the steer output with the lateral controller.
            steer_output    = 0

            #Use of stanley controller as lateral control
            #specify stanley parameters
            k_e = 0.3
            k_v = 10

            #calculate heading error
            yaw_path = np.arctan2(waypoints[-1][1]-waypoints[0][1],waypoints[-1][0]-waypoints[0][0])
            yaw_diff = yaw_path - yaw
            if yaw_diff &gt; np.pi :
                yaw_diff -= 2*np.pi
            if yaw_diff &lt; -np.pi :
                yaw_diff += 2*np.pi
            #calculate cross track error
            current_xy = np.array([x,y])
            crosstrack_error = np.min(np.sum((current_xy - np.array(waypoints)[:, :2])**2,axis =1))

            yaw_cross_track = np.arctan2(y-waypoints[0][1],x-waypoints[0][0])
            yaw_path2ct = yaw_path -yaw_cross_track
            if yaw_path2ct &gt; np.pi:
                yaw_path2ct -= 2 *np.pi
            if yaw_path2ct &lt; -np.pi :
                yaw_path2ct += 2* np.pi
            if yaw_path2ct &gt; 0 :
                crosstrack_error = abs(crosstrack_error)
            else:
                crosstrack_error = - abs(crosstrack_error)

            yaw_diff_crosstrack = np.arctan(k_e*crosstrack_error/(k_v+v))

            print(crosstrack_error,yaw_diff,yaw_diff_crosstrack)

            #Control law
            steer_expect = yaw_diff + yaw_diff_crosstrack
            if steer_expect &gt; np.pi:
                steer_expect -= 2*np.pi
            if steer_expect &lt; -np.pi:
                steer_expect += 2 *np.pi
            steer_expect = min(1.22,steer_expect)
            steer_expect = max(-1.22,steer_expect)

            #update
            steer_output = steer_expect



            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        
        
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
        self.vars.int_val = integral
        self.vars.throttle_previous = throttle_output
