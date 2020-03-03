#!/usr/bin/env python3

import cutils
import numpy as np

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
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
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

        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################

        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('int_acc', 0)
        self.vars.create_var('prev_error', 0)
        self.vars.create_var('prev_heading', 0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:

            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################

            kp = 1.4
            kd = 0.1
            ki = 0.4
            curr_error = v_desired - v
            proportional = kp * curr_error
            integral = self.vars.int_acc + ki *curr_error *(t - self.vars.t_previous)
            derivative = kd * (curr_error - self.vars.prev_error) / (t - self.vars.t_previous)
            throttle_output = proportional + integral + derivative
            # brake_output    = 0

            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            front_distance = 1.5
            L1 = front_distance*2

            Kdd = 1.2

            alpha = np.arctan2(waypoints[len(waypoints)-1][1]-y-front_distance*np.sin(yaw),
                               waypoints[len(waypoints)-1][0]-x-front_distance*np.cos(yaw))
            alpha_input = alpha - yaw
            print('my alpha', alpha_input)
            steer_output = np.arctan(2*L1*np.sin(alpha_input)/(Kdd*v))
            print('my steer', steer_output)

            K_dd = 1 # Look ahead gain
            L = 3.0 # Distance between rear and front axe


            track_point_x = waypoints[len(self._waypoints) - 1][0]
            track_point_y = waypoints[len(self._waypoints) - 1][1]

            rear_x = x - L / 2 * np.cos(yaw)
            rear_y = y - L / 2 * np.sin(yaw)

            alpha = np.arctan2((track_point_y - rear_y), (track_point_x - rear_x)) - yaw
            print('his alpha', alpha)

            # Change the steer output with the lateral controller.
            steer_output1 = np.arctan((2 * L * np.sin(alpha)) / (K_dd * v))
            print('his steer', steer_output1)

            # error_gain = 0.2
            # softening_constant = 15
            # steer_output = 0
            # #
            # desired_heading = np.arctan2(waypoints[len(waypoints)-1][1]-waypoints[0][1],
            #                              waypoints[len(waypoints)-1][0]-waypoints[0][0])
            # prev_heading = np.arctan2(self._current_y, self._current_x)
            # current_heading = desired_heading - yaw
            # current_heading = (-2*np.pi + current_heading) if current_heading > np.pi else current_heading
            # current_heading = (2*np.pi + current_heading) if current_heading < - np.pi else current_heading
            #
            # crosstrack_error = np.sqrt(np.min((x- np.array(waypoints)[:, 0])**2 + (y- np.array(waypoints)[:, 1])**2))
            # # print('my crosstrack', crosstrack_error1)
            #
            # # yaw_error = desired_heading - np.arctan2(y-waypoints[-1][1],
            # #                                          x - waypoints[-1][0])
            # # yaw_error = (-2*np.pi+yaw_error) if yaw_error > np.pi else yaw_error
            # # yaw_error = (2*np.pi+yaw_error) if yaw_error < -np.pi else yaw_error
            # # crosstrack_error = crosstrack_error if yaw_error > 0 else -crosstrack_error
            #
            # steer = np.arctan(error_gain*crosstrack_error/(softening_constant+v))
            # print('Steer', steer)
            # steer = (-2*np.pi + steer) if steer > np.pi else steer
            # steer = (2*np.pi + steer) if steer < - np.pi else steer
            # if steer > 1.22:
            #     steer = 1.22
            # if steer < -1.22:
            #     steer = -1.22
            # steer_output = steer

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################

        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
        self.vars.int_acc = integral
        self.vars.prev_error = curr_error

