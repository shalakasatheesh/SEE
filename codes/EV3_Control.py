#!/usr/bin/env micropython

from time import sleep, time
import sys
import math
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
from ev3dev2.button import Button

WHEEL_DIAMETER = 5.6  # cm
MAIN_AXIS_LENGTH = 12.0  # cm

buttons = Button()
move = MoveTank(OUTPUT_A, OUTPUT_D)
spkr = Sound()
motor_1 = LargeMotor(OUTPUT_A)
motor_2 = LargeMotor(OUTPUT_D)

motor_1_path = []  # in rad
motor_2_path = []  # in rad

motor_1_path.append(motor_1.position)
motor_2_path.append(motor_2.position)

robot_orientation = 0.0  # in rad
robot_position_x = 0.0  # in cm
robot_position_y = 0.0  # in cm

distance_traveled_wheel_1 = 0.0  # in cm
distance_traveled_wheel_2 = 0.0  # in cm

start_time = str(time()).split(".")[0]
times_motor_1 = []
times_motor_2 = []

spkr.speak('Press a button')
while True:
    if buttons.left:
        move.on_for_seconds(SpeedPercent(30), SpeedPercent(40), 2.2, block=False)
        file_name = 'left_' + start_time

    elif buttons.up:
        move.on_for_seconds(SpeedPercent(40), SpeedPercent(40), 2.2, block=False)
        file_name = 'up_' + start_time

    elif buttons.right:
        move.on_for_seconds(SpeedPercent(40), SpeedPercent(30), 2.2, block=False)
        file_name = 'right_' + start_time

    if (motor_1.is_running):
        motor_1_path.append((motor_1.position * math.pi) / 180.0)
        times_motor_1.append(time())
    if (motor_2.is_running):
        motor_2_path.append((motor_2.position * math.pi) / 180.0)
        times_motor_2.append(time())

    if (motor_1.is_holding and motor_2.is_holding):
        data_length = min(len(motor_1_path), len(motor_2_path))

        with open(file_name + '_both_motors_path.csv', "w") as f_wheels_path:
            header = 'motor_1_path_rad, motor_2_path_rad, time_motor_1, time_motor_2\n'
            f_wheels_path.write(header)
            for motor_1, motor_2, time_1, time_2 in zip(motor_1_path, motor_2_path, times_motor_1, times_motor_2):
                f_wheels_path.write(str(motor_1) + ', ' + str(motor_2) + ', ' +
                                    str(time_1) + ', ' + str(time_2) + '\n')  # in rad

        with open(file_name + '_robot_path.csv', "w") as f_robot_path:
            header = 'robot_position_x, robot_position_y, robot_orientation\n'

            f_robot_path.write(header)
            for i in range(data_length):
                if i is 0:
                    distance_traveled_wheel_1 = (WHEEL_DIAMETER * math.pi * motor_1_path[0]) / (2 * math.pi)
                    distance_traveled_wheel_2 = (WHEEL_DIAMETER * math.pi * motor_2_path[0]) / (2 * math.pi)
                else:
                    distance_traveled_wheel_1 = (WHEEL_DIAMETER * math.pi *
                                                 (motor_1_path[i] - motor_1_path[i - 1])) / (2 * math.pi)
                    distance_traveled_wheel_2 = (WHEEL_DIAMETER * math.pi *
                                                 (motor_2_path[i] - motor_2_path[i - 1])) / (2 * math.pi)

                delta_distance = (distance_traveled_wheel_1 + distance_traveled_wheel_2) / 2
                delta_angle = (distance_traveled_wheel_1 - distance_traveled_wheel_2) / MAIN_AXIS_LENGTH

                robot_orientation = robot_orientation + delta_angle
                robot_position_x = robot_position_x + delta_distance * math.sin(robot_orientation)
                robot_position_y = robot_position_y + delta_distance * math.cos(robot_orientation)

                str_robot_position_x = str(robot_position_x)
                str_robot_position_y = str(robot_position_y)
                str_robot_orientation = str((math.pi / 2) - robot_orientation)

                f_robot_path.write(str(str_robot_position_x + ', ' + str_robot_position_y +
                                       ', ' + str_robot_orientation + '\n'))

        spkr.speak('Motion completed')
        break

    # don't let this loop use 100% CPU
    sleep(0.001)
