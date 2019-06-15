#!/usr/bin/env python3
from ev3dev.ev3 import *
import math

route = [[0.1, 0.1, 0.1], [0.1, 0.1, 0.3], [0.1, -0.1, 0.3], [0.1, -0.1, 0.1], [0.1, 0.1, 0.1]]

kp = 5
ki = 0.07
kd = 0.5
anti_windup = 10

d1 = 0.14
a2 = 0.12
a3 = 0.12
gear = [-60/8, 40/8, -40/8]
limit = [[90*math.pi/180, -90*math.pi/180],[90*math.pi/180,-30*math.pi/180],[135*math.pi/180,-30*math.pi/180]]
motor = [LargeMotor('outA'), LargeMotor('outB'), LargeMotor('outC')]
sensor = [TouchSensor('in1'), TouchSensor('in2'), TouchSensor('in3')]
file_obj = open("data_ind.txt", "w")

integral = err_old = {}
for i in range(3):
    integral[i] = 0
    err_old[i] = 0


def calibrate():
    for i in range(3):
        while (sensor[i].is_pressed == False):
            motor[i].run_direct(duty_cycle_sp = 25*(-1)**(i+1))
        motor[i].position = limit[i][0]*180/math.pi*gear[i]


def inverse_kinematics(x, y, z):
    r1=math.sqrt(x**2+y**2)
    r3=math.sqrt(x**2+y**2+(z-d1)**2)
    if r3 > a2 + a3:
        return False, {}

    desired_angle = {}
    if x == 0:
        if y == 0:
            desired_angle[0] = 0
        elif y > 0:
            desired_angle[0] = math.pi/2
        else:
            desired_angle[0] = -math.pi/2
    elif y == 0:
        if x > 0:
            desired_angle[0] = 0
        else:
            desired_angle[0] = math.pi
    else:
        desired_angle[0] = math.atan2(y, x)

    if z - d1 == 0:
        desired_angle[1] = math.pi/2 - math.acos((a2**2+r3**2-a3**2)/(2*a2*r3))
    elif z - d1 > 0:
        desired_angle[1] = math.atan(r1/(z-d1)) - math.acos((a2**2+r3**2-a3**2)/(2*a2*r3))
    else:
        desired_angle[1] = math.pi + math.atan(r1/(z-d1)) - math.acos((a2**2+r3**2-a3**2)/(2*a2*r3))

    desired_angle[2] = math.pi - math.acos((a2**2+a3**2-r3**2)/(2*a2*a3))

    for i in range(3):
        if desired_angle[i] > limit[i][0] or desired_angle[i] < limit[i][1]:
            return False, desired_angle
        desired_angle[i] = desired_angle[i]*gear[i]
    return True, desired_angle


try:
    calibrate()
    start_time = time.time()
    for j in range(len(route)):
        reachable, desired_angle = inverse_kinematics(route[j][0], route[j][1], route[j][2])
        if reachable:
            err = derivative = u = {}
            t0 = time.time() - start_time
            while (time.time() - t0 - start_time < 3):
                s = ""
                for i in range(3):
                    err[i] = desired_angle[i] - motor[i].position*math.pi/180
                    integral[i] += err[i]*ki
                    if abs(integral[i]) > anti_windup:
                        integral[i] = math.copysign(anti_windup, integral[i])
                    derivative[i] = (err[i] - err_old[i])*kd
                    err_old[i] = err[i]
                    u[i] = kp * err[i] + integral[i] + derivative[i]
                    u[i] = u[i]*100/8.4
                    if abs(u[i]) > 100:
                        u[i] = math.copysign(100, u[i])
                    motor[i].run_direct(duty_cycle_sp = u[i])
                    s += str(motor[i].position) + " "
                file_obj.write(s + str(time.time() - start_time) + '\n \n')
            print("moved to (" + str(route[j][0]) + ", " + str(route[j][1]) + ", " + str(route[j][2]) + ")")
        else:
            print("unreachable point (" + str(route[j][0]) + ", " + str(route[j][1]) + ", " + str(route[j][2]) + ")")

finally:
    for i in range(3):
        motor[i].stop(stop_action='brake')
    
