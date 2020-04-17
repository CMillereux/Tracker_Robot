#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

import actionlib
from geometry_msgs.msg import Twist
import sys
import select
import os
import tty
import termios
if os.name == 'nt':
    import msvcrt

msg = """
Your TurtleBot3 is self controlled!
---------------------------
You don't need to touch anything to make your TurtleBot3 operate !

Linear and angular velocity will be increase or decrease automatically.
Linear speed for Burger is about 0.22, Waffle and Waffle Pi is about 0.26
Angular velocity for Burger is about 2.84, Waffle and Waffle Pi is about 1.82

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)


def getKey():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def erreur_x(data):
    datax = []
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    for i in range(len(data.ranges)):
        if np.isinf(data.ranges[i]):
            # deleting useless value
            None

        else:
            angle = angle_min + angle_increment * i
            x = data.ranges[i] * np.cos(angle)

            if x > 0.5 and x < 1:
                datax.append(x)

        x_moy = 0

        for i in range(len(datax)):
            #        rospy.loginfo("x={}, y ={}".format(datax[i],datay[i]))
            x_moy = x_moy + datax[i]

        if len(datax) != 0:
            x_moy = x_moy / len(datax)
            rospy.loginfo("x={}".format(x_moy))
            # on veut ramener le baricentre vers la zone d'interet : x = 0.75 et y = 0
            # si erreur_x est positif on veut avancer et si erreur_x est negatif on veut reculer
            error_x = 0.75 - x
            # si erreur_y est positif il faut tourner vers la gauche et si erreur_y est negatif il faut tourner vers la droite
            k_l = 1
            k_r = 1
        # cmd_vel(erreur_x * k_l,erreur_y * k_r ) # to_do envoyer ces vitesses en s'inspirant du teleop_key
        else:
            rospy.loginfo("item lost !!!!!")
        return error_x


def erreur_y(data):
    datay = []
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    for i in range(len(data.ranges)):
        if np.isinf(data.ranges[i]):
            # deleting useless value
            None

        else:
            angle = angle_min + angle_increment * i
            y = data.ranges[i] * np.sin(angle)
            if y > -0.5 and y < 0.5:
                datay.append(y)
        y_moy = 0
        for i in range(len(datay)):
            #        rospy.loginfo("x={}, y ={}".format(datax[i],datay[i]))
            y_moy = y_moy + datay[i]
        if len(datay) != 0:
            y_moy = y_moy / len(datay)
            rospy.loginfo("y ={}".format(y_moy))
            # on veut ramener le baricentre vers la zone d'interet : x = 0.75 et y = 0
            # si erreur_x est positif on veut avancer et si erreur_x est negatif on veut reculer

            error_y = 0 - y  # si erreur_y est positif il faut tourner vers la gauche et si erreur_y est negatif il faut tourner vers la droite
            k_l = 1
            k_r = 1
        # cmd_vel(erreur_x * k_l,erreur_y * k_r ) # to_do envoyer ces vitesses en s'inspirant du teleop_key
        else:
            rospy.loginfo("item lost !!!!!")
    return error_y


def callback(data):
    datax = []
    datay = []
    angle_min = data.angle_min
    angle_increment = data.angle_increment
#    rospy.loginfo(data.ranges)
    rospy.loginfo(len(data.ranges))
    for i in range(len(data.ranges)):
        if np.isinf(data.ranges[i]):
            # deleting useless value
            None

        else:
            angle = angle_min + angle_increment * i
            x = data.ranges[i] * np.cos(angle)
            y = data.ranges[i] * np.sin(angle)
            if x > 0.5 and x < 1 and y > -0.5 and y < 0.5:
                datax.append(x)
                datay.append(y)
    x_moy = 0
    y_moy = 0
    for i in range(len(datax)):
        #        rospy.loginfo("x={}, y ={}".format(datax[i],datay[i]))
        x_moy = x_moy + datax[i]
        y_moy = y_moy + datay[i]
    if len(datax) != 0:
        x_moy = x_moy / len(datax)
        y_moy = y_moy / len(datay)
        rospy.loginfo("x={}, y ={}".format(x_moy, y_moy))
        # on veut ramener le baricentre vers la zone d'interet : x = 0.75 et y = 0
        # si erreur_x est positif on veut avancer et si erreur_x est negatif on veut reculer
        err_x = 0.75 - x
        err_y = 0 - y  # si erreur_y est positif il faut tourner vers la gauche et si erreur_y est negatif il faut tourner vers la droite
        k_l = 1
        k_r = 1
       # cmd_vel(erreur_x * k_l,erreur_y * k_r ) # to_do envoyer ces vitesses en s'inspirant du teleop_key
    else:
        rospy.loginfo("item lost !!!!!")


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while(1):
            key = getKey()
            if erreur_x > 0:
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            else:
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            
            if erreur_y > 0:
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            else:
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            if erreur_x == 0:
                target_linear_vel = 0.0
                control_linear_vel = 0.0
            elif erreur_y == 0.0:
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break
                if status == 20:
                    print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(
                control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(
                control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
