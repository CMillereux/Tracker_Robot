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
import simple_navigation_goals
from simple_navigation_goals import SimpleNavigationGoals



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

WAFFLE_MAX_LIN_VEL = 0.3
WAFFLE_MAX_ANG_VEL = 2

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.1

k_l = 1
k_r = 1

chrono_start = 0
move = 0


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


def callback(data):
    erreur_x = 0
    erreur_y = 0
    datax = []
    datay = []
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    global chrono_start
    global move
    move = simple_navigation_goals.SimpleNavigationGoals()
    move.go_to (-3,1,0)
#    rospy.loginfo(data.ranges)
    rospy.loginfo(len(data.ranges))
    for i in range(len(data.ranges)):
        if np.isinf(data.ranges[i]):
            # deleting useless value
            pass

        else:
            angle = angle_min + angle_increment * i
            x = data.ranges[i] * np.cos(angle)
            y = data.ranges[i] * np.sin(angle)
            if x > 0.01 and x < 0.7 and y > -0.45 and y < 0.45:
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
        erreur_x = x_moy - 0.3
        erreur_y = y_moy - 0  # si erreur_y est positif il faut tourner vers la gauche et si erreur_y est negatif il faut tourner vers la droite
        k_l = 1
        k_r = 1
       # cmd_vel(erreur_x * k_l,erreur_y * k_r ) # to_do envoyer ces vitesses en s'inspirant du teleop_key
    else:
        rospy.loginfo("item lost !!!!!")
    if erreur_x != 0.3:
        chrono_start = rospy.Time.now()
    elif rospy.Time.now() - chrono_start >= rospy.Duration(3):
        move.go_to (-3,1,0)
    bouge(erreur_x, erreur_y)
    


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


def bouge(erreur_x, erreur_y):
    
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
    
            if erreur_x == 0:
                target_linear_vel = 0.0
                control_linear_vel = 0.0
            elif erreur_y == 0.0:
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                              
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
        twist.linear.x = erreur_x * k_l
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = erreur_y * k_r
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def listener():
    global chrono_start
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    chrono_start = rospy.Time.now()
    

    rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
