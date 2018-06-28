#!/usr/bin/env python
import rospy
import sys
import time
import math
import os
import numpy as np
import cv2
# from typing import NamedTuple

from gpd.msg import GraspConfigList
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import String
from sensor_msgs.msg import Image
from detecting_hand.srv import GetObjLoc
from detecting_hand.srv import Conv2DTo3D
from detecting_hand.msg import bounding_box_calculated_center
from detecting_hand.msg import msg2dto3d
from detecting_hand.msg import msg2dto3d_object
from cv_bridge import CvBridge, CvBridgeError

theta1_cup = 0
theta2_cup = 0
theta1_ban = 0
theta2_ban = 0
theta1_clock = 0
theta2_clock = 0
count = 0
flag = 1
hand_x_sum = 0
hand_x_avg = 0
hand_y_avg = 0
hand_y_sum = 0
hand_z_sum = 0
hand_z_avg = 0
hand_count = 0
change = 2
change2 = 2
distance_cup_hand1 = 0
distance_cup_hand2 = 0
distance_ban_hand1 = 0
distance_ban_hand2 = 0
prev_hand_x_sum = 0
prev_hand_y_sum = 0
cup_loc = []
hand_loc = []
v1 = []
v2 = []
v3 = []
v4 = []
count_inside = 0;
count_cup = 0;
count_ban = 0;
cup_loc_save = []
bowl_loc_save = []
draw_box = 0
cup_chance = 0
ban_chance = 0
clock_chance = 0


class location:
    center_x_1 = 0
    center_y_1 = 0
    center_x_2 = 0
    center_y_2 = 0


class location_center_3d:
    center_x = 0
    center_y = 0


class location_center:
    center_x = 0
    center_y = 0


hand = location_center()
cup1 = location_center()  # first point of the cup
cup2 = location_center()  # second point of the cup
ban1 = location_center()  # first point of the banana
ban2 = location_center()  # second point of the banana
clock1 = location_center()
clock2 = location_center()
cup1_loc = location()
cup1_loc_center = location_center()
obj_cup1 = []
obj_cup2 = []
cup_fix_loc_1 = location_center()
cup_fix_loc_2 = location_center()
ban_fix_loc_1 = location_center()
ban_fix_loc_2 = location_center()
clock_fix_loc_1 = location_center()
clock_fix_loc_2 = location_center()
fix_cup_flag = 0
fix_ban_flag = 0
fix_clock_flag = 0
cup_check = 1
ban_check = 1
clock_check = 1


def center_callback(hand_center):  # storing the hand location in 3d

    global hand, flag
    hand.center_x = hand_center.x
    hand.center_y = hand_center.y
    r_val = get_obj_loc_client()
    callback(r_val)


#


def get_obj_loc_client():  # service call to find the cup location from YOLO

    rospy.wait_for_service('get_object_loc')

    try:
        get_obj_loc = rospy.ServiceProxy('get_object_loc', GetObjLoc)
        global cup_loc_save, bowl_loc_save, clock_loc_save, hand_loc, hand_count
        global fix_cup_flag, fix_ban_flag, fix_clock_flag, cup_fix_loc_1, cup_fix_loc_2, ban_fix_loc_1, ban_fix_loc_2, clock_fix_loc_1, clock_fix_loc_2, cup_check, ban_check, clock_check
        msg = get_obj_loc("scissors")
        if msg.Class == "scissors" and cup_check == 1:
            cup_loc_save = msg
            cup1.center_x = msg.xmin;
            cup1.center_y = msg.ymin + (msg.ymaxx - msg.ymin) / 2;
            cup2.center_x = msg.xmax;
            cup2.center_y = msg.ymin + (msg.ymaxx - msg.ymin) / 2;
            val_1 = 1
            if fix_cup_flag == 0:
                cup_fix_loc_1 = cup1
                cup_fix_loc_2 = cup2
                fix_cup_flag = 1
            if fix_cup_flag == 2:
                if cup_fix_loc_1.center_x - 10 <= cup1.center_x <= cup_fix_loc_1.center_x + 10 or cup_fix_loc_1.center_y - 10 <= cup1.center_y <= cup_fix_loc_1.center_y + 10:
                    # if cup_fix_loc_1 == cup1 and cup_fix_loc_2 == cup2:
                    print
                    "cup location didn't change**********"
                else:
                    print
                    "cup location change"
                    # cup_fix_loc_1 = cup1
                    # cup_fix_loc_2 = cup2
                    fix_cup_flag = 1
                    cup_check = 0
                    ban_check = 0
                    clock_check = 0
                    hand_loc = [];
                    hand_count = 0;
        elif msg.Class == ' ':
            val_1 = 0
            fix_cup_flag = 2
            print
            "cup loc change"
            with open("example2.txt", "a") as f:
                f.write("I didn't find cup location. my bad")
        msg = get_obj_loc("bowl")
        if msg.Class == "bowl" and ban_check == 1:
            bowl_loc_save = msg
            ban1.center_x = msg.xmin;
            ban1.center_y = msg.ymin + (msg.ymaxx - msg.ymin) / 2;
            ban2.center_x = msg.xmax;
            ban2.center_y = msg.ymin + (msg.ymaxx - msg.ymin) / 2;
            val_2 = 1
            if fix_ban_flag == 0:
                ban_fix_loc_1 = ban1
                ban_fix_loc_2 = ban2
                fix_ban_flag = 1
            if fix_ban_flag == 2:
                if ban_fix_loc_1.center_x - 10 <= ban1.center_x <= ban_fix_loc_1.center_x + 10 or ban_fix_loc_1.center_y - 10 <= ban1.center_y <= ban_fix_loc_1.center_y + 10:
                    # if ban_fix_loc_1 == cup1 and ban_fix_loc_2 ==  cup2:
                    print
                    "bowl location didn't change**********"
                else:
                    print
                    "bowl location change"
                    # ban_fix_loc_1 = ban11
                    # ban_fix_loc_2 = ban2
                    fix_ban_flag = 1
                    hand_loc = [];
                    hand_count = 0;
                    cup_check = 0
                    ban_check = 0
                    clock_check = 0
        elif msg.Class == ' ':

            print
            "bowl location not found\n"
            with open("example2.txt", "a") as f:
                f.write("I didn't find bowl location. my bad")
            val_2 = 0
            fix_ban_flag = 2

        msg = get_obj_loc("clock")
        if msg.Class == "clock" and clock_check == 1:
            clock_loc_save = msg
            clock1.center_x = msg.xmin;
            clock1.center_y = msg.ymin + (msg.ymaxx - msg.ymin) / 2;
            clock2.center_x = msg.xmax;
            clock2.center_y = msg.ymin + (msg.ymaxx - msg.ymin) / 2;
            val_3 = 1
            if fix_clock_flag == 0:
                clock_fix_loc_1 = clock1
                clock_fix_loc_2 = clock2
                fix_clock_flag = 1
            if fix_clock_flag == 2:
                if clock_fix_loc_1.center_x - 10 <= clock1.center_x <= clock_fix_loc_1.center_x + 10 or clock_fix_loc_1.center_y - 10 <= clock1.center_y <= clock_fix_loc_1.center_y + 10:
                    # if clock_fix_loc_1 == clock1 and clock_fix_loc_2 ==  clock2:
                    print
                    "clock location didn't change**********"
                else:
                    print
                    "clock location change"
                    # clock_fix_loc_1 = clock1
                    # clock_fix_loc_2 = clock2
                    fix_clock_flag = 1
                    hand_loc = [];
                    hand_count = 0;
                    cup_check = 0
                    ban_check = 0
                    clock_check = 0

        elif msg.Class == ' ':

            print
            "clock location not found\n"
            with open("example2.txt", "a") as f:
                f.write("I didn't find bowl location. my bad")
            val_3 = 0
            fix_clock_flag = 2

    except rospy.ServiceException, e:
        print
        "Service call failed: %s" % e
    return (val_1 and val_2 and val_3)


def unit_vector(vector):  # calculating angle between two vectors
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    with open("example2.txt", "a") as f:
        f.write("%s %s\n" % (v1, v2))
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return round(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)), 2)


def calculating_vectors(objx2, objy2, objx1, objy1, handx, handy):
    global v1, v2, v3, v4
    v1 = []
    v2 = []
    v3 = []
    v4 = []
    v1.append(objx2 - objx1)  # vector between obj1 -> obj2
    v1.append(objy2 - objy1)

    v2.append(handx - objx1)  # vector between obj1 -> hand
    v2.append(handy - objy1)

    v3.append(objx1 - objx2)  # vector between obj2 -> obj1
    v3.append(objy1 - objy2)

    v4.append(handx - objx2)  # vector between obj2 -> hand
    v4.append(handy - objy2)


def intention_object(theta1, prev_theta1, theta2, prev_theta2):
    if theta1 == prev_theta1 and theta2 == prev_theta2:

        val = 1
    elif theta1 < prev_theta1 and theta2 < prev_theta2:
        val = 2
    elif theta1 < prev_theta1 and theta2 == prev_theta2:
        val = 2
    elif theta1 <= prev_theta1 and theta2 < prev_theta2:
        val = 2
    elif theta1 == prev_theta1 and theta2 < prev_theta2:
        val = 2
    else:
        val = 0
    print
    "helloooooo"
    return val


def callback(r_val):
    global count, near_cup, theta1_cup, theta2_cup, theta1_ban, theta2_ban, theta1_clock, theta2_clock, hand_count, hand_x_sum, hand_y_sum, hand_z_sum, hand_loc, prev_hand_x_avg, prev_hand_y_avg, hand_x_avg, hand_y_avg, cup1_loc, cup1_loc_center, hand, obj1, obj2, v1, v2, v3, v4, flag, obj_fix1, obj_fix2
    # time.sleep(0.01)
    global distance_cup_hand1, distance_cup_hand2, distance_ban_hand1, distance_ban_hand2, change, change2, count_inside, count_cup, count_ban, draw_box, cup_chance, ban_chance, clock_chance

    # print "hand center: %.2f %.2f %.2f"%(hand.center_x,hand.center_y,hand.center_z)

    if r_val != 0:
        print
        "%d change" % (change)
        print
        "hand center: %.2f %.2f" % (hand.center_x, hand.center_y)
        print
        "%d" % (count)
        print
        "cup center:%.2f %.2f" % (cup1.center_x, cup1.center_y)
        print
        "cup center: %.2f %.2f " % (cup2.center_x, cup2.center_y)
        print
        "bowl center:%.2f %.2f " % (ban1.center_x, ban1.center_y)
        print
        "bowl center: %.2f %.2f " % (ban2.center_x, ban2.center_y)

        hand_loc.append(hand);  # storing the hand location in a list for average

        hand_x_sum = hand_x_sum + hand_loc[hand_count].center_x;
        hand_y_sum = hand_y_sum + hand_loc[hand_count].center_y;

        with open("example2.txt", "a") as f:
            f.write('hand center(not avg):%.2f %.2f %d\n' % (
            hand_loc[hand_count].center_x, hand_loc[hand_count].center_y, hand_count))
        hand_count = hand_count + 1;

        if count % 2 == 0:  # average the hand values after 10 rounds
            count_inside = count_inside + 1
            prev_hand_x_avg = hand_x_avg
            prev_hand_y_avg = hand_y_avg
            hand_x_avg = hand_x_sum / 2
            hand_y_avg = hand_y_sum / 2

            with open("example2.txt", "a") as f:
                f.write('count %d\ncup center: %.2f %.2f \ncup center: %.2f %.2f \nhand center:%.2f %.2f \n' % (
                count, cup1.center_x, cup1.center_y, cup2.center_x, cup2.center_y, hand_x_avg, hand_y_avg))
            with open("example2.txt", "a") as f:
                f.write('count %d\nbanana center: %.2f %.2f \nbanana center: %.2f %.2f \n' % (
                count, ban1.center_x, ban1.center_y, ban2.center_x, ban2.center_y))
            print
            "hand avg center: %.2f %.2f " % (hand_x_avg, hand_y_avg)
            calculating_vectors(cup2.center_x, cup2.center_y, cup1.center_x, cup1.center_y, hand_x_avg, hand_y_avg)

            prev_theta1_cup = theta1_cup
            prev_theta2_cup = theta2_cup
            prev_distance_cup_hand1 = distance_cup_hand1

            # if not (hand_x_avg>=prev_hand_x_avg-1 and hand_x_avg<=prev_hand_x_avg+1 and hand_y_avg>=prev_hand_y_avg-1 and hand_y_avg<=prev_hand_y_avg+1):

            # distance_cup_hand2 = round(math.sqrt((cup2.center_y-hand_y_avg)*(cup2.center_y-hand_y_avg) + (cup2.center_x-hand_x_avg)*(cup2.center_x-hand_x_avg)),2)
            theta1_cup = angle_between(v1, v2)  # calculating the angle between first two vectors in degree
            theta1_cup = (int)(math.degrees(theta1_cup))
            theta2_cup = angle_between(v3, v4)  # calculating the angle between last two vectors in degree
            theta2_cup = (int)(math.degrees(theta2_cup))

            calculating_vectors(ban2.center_x, ban2.center_y, ban1.center_x, ban1.center_y, hand_x_avg, hand_y_avg)
            prev_theta1_ban = theta1_ban
            prev_theta2_ban = theta2_ban
            prev_distance_ban_hand1 = distance_ban_hand1

            # if not (hand_x_avg>=prev_hand_x_avg-1 and hand_x_avg<=prev_hand_x_avg+1 and hand_y_avg>=prev_hand_y_avg-1 and hand_y_avg<=prev_hand_y_avg+1):

            # distance_ban_hand2 = round(math.sqrt((ban2.center_y-hand_y_avg)*(ban2.center_y-hand_y_avg) + (ban2.center_x-hand_x_avg)*(ban2.center_x-hand_x_avg)),2)
            theta1_ban = angle_between(v1, v2)  # calculating the angle between first two vectors in degree
            theta1_ban = (int)(math.degrees(theta1_ban))
            theta2_ban = angle_between(v3, v4)  # calculating the angle between last two vectors in degree
            theta2_ban = (int)(math.degrees(theta2_ban))

            calculating_vectors(clock2.center_x, clock2.center_y, clock1.center_x, clock1.center_y, hand_x_avg,
                                hand_y_avg)
            prev_theta1_clock = theta1_clock
            prev_theta2_clock = theta2_clock
            # prev_distance_ban_hand1 = distance_ban_hand1

            # if not (hand_x_avg>=prev_hand_x_avg-1 and hand_x_avg<=prev_hand_x_avg+1 and hand_y_avg>=prev_hand_y_avg-1 and hand_y_avg<=prev_hand_y_avg+1):

            # distance_ban_hand2 = round(math.sqrt((ban2.center_y-hand_y_avg)*(ban2.center_y-hand_y_avg) + (ban2.center_x-hand_x_avg)*(ban2.center_x-hand_x_avg)),2)
            theta1_clock = angle_between(v1, v2)  # calculating the angle between first two vectors in degree
            theta1_clock = (int)(math.degrees(theta1_clock))
            theta2_clock = angle_between(v3, v4)  # calculating the angle between last two vectors in degree
            theta2_clock = (int)(math.degrees(theta2_clock))
            with open("example2.txt", "a") as f:
                f.write("\ndistance: %f %f %f %f\n" % (
                distance_cup_hand1, distance_cup_hand2, distance_ban_hand1, distance_ban_hand2))
            if cup_check == 0:
                val_cup = 0
            else:
                val_cup = intention_object(theta1_cup, prev_theta1_cup, theta2_cup, prev_theta2_cup)
            if ban_check == 0:
                val_ban = 0
            else:
                val_ban = intention_object(theta1_ban, prev_theta1_ban, theta2_ban, prev_theta2_ban)
            if clock_check == 0:
                val_clock = 0
            else:
                val_clock = intention_object(theta1_clock, prev_theta1_clock, theta2_clock, prev_theta2_clock)

            val_list = [val_cup, val_ban, val_clock]
            m = [i for i, v in enumerate(val_list) if v == max(val_list)]
            if len(m) == 1:

                if val_cup > val_ban and val_cup > val_clock:
                    print
                    "going to the cup%d " % (count)
                    draw_box = 1
                    if hand_x_avg != 515 and hand_y_avg != 363:
                        cup_chance = cup_chance + 1
                        with open("example3.txt", "a") as f:
                            f.write("\ngoing to the bottle %d" % (count))
                    else:
                        with open("example3.txt", "a") as f:
                            f.write("\norange %d" % (count))
                elif val_ban > val_cup and val_ban > val_clock:
                    print
                    "going to the cbowl%d " % (count)
                    draw_box = 2
                    if hand_x_avg != 515 and hand_y_avg != 363:
                        ban_chance = ban_chance + 1
                        with open("example3.txt", "a") as f:
                            f.write("\ngoing to the clock %d" % (count))
                    else:
                        with open("example3.txt", "a") as f:
                            f.write("\norange %d" % (count))
                elif val_clock > val_cup and val_clock > val_ban:
                    print
                    "going to the cbowl%d " % (count)
                    draw_box = 2
                    if hand_x_avg != 515 and hand_y_avg != 363:
                        clock_chance = clock_chance + 1
                        with open("example3.txt", "a") as f:
                            f.write("\ngoing to the clock %d" % (count))
                    else:
                        with open("example3.txt", "a") as f:
                            f.write("\norange %d" % (count))
            elif len(m) > 1:
                if max(val_list) != 0:
                    print
                    "confused%d " % (count)
                    draw_box = 0
                    if val_cup == max(val_list) and cup_chance > ban_chance and cup_chance > clock_chance:
                        if hand_x_avg != 515 and hand_y_avg != 363:
                            cup_chance = cup_chance + 1
                            with open("example3.txt", "a") as f:
                                f.write("\ngoing to the bottle %d" % (count))
                        else:
                            with open("example3.txt", "a") as f:
                                f.write("\norange %d" % (count))
                    elif val_ban == max(val_list) and ban_chance > cup_chance and ban_chance > clock_chance:
                        if hand_x_avg != 515 and hand_y_avg != 363:
                            ban_chance = ban_chance + 1
                            with open("example3.txt", "a") as f:
                                f.write("\ngoing to the clock %d" % (count))
                        else:
                            with open("example3.txt", "a") as f:
                                f.write("\norange %d" % (count))
                    elif val_clock == max(val_list) and clock_chance > cup_chance and clock_chance > ban_chance:
                        if hand_x_avg != 515 and hand_y_avg != 363:
                            clock_chance = clock_chance + 1
                            with open("example3.txt", "a") as f:
                                f.write("\ngoing to the clock %d" % (count))
                        else:
                            with open("example3.txt", "a") as f:
                                f.write("\norange %d" % (count))
                    else:
                        if hand_x_avg != 515 and hand_y_avg != 363:
                            ban_chance = ban_chance + 1
                            cup_chance = cup_chance + 1
                            clock_chance = clock_chance + 1
                            with open("example3.txt", "a") as f:
                                f.write("\nboth same %d" % (count))
                        else:
                            with open("example3.txt", "a") as f:
                                f.write("\norange %d" % (count))
                else:
                    print
                    "I guess it's going away%d " % (count)
                    draw_box = 0
                    if hand_x_avg != 515 and hand_y_avg != 363:
                        with open("example3.txt", "a") as f:
                            f.write("\nGOING AWAY %d" % (count))
                    else:
                        with open("example3.txt", "a") as f:
                            f.write("\norange %d" % (count))

            '''if theta1_cup<=prev_theta1_cup and theta2_cup<=prev_theta2_cup:
            #if ((theta1_cup>=prev_theta1_cup and theta1_cup<=prev_theta1_cup+1) or theta1_cup<=prev_theta1_cup) and ((theta2_cup>=prev_theta2_cup and theta2_cup<=prev_theta2_cup+1) or theta2_cup<=prev_theta2_cup):

                if distance_cup_hand1 <=80:
                    with open("example2.txt", "a") as f:
                        f.write("going to the grab the cup")

                elif distance_cup_hand1 < prev_distance_cup_hand1:	
                    print "going to the cup%d "%(count)
                    with open("example2.txt", "a") as f:
                        f.write("going to the cup")
                elif distance_cup_hand1 == prev_distance_cup_hand1 :
                    print "staying in fix to cup%d "%(count)
                    with open("example2.txt", "a") as f:
                        f.write("\nstaying in fix to cup ")
                count_cup=count_cup+1;
            if theta1_ban<=prev_theta1_ban and theta2_ban<=prev_theta2_ban:
            #if ((theta1_ban>=prev_theta1_ban and theta1_ban<=prev_theta1_ban+1) or theta1_ban<=prev_theta1_ban) and ((theta2_ban>=prev_theta2_ban and theta2_ban<=prev_theta2_ban+1) or theta2_ban<=prev_theta2_ban):
                if distance_ban_hand1 <=80:
                    with open("example2.txt", "a") as f:
                        f.write("going to the grab the bowl")

                elif distance_ban_hand1 < prev_distance_ban_hand1 :	
                    print "going to the banana%d "%(count)
                    with open("example2.txt", "a") as f:
                            f.write("\ngoing to the bowl")
                elif distance_ban_hand1 == prev_distance_ban_hand1 :
                    print "staying in fix towards banana%d "%(count)
                    with open("example2.txt", "a") as f:
                            f.write("\nstaying in fix to bowl")
                count_ban=count_ban+1;
            else:
                print "going away %d "%(count)
                with open("example2.txt", "a") as f:
                    f.write("\ngoing away")	'''
            # hand_loc = [];
            # hand_count = 0;
            hand_x_sum = 0;
            hand_y_sum = 0;

            with open("example2.txt", "a") as f:
                f.write("current angle :%f %f\n" % (theta1_cup, theta2_cup))
            with open("example2.txt", "a") as f:

                f.write("prev angle :%f %f\n\n\n" % (prev_theta1_cup, prev_theta2_cup))
            with open("example2.txt", "a") as f:
                f.write("current angle :%f %f\n" % (theta1_ban, theta2_ban))
            with open("example2.txt", "a") as f:

                f.write("prev angle :%f %f\n\n\n" % (prev_theta1_ban, prev_theta2_ban))
            with open("example2.txt", "a") as f:

                f.write("prev angle :%f %f\n\n\n" % (prev_theta1_clock, prev_theta2_clock))
        #	else:
        #	print"one class is blank"
        count = count + 1
        '''if count_inside == 5:
            count_inside =0
            with open("example2.txt","a") as f:
                f.write("%d %d\n"%(count_cup,count_ban))

            if count_cup>count_ban:
                with open("example2.txt","a") as f:
                    f.write("i guess it is going for cup. :S \n")
            else:
                with open("example2.txt","a") as f:
                    f.write("i guess it is going for bowl. :S \n")

            count_cup=0;
            count_ban=0;'''
    else:
        print
        "location change"


def image_callback(msg_img):
    global draw_box, cup_chance, ban_chance, clock_chance
    print
    "hei hei hei"
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg_img, 'bgr8')
    with open("example3.txt", "a") as f:
        f.write("\nimage_callback %d %d %d" % (count, cup_chance, ban_chance))
    if cup_chance == 0 and ban_chance == 0 and clock_chance == 0:
        cv2.rectangle(cv_image, (100, 100), (200, 200), (255, 255, 255), 1, 8, 0)
    elif cup_chance > ban_chance and cup_chance > clock_chance:
        cv2.rectangle(cv_image, (cup_loc_save.xmin, cup_loc_save.ymin), (cup_loc_save.xmax, cup_loc_save.ymaxx),
                      (255, 255, 255), 4, 8, 0)
    elif ban_chance > clock_chance and ban_chance > clock_chance:
        cv2.rectangle(cv_image, (bowl_loc_save.xmin, bowl_loc_save.ymin), (bowl_loc_save.xmax, bowl_loc_save.ymaxx),
                      (255, 255, 255), 4, 8, 0)
    elif draw_box == 0:
        cv2.rectangle(cv_image, (100, 100), (200, 200), (255, 255, 255), 1, 8, 0)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)


def get_intention():
    # sub_center_object = rospy.Subscriber('chatter', msg2dto3d_object, center_callback_object) #subscribing the topic which publishing 3d cup points
    # sub_center = rospy.Subscriber('hand_bounding_box_center', bounding_box_calculated_center, center_callback) #subscribing center of a hand box from skinDetection
    sub_center = rospy.Subscriber('hand_bounding_box_center', bounding_box_calculated_center,
                                  center_callback)  # subscribing the topic which publishing hand center in 3d
    sub_img = rospy.Subscriber('/kinect2/qhd/image_color', Image, image_callback)


# if change !=0 and change2!=0:
# cup_fix1=cup1
# cup_fix2=cup2
# ban_fix1=ban1
# ban_fix2=ban2


def usage():
    return "%s [obj_name]" % sys.argv[0]


if __name__ == "__main__":
    '''if len(sys.argv) == 1:
       # obj_name = sys.argv[1]
	obj_name = "person"
    else:
        print usage()
        sys.exit(1)'''
    # for x in range(0, 10):

    # while 1:

    # rospy.sleep(1);
    rospy.init_node('get_obj_loc_client')
    get_intention()

    # print "Requesting: %s"%(obj_name)

    rospy.spin()

# time.sleep(0.5)

