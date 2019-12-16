#!/usr/bin/env python
import argparse
import random
import copy
import cv2
import math

import rospy

# import baxter_interface

# from baxter_interface import CHECK_VERSION

from tictactoe_AI import play
from corner_calculate import corner_calculate
from block_set import block_set, area_fix
from player_move_read import player_move_read, move_test
from ros_image_saver import get_image
from baxter_kinematic import TicTacToeBaxter

# import kinematics_ttt

import time



ACTIVE = True
INACTIVE = False

# img = cv2.imread('camera_image.jpeg')
# img2  =  cv2.imread('ReadPoseImg_moved2.png')

def main():
    """Play Tic Tac Toe with Human Player
    """

    print("Initializing node... ")
    rospy.init_node("play_ttt", anonymous=True)
    #rospy.init_node("rsdk_ik_service_client")
    baxter = TicTacToeBaxter()
    rospy.on_shutdown(baxter.clean_shutdown)
    #baxter.play()

    final = "No Final"

    list_current_status = [ACTIVE  , ACTIVE  , ACTIVE,
                       ACTIVE, ACTIVE, ACTIVE  ,
                       ACTIVE, ACTIVE, ACTIVE ]
    # list_current_status = [ACTIVE  , ACTIVE  , INACTIVE,
    #                    INACTIVE, INACTIVE, ACTIVE  ,
    #                    INACTIVE, ACTIVE  , ACTIVE   ]

    grid = [[0,0,0],[0,0,0],[0,0,0]]
    move = [0,0,0]

    # kinematics_ttt.free_position()   #-------Kinematics
    baxter.set_neutral()
    # kinematics_ttt.read_position()   #-------Kinematics
    baxter.move_to_position("right", baxter.get_position_coords(-1,0))
    time.sleep(2)
    # get image                        #-------ROS Camera
    print(0)
    get_image()
    print(1)
    img = cv2.imread('camera_image.jpeg')
    print(2)
    time.sleep(2)
    list_point = corner_calculate(img)
    th_b = 30
    list_area = block_set(list_point, th_b)
    list_area_fix, list_area_fix_dig = area_fix(list_area)

#####
    for k in range(len(list_area)):
        for kk in range(len(list_area[k])):
            cv2.line(img,(list_area[k][kk%4][0],list_area[k][kk%4][1]),(list_area[k][(kk+1)%4][0],list_area[k][(kk+1)%4][1]),(0,0,255),2)
            cv2.line(img,(list_area_fix[k][kk%4][0],list_area_fix[k][kk%4][1]),(list_area_fix[k][(kk+1)%4][0],list_area_fix[k][(kk+1)%4][1]),(0,255,0),2)

    cv2.imwrite('blockset.jpg',img)
#####

    # print(list_current_status)

    # player_move, list_current_status_new = player_move_read(img2, list_area_fix, list_area_fix_dig, list_current_status)
    # list_current_status = copy.copy(list_current_status_new)
    # print(player_move)
    # print(list_current_status)



    # Repeat while not in end state
    print('move[2]', '\n',move[2])
    while(move[2] == 0):
	print('*')
        # kinematics_ttt.free_position()   #-------Kinematics
        baxter.set_neutral()
        time.sleep(10)
        # kinematics_ttt.read_position()   #-------Kinematics
        baxter.move_to_position("right", baxter.get_position_coords(-1,0))
        time.sleep(1)
        # get image                     #-------ROS Camera
        get_image()
        img = cv2.imread('camera_image.jpeg')
        time.sleep(1)
        player_move, list_current_status_new = player_move_read(img, list_area_fix, list_area_fix_dig, list_current_status)
        list_current_status = copy.copy(list_current_status_new)
        print(player_move)

        move_row = int(math.floor(player_move / 3))
        move_col = int(player_move % 3)
        grid[move_row][move_col] = -1
        move = play(grid)
        # if already lost
        if move[0] == -1:
            final = "Draw"
            break
        # if still playable, draw move and repeat
        else:
            # kinematics_ttt.draw_x(move)    #-------Kinematics
            baxter.make_move(move[0],move[1])
            grid[move[0]][move[1]] = 1
            move_next = (move[0] * 3) + move[1]
            list_current_status[move_next] = INACTIVE
    # Extrapolate if Won or Lost
    if(move[2] == -1):
        final = "Draw"
        print(final)
    elif(move[2] == 1):
        final = "Win"
        print(final)
    else:
        final = "Shouldnt get here: Lose"
        print(final)


if __name__ == '__main__':
    main()
