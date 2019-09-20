#!/bin/bash

#cut.py --inbag INBAG --outbag OUTBAG --start START --duration DURATION

python /home/robo/catkin_ws/src/srv_tools/bag_tools/scripts/cut.py \
--inbag /home/robo/bagfile/2017_09_21_move_base_2.bag \
--outbag /home/robo/bagfile/2017_09_21_move_base_2_cut_tmp.bag \
--start 666 \
--duration 174


