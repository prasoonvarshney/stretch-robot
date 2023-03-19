#!/bin/bash

python3 arm_motion.py -loc retracted -int rest  # wherever it starts, retract arm first
python3 coord_goals_tepper.py -g 1 # go to counter1

python3 arm_motion.py -loc retracted -int open
python3 arm_motion.py -loc counter1 -int open
python3 arm_motion.py -loc counter1 -int close
python3 arm_motion.py -loc retracted -int carry
python3 coord_goals_tepper.py -g 0 # go to couch

python3 arm_motion.py -loc couch -int close  # extend arm
python3 arm_motion.py -loc couch -int open  # place the basket
python3 arm_motion.py -loc retracted -int open  # retract arm
python3 arm_motion.py -loc retracted -int rest  # rest gripper
python3 coord_goals_tepper.py -g 1 # go back to counter1
