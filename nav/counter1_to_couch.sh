#!/bin/bash
# make sure to 1. calibrate 2. run nav stack first
now=$(date +%T)

python3 arm_motion.py -loc retracted -int rest  # wherever it starts, retract arm first
echo "rest position | $(date +%T)" >> $now.txt
python3 coord_goals_tepper.py -g 1 # go to counter1
echo "go to counter1 | $(date +%T)" >> $now.txt

python3 arm_motion.py -loc retracted -int reach
python3 arm_motion.py -loc counter1 -int reach
echo "reach for basket | $(date +%T)" >> $now.txt
python3 arm_motion.py -loc counter1 -int close
python3 arm_motion.py -loc retracted -int carry
echo "grab basket | $(date +%T)" >> $now.txt

python3 coord_goals_tepper.py -g 0 # go to couch
echo "go to couch | $(date +%T)" >> $now.txt

python3 arm_motion.py -loc counter1 -int carry  # extend
python3 arm_motion.py -loc couch -int close  # drop arm
python3 arm_motion.py -loc couch -int open  # place the basket
echo "place basket | $(date +%T)" >> $now.txt

python3 arm_motion.py -loc retracted -int open  # retract arm
python3 arm_motion.py -loc retracted -int rest  # rest gripper
echo "rest position | $(date +%T)" >> $now.txt
python3 coord_goals_tepper.py -g 1 # go back to counter1
echo "go back to counter1 | $(date +%T)" >> $now.txt
