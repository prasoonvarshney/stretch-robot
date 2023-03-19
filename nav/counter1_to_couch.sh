python coord_goals_tepper.py -g 1 # go to counter1

python arm_motion.py -loc counter1 -int open
python arm_motion.py -loc counter1 -int close
python arm_motion.py -loc retracted -int close
python coord_goals_tepper.py -g 0 # go to couch

navigate to couch
python arm_motion.py -loc couch -int close
python arm_motion.py -loc couch -int open
python arm_motion.py -loc retracted -int rest
python coord_goals_tepper.py -g 1 # go back to counter1
