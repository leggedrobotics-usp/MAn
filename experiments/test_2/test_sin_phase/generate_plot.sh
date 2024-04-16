#!/bin/bash
# python ../../../python/compare_simulations_per_joint.py -f log_arm2_mbffwd_no_pred.csv -f log_arm2_mbffwd_2tau.csv -c "fi0" -c "fi1" -y "Shoulder" -y "Elbow" -l "Feedforward" -l "Feedforward + Prediction" -t "Time x Interaction Torques (sin input)"
# python ../../../python/compare_simulations_per_joint.py -f log_arm2_mbffwd_no_pred.csv -f log_arm2_mbffwd_3tau.csv -c "fi0" -c "fi1" -y "Shoulder" -y "Elbow" -l "Feedforward" -l "Feedforward + Prediction" -t "Time x Interaction Torques (sin input)"
python ../../../python/compare_simulations_per_joint.py -f log_arm2_mbffwd_no_pred.csv -f log_arm2_mbffwd_2tau.csv -c "fi0" -c "fi1" -y "Shoulder" -y "Elbow" -l "Feedforward" -l "Feedforward + Prediction" -t "Time x Interaction Torques (sin input)" -w 5 -w 15