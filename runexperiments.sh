#!/bin/bash

#Source: https://github.com/BCLab-UNM/DDSA-ARGoS/blob/master/runexperimentset1.sh

#Run experiments for CASE 1: powerfailure
# for i in 10 30 50 70 100 120
# do
# 	./runtimes.sh $1 experiments/powerFailure${i}.argos results/powerFailure${i}.txt
# done

# #Run experiments for CASE 2: sensorFailure
# for i in 10 30 50 70 100 120
# do 
# 	./runtimes.sh $1 experiments/sensorFailure${i}.argos results/sensorFailure${i}.txt
# done

#Run experiments for CASE 3: motorFailure
for i in 10 30 50 70 100 120
do
	./runtimes.sh $1 experiments/motorFailure${i}.argos results/motorFailure${i}.txt
done

