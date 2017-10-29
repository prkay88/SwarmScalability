#!/bin/bash
#Compile build one time.

#Souce: https://github.com/BCLab-UNM/DDSA-ARGoS/blob/master/runntimes.sh

#Run the build.sh file
echo Running build.sh
./build.sh

#Iterate 1 until the 1st argument is reached. Inside we compile code send
#the terminal output to argos_log.txt and compile the 2nd arguement and 
#send its output to 3rd arguement.
for ((i=1;i<=$1;i++));
do
 echo "Running $i of $1"
 argos3 -l argos_log.txt -c $2 >> $3 &
done

#Wait for the proceses to finish 
wait

#Prints done 
echo All done
