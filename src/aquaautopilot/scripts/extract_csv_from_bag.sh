for f in "$@"
do

   echo Converting bag file ${f}

   s=`echo ${f} | cut -d. -f1`
   echo $s

   echo Extracting commands
   rostopic echo -b ${f} -p /aqua/command > ${s}_command.csv

   echo Extracting autopilot status
   rostopic echo -b ${f} -p /aqua/autopilot/status > ${s}_status.csv

   echo Extracting target poses
   rostopic echo -b ${f} -p /aqua/target_pose | sed "s|/latest_fix||g" > ${s}_target_pose.csv 

   echo Extracting aqua states
   rostopic echo -b ${f} -p /aqua/state > ${s}_state.csv

   echo Extracting depths
   rostopic echo -b ${f} -p /aqua/filtered_depth > ${s}_filtered_depth.csv
done
