#!/bin/bash
# Must execute in explore_stage dir

MAPPATH="hospital-section"
MAPNAME="hospital"
INDEX=0

#for each starting pose
for line in `cat ../data/$MAPNAME/positions`
do
  #read in from positions
  line_array=($line)
  $XPOS=${line_array[0]}
  $YPOS=${line_array[1]}
  $MAXDIST=${line_array[3]}

  #set pose
  cat ../bosch_demos/bosch_worlds/$MAPPATH.world.top > used.world
  echo "pose [$XPOS $YPOS 0 0]" >> used.world
  cat ../bosch_demos/bosch_worlds/$MAPPATH.world.bottom >> used.world

  #for each rating
  for j in `seq 0 2`
  do
    
    #set rating
    cat world_configs/stage_hospital_section/explore_slam.xml.top > world_configs/stage_hospital_section/explore_slam.xml
    echo "<param name=\"rating_type\" value=\"$j\" />" >> world_configs/stage_hospital_section/explore_slam.xml
    cat world_configs/stage_hospital_section/explore_slam.xml.bottom >> world_configs/stage_hospital_section/explore_slam.xml

    #run
    roslaunch explore_stage stage_hospital_section.launch &

    FINISHED=0
    #wait for finish
    while [ $FINISHED = 0 ]
    do
      CURR_LAST_LINE=`tail -n 1 ~/.ros/poses_output`
      CURR_LAST_LINE_ARRAY=($CURR_LAST_LINE)
      CURR_MAX_DIST=${CURR_LAST_LINE_ARRAY[3]}

      if [ "$CURR_MAX_DIST" -gt "$MAXDIST" ]
      then
        FINISHED=1
      fi

      RUNNING=`ps -ef | grep explore`
      if [ -z "$RUNNING" ]
      then
        FINISHED=1
      fi
    done

    #end it all
    killall -9 explore
    killall -9 move_base
    killall -9 stage
    killall -9 stageros
    killall -9 gmapping
    
    #move and rename output
    mv ~/.ros/poses_output ../data/$MAPNAME/$INDEX-$j-$MAPNAME
  done
  INDEX=$INDEX+1
done
