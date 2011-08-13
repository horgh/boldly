#!/bin/bash
#
# Must execute in a subdir of ros/boldly/
#

MAPNAME="hospital"
INDEX=0

#for each starting pose
while read line # this gets filename at bottom of loop
do
  #read in from positions
  line_array=($line)
  XPOS=${line_array[0]}
  YPOS=${line_array[1]}
  MAXDIST=${line_array[3]}

  echo "Starting at $XPOS, $YPOS in $MAPNAME."

  #set pose
  cat ../bosch_demos/bosch_worlds/$MAPNAME.world.top > ../bosch_demos/bosch_worlds/current.world
  echo "pose [$XPOS $YPOS 0 0]" >> ../bosch_demos/bosch_worlds/current.world
  cat ../bosch_demos/bosch_worlds/$MAPNAME.world.bottom >> ../bosch_demos/bosch_worlds/current.world

  #for each rating
  for j in `seq 0 2`
  do
    rm -f ~/.ros/poses_output
    
    echo "Using rating type $j at $XPOS, $YPOS in $MAPNAME."

    #set rating
    cat ../explore_stage/explore_slam.xml.top > ../explore_stage/current_explore_slam.xml
    echo "<param name=\"rating_type\" value=\"$j\" />" >> ../explore_stage/current_explore_slam.xml
    cat ../explore_stage/explore_slam.xml.bottom >> ../explore_stage/current_explore_slam.xml

    #run
    roslaunch explore_stage stage_current.launch &> ../explore_stage/current.log &

    FINISHED=0
    #wait for finish
    while [ $FINISHED == "0" ]
    do
      CURR_LAST_LINE=`tail -n 1 ~/.ros/poses_output`
      CURR_LAST_LINE_ARRAY=($CURR_LAST_LINE)
      CURR_MAX_DIST=${CURR_LAST_LINE_ARRAY[3]}

      echo "Checking if we have finished (current max distance: $CURR_MAX_DIST, needed max distance: $MAXDIST"
      echo "  using rating scheme $j at $XPOS, $YPOS in $MAPNAME."

      BOOL_GT=`perl -e 'die unless @ARGV == 2; if ($ARGV[0] >= $ARGV[1]) { print 1 } else { print 0 }' $CURR_MAX_DIST $MAXDIST`
      if [ "$BOOL_GT" == 1 ]
      then
        FINISHED=1
      fi

      RUNNING=`ps -ef | grep bin/explore | grep -v grep`
      if [ -z "$RUNNING" ]
      then
        FINISHED=1
      fi

      # Wait a bit before checking again
      SECS=10
      echo "Sleeping for $SECS seconds..."
      sleep $SECS
    done

    #end it all
    killall -9 roslaunch
    killall -9 explore
    killall -9 move_base
    killall -9 stage
    killall -9 stageros
    killall -9 gmapping
    killall -9 slam_gmapping
    
    #move and rename output
    mv ~/.ros/poses_output ../data/$MAPNAME/$INDEX-$j-$MAPNAME

    SECS=30
    echo "Sleeping for $SECS seconds to ensure processes died..."
    sleep $SECS
  done
  INDEX=$INDEX+1
done < <(cat ../data/$MAPNAME/positions)
