#!/bin/bash
#
# Must execute in a subdir of ros/boldly/
#

# Must be space separated.
WORLDS="hospital"

function nap {
  if [ -z $1 ]
  then
    echo "nap called with no argument!"
    exit 1
  fi
  echo "Sleeping for $1 seconds..."
  sleep $1
}

function do_trial {
  rm -f ~/.ros/poses_output

  # Start processes
  roslaunch explore_stage stage_current.launch &> ../explore_stage/current.log &

  # Sleep a bit since processes take a few seconds to start
  nap 30

  FINISHED=0
  FINISHED_BADLY=0
    while [ $FINISHED == "0" ]
    do
      CURR_LAST_LINE=`tail -n 1 ~/.ros/poses_output`
      CURR_LAST_LINE_ARRAY=($CURR_LAST_LINE)
      CURR_MAX_DIST=${CURR_LAST_LINE_ARRAY[3]}

      echo ""
      echo "Checking if we have finished..."
      echo "On trial #$COUNT for position $POSE_INDEX ($XPOS, $YPOS), rating type $j on $MAPNAME."
      echo "  Current max distance: $CURR_MAX_DIST, needed max distance: $MAXDIST."

      # Check if we're done due to max distance hit
      BOOL_GT=`perl -e 'die unless @ARGV == 2; if ($ARGV[0] >= $ARGV[1]) { print 1 } else { print 0 }' $CURR_MAX_DIST $MAXDIST`
      if [ "$BOOL_GT" == 1 ]
      then
        echo "Finished trial due to hit max distance."
        FINISHED=1
      fi

      # Check if done due to explore process death
      RUNNING=`ps -ef | grep bin/explore | grep -v grep`
      if [ -z "$RUNNING" ]
      then
        echo "Finished trial due to determined explore is no longer running."
        FINISHED=1
        FINISHED_BADLY=1
      fi

      # Check if done due to robot getting stuck
      BOOL_STUCK=`../explore_stage/scripts/is_stuck.pl`;
      if [ "$BOOL_STUCK" == 1 ]
      then
        echo "Finished trial due to determining robot is stuck."
        FINISHED=1
        FINISHED_BADLY=1
      fi

      # Wait a bit before checking again (to avoid busy loop)
      nap 10
    done

    # Kill all processes
    killall -9 roslaunch
    killall -9 explore
    killall -9 move_base
    killall -9 stage
    killall -9 stageros
    killall -9 gmapping
    killall -9 slam_gmapping

    if [ $FINISHED_BADLY == "0" ]
    then
      SUCCESSFUL_TRIAL=1
    fi
}

# For each world
for MAPNAME in $WORLDS
do
  POSE_INDEX=0
  # for each starting pose
  while read line # this gets filename at bottom of loop
  do
    #read in from positions
    line_array=($line)
    XPOS=${line_array[0]}
    YPOS=${line_array[1]}
    MAXDIST=${line_array[3]}

    # Set robot initial pose in world file
    cat ../bosch_demos/bosch_worlds/$MAPNAME.world.top > ../bosch_demos/bosch_worlds/current.world
    echo "pose [$XPOS $YPOS 0 0]" >> ../bosch_demos/bosch_worlds/current.world
    cat ../bosch_demos/bosch_worlds/$MAPNAME.world.bottom >> ../bosch_demos/bosch_worlds/current.world

    # Run a trial for each rating type
    for j in `seq 0 2`
    do
      echo "Using rating type $j at pose $POSE_INDEX ($XPOS, $YPOS) in $MAPNAME."
      FILENAME=../data/$MAPNAME/$POSE_INDEX-$j-$MAPNAME
      echo "Will save data in $FILENAME."

      # Check if we already have data for this set
      if [ -e $FILENAME ]
      then
        echo "We already have data for this set ($FILENAME). Continuing to next..."
        continue
      fi

      # Set which rating type to use in explore XML
      cat ../explore_stage/explore_slam.xml.top > ../explore_stage/current_explore_slam.xml
      echo "<param name=\"rating_type\" value=\"$j\" />" >> ../explore_stage/current_explore_slam.xml
      cat ../explore_stage/explore_slam.xml.bottom >> ../explore_stage/current_explore_slam.xml

      SUCCESSFUL_TRIAL=0
      COUNT=0
      while [ $SUCCESSFUL_TRIAL == "0" ]
      do
        echo "Initiating trial #$COUNT for position $POSE_INDEX ($XPOS, $YPOS) with rating $j for $MAPNAME."
        do_trial

        # Wait for processes to cleanly die before beginning next trial
        nap 30

        COUNT=$(expr $COUNT + 1)
      done

      # Good trial data
      mv ~/.ros/poses_output $FILENAME
    done
    POSE_INDEX=$(expr $POSE_INDEX + 1)

  done < <(cat ../data/$MAPNAME/positions)
done
