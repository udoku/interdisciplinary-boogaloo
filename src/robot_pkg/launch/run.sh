#!/bin/bash

export ROSCONSOLE_FORMAT='[${severity}]: ${message}'

# This script will start all four processes and an extra window to code in.
#   To stop everything hit F12 (or $ tmux kill-session from another terminal)
#   To leave everything running (detached): Ctrl-b followed by d
#   Everything else is contained in the tmux documentation
#
# Usage:
# ./run.sh task [folder]
#  - task: A string that is passed to the commander to tell it what task to run
#  - folder: A string that sets the folder name for logging this run. If ommited,
#            nothing will be logged.

# Use the right virualenv

# Check to make sure the current python version is correct by checking the
# opencv version
if [ `python -c 'import cv2; print(1 if cv2.__version__ == "3.4.2" else 0)'` ]
    then
        python_exe=`which python`
        python_dir=`dirname $python_exe`
        VIRTUAL_ENV_LOC=$python_dir/activate

# If its not, fall back to the original correct one.
    else
        echo "Please use a venv with opencv 3.4.2"
        exit
fi

# Make sure $RVR_ROOT has been initialized (running the binaries requires it)
if [ -z $RVR_ROOT ]
    then
        echo "Please run 'source config.sh' in the src folder first"
        exit
fi

if [ ! -n "$2" ]
    then
    read -p "Will not log! Are you sure? [y/n] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit
    fi
fi

export RECORD=0
# Put the location of the log folder for this run into an env variable for the
# program to fetch
if [ -n "$2" ]
    then
        export RECORD=1
        export LOG_FILE=$RVR_ROOT/../../../robot_logs/$(date +"%F__%H-%M-%S")__$2.bag
fi



echo Setting up terminal...

# Create a new session and 2 window named status and logger, with 4 panes (0,..,3)
# The logger window has 3 panes, one for logger, and three for streaming.
SHELL=/bin/bash tmux new-session -d -s boogaloo -n status
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 2    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 2    # select the top-right  pane
# tmux splitw -h -p 33 # split it into two halves
tmux split-window -h -l 28

tmux selectp -t 0    # go back to the first pane
tmux neww -n roscore_window # New window for roscore

echo Starting roscore

tmux send-keys \
    "roscore" Enter

# sleep 2

tmux selectw -t 0
# tmux split-window -h -t boogaloo:status
# tmux split-window -v -t boogaloo:status.0
# tmux split-window -v -t boogaloo:status.1

echo Setting up options...

# Make these windows nice to use
tmux set-window-option -t boogaloo mode-mouse on
tmux set-window-option -t boogaloo mouse on
tmux set-option -t boogaloo mouse-select-pane on
tmux set-option -t boogaloo mouse-select-window on
tmux set-option -t boogaloo mouse-resize-pane on
tmux bind-key -n F12 kill-session
tmux bind-key -n C-q kill-session
# tmux select-layout even-vertical
# tmux select-layout tiled

echo Starting hardware...

echo -n "Rvr root: "
echo $RVR_ROOT

if [ ! -z $ROS_IP ]
then
    if [ -z "$IGNORE_HARDWARE" ]
    then
        # Set up each pane with the appropraiate process
        tmux send-keys -t boogaloo:status.0 \
        "export ROS_IP=$ROS_IP" Enter \
        "cd $RVR_ROOT && source config.sh" Enter \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "cd $RVR_ROOT/../ && roslaunch --wait launch/hardware.launch" Enter

    else
    echo "Ignoring Hardware"
        tmux send-keys -t boogaloo:status.0 \
            "echo Hardware Disabled" Enter \ 
    fi

    echo Starting vision...

    tmux send-keys -t boogaloo:status.1 \
        "export ROS_IP=$ROS_IP" Enter \
        "cd $RVR_ROOT && source config.sh" Enter \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "source $VIRTUAL_ENV_LOC" Enter \
        "cd $RVR_ROOT/../ && roslaunch --wait launch/vision.launch RECORD:=$RECORD LOG_FILE:=$LOG_FILE" Enter

    echo Starting mobility...

    tmux send-keys -t boogaloo:status.2 \
        "export ROS_IP=$ROS_IP" Enter \
        "cd $RVR_ROOT && source config.sh" Enter \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "cd $RVR_ROOT/../ && roslaunch --wait launch/mobility.launch" Enter

    echo Starting robot state echo...

    tmux send-keys -t boogaloo:status.3 \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "export ROS_IP=$ROS_IP" Enter \
        "sleep 5" Enter \
        "rostopic echo -c /robot_state" Enter 
    echo Attaching...

    echo Starting commander...

    tmux send-keys -t boogaloo:status.4 \
        "export ROS_IP=$ROS_IP" Enter \
        "cd $RVR_ROOT && source config.sh" Enter \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "cd $RVR_ROOT/../ && roslaunch --wait launch/commander.launch commander_name:=$1" Enter
else
    if [ -z "$IGNORE_HARDWARE" ]
    then
        # Set up each pane with the appropraiate process
        tmux send-keys -t boogaloo:status.0 \
        "unset ROS_IP" Enter \
        "cd $RVR_ROOT && source config.sh" Enter \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "cd $RVR_ROOT/../ && roslaunch --wait launch/hardware.launch" Enter

    else
    echo "Ignoring Hardware"
        tmux send-keys -t boogaloo:status.0 \
        "unset ROS_IP" Enter \
            "echo Hardware Disabled" Enter \ 
    fi

    echo Starting vision...

    tmux send-keys -t boogaloo:status.1 \
        "unset ROS_IP" Enter \
        "cd $RVR_ROOT && source config.sh" Enter \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "source $VIRTUAL_ENV_LOC" Enter \
        "cd $RVR_ROOT/../ && roslaunch --wait launch/vision.launch RECORD:=$RECORD LOG_FILE:=$LOG_FILE" Enter

    echo Starting mobility...

    tmux send-keys -t boogaloo:status.2 \
        "unset ROS_IP" Enter \
        "cd $RVR_ROOT && source config.sh" Enter \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "cd $RVR_ROOT/../ && roslaunch --wait launch/mobility.launch" Enter

    echo Starting robot state echo...

    tmux send-keys -t boogaloo:status.3 \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "unset ROS_IP" Enter \
        "sleep 5" Enter \
        "rostopic echo -c /robot_state" Enter 
    echo Attaching...

    echo Starting commander...

    tmux send-keys -t boogaloo:status.4 \
        "unset ROS_IP" Enter \
        "cd $RVR_ROOT && source config.sh" Enter \
        "cd $RVR_ROOT/../../../ && source devel/setup.bash" Enter \
        "cd $RVR_ROOT/../ && roslaunch --wait launch/commander.launch commander_name:=$1" Enter
fi


# Enter the tmux session that was created
tmux attach
