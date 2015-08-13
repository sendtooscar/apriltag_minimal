#!/bin/sh


if [ $# -eq 0 ] ; then
    /bin/echo "Entering build environment at /home/oscar/ros_workspace/apriltag_minimal/build"
    . /home/oscar/ros_workspace/apriltag_minimal/build/setup.sh
    $SHELL -i
    /bin/echo "Exiting build environment at /home/oscar/ros_workspace/apriltag_minimal/build"
else
    . /home/oscar/ros_workspace/apriltag_minimal/build/setup.sh
    exec "$@"
fi


