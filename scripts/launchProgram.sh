#!/bin/sh

until '/home/pioneer/ros/src/p3dx_apps/scripts/launch_basic.sh'; do
    echo "Respawning program.."
    sleep 1
done
