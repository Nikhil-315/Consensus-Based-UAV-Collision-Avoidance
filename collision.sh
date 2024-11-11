#!/bin/bash

echo "Select Scenario:"
echo "1. Head On"
echo "2. 135 deg"
echo "3. 90 deg"
echo "4. 45 deg"
read -p "Enter No.: " SCENARIO


case $SCENARIO in
    1)
        WORLD="head_on.world"
        SOURCE_FILE="ak_ws/src/collision_avoidance/src/head_on.csv"
        ;;
    2)
        WORLD="angle_135.world"
        SOURCE_FILE="ak_ws/src/collision_avoidance/src/angle_135.csv"
        ;;
    3)
        WORLD="angle_90.world"
        SOURCE_FILE="ak_ws/src/collision_avoidance/src/angle_90.csv"
        ;;
    4)
        WORLD="angle_45.world"
        SOURCE_FILE="ak_ws/src/collision_avoidance/src/angle_45.csv"
        ;;
    *)
        echo "Invalid selection. Exiting."
        exit 1
        ;;
esac


cp -f "$SOURCE_FILE" "ak_ws/src/collision_avoidance/src/waypoints.csv"
if [ $? -ne 0 ]; then
    echo "Error copying file. Exiting."
    exit 1
fi


BASE_COMMAND="cd && cd ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter"


gnome-terminal --tab --title="Simulation World" --command="bash -c 'cd ardupilot_gazebo/ && gazebo --verbose worlds/$WORLD'"
gnome-terminal --title="drone1" --command="bash -c '$BASE_COMMAND -f gazebo-drone1 -I0 -L Col --out=tcpin:0.0.0.0:8100; exec bash'"
gnome-terminal --title="drone2" --command="bash -c '$BASE_COMMAND -f gazebo-drone2 -I1 -L Col --out=tcpin:0.0.0.0:8200; exec bash'"

