#! /bin/bash
if [ -f /proc/xenomai/version ];then
	EXEC=./uavcoop_simulator_rt
else
	EXEC=./uavcoop_simulator_nrt
fi

$EXEC -n uav -t x4 -p 9000 -a 127.0.0.1 -x simulator_uav_ugv.xml -o 10 -m $FLAIR_ROOT/flair-src/models -s $FLAIR_ROOT/flair-src/models/indoor_flight_arena.xml
