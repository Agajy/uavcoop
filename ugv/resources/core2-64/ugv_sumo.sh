#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./test_fleet_ugv_rt
else
	EXEC=./test_fleet_ugv_nrt
fi

$EXEC -n ugv -t sumo -a 127.0.0.1 -p 9000 -l /tmp -x setup_sumo.xml
