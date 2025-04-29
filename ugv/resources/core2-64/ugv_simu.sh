#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./uavcoop_ugv_rt
else
	EXEC=./uavcoop_ugv_nrt
fi

$EXEC -n uav_0 -g ugv -t ugv_simu -a 127.0.0.1 -p 9000 -l /tmp -x setup_simu.xml -d 20001
