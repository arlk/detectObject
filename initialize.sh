#!/bin/bash

usage()
{
cat << EOF

 Usage     : ./initialize.sh  --[options]

This script auto-sets parameters and starts running mjpeg-streamer.

OPTIONS:
 --nogui   : This stops the xserver on Odroid, improving performance.
	     Restarts xserver after script execution
 --gui  : Retains gui.

EOF
}

if ! [[ "$*" =~ "--nogui" ]]
then
	if [[ "$*" =~ "--gui" ]]
	then
		usage
		exit 0
	fi	
	export LD_LIBRARY_PATH=.
	sudo modprobe uvcvideo
	v4l2-ctl -c focus_auto=0
	./detect.py
else
	sudo service lightdm stop
	export LD_LIBRARY_PATH=.
	sudo modprobe uvcvideo
	v4l2-ctl -c focus_auto=0
	./detect.py
	sudo service lightdm start
fi