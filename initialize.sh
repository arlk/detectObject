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
	if [[ "$*" == "--gui" ]]
	then
		export LD_LIBRARY_PATH=.
		sudo modprobe uvcvideo
		v4l2-ctl -c focus_auto=0
        v4l2-ctl -c sharpness=180
	else
		usage
		exit 0
	fi
else
	sudo service lightdm stop
	export LD_LIBRARY_PATH=.
	sudo modprobe uvcvideo
	v4l2-ctl --device=0 -c focus_auto=0
    v4l2-ctl --device=0 -c sharpness=230
fi
