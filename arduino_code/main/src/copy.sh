#!/bin/bash
export src_dir=${HOME}/catkin_ws/src/brain/arduino_code/main

if [ "$1" = "back" ]; then
	cp *.ino *.h $src_dir
else
	cp ${src_dir}/*.ino ${src_dir}/*.h .
fi
