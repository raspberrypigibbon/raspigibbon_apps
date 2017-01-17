#!/bin/bash
for i in `seq 0 9`; do
	TARGET=192.168.111.8$i
	sshpass -p 'ubuntu' ssh ubuntu@$TARGET "./raspiarm/$1 || exit" &
	echo $TARGET $?
	sleep 0.1
done
