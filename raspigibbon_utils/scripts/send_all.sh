#!/bin/bash
for i in `seq 0 9`; do
	TARGET=192.168.111.8$i
	echo $TARGET
	sshpass -p 'ubuntu' scp $1 ubuntu@$TARGET:~/raspiarm/
	echo $TARGET $?
	sleep 1
done
