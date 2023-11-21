#!/bin/bash -xe

logfile="log_$(date +'%H-%M-%S').log"

minicom -b 9600 -C logs/${logfile} -D /dev/ttyACM0
