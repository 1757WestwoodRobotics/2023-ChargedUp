#!/bin/bash 

timeout --preserve-status 10s python robot.py sim --nogui

# exit code 143 is exited by SIGTERM which means simulator was killed by timeout but otherwise started
if [ $? -eq 143 ] 
then
    exit 0
fi

exit 1
