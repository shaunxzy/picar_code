#!/bin/bash
# if virtualenvwrapper.sh is in your PATH (i.e. installed with pip)
#source `which virtualenvwrapper.sh`
cd detect_stop_sign 
source /usr/local/bin/virtualenvwrapper.sh  # if it's not in your PATH
workon cv
python3 detect.py $1 $2
