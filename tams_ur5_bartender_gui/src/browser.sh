#!/bin/bash

# Start Browser

# use default browser
fileFragment=$"*project16_gui/public_html/index.html"
fullPath=$(find ../ -path $fileFragment)
xdg-open $fullPath &


# uncomment this again, if things from above don't work
#if false; then
#
#if [ -d ~/ros/src/project16/project16_gui/public_html ] ; then
#    firefox ~/ros/src/project16/project16_gui/public_html/index.html
#fi
#
#if [ -d ~/pr16_ws/src/project16_gui/public_html ] ; then
#    firefox ~/pr16_ws/src/project16_gui/public_html/index.html
#fi
#
#fi
