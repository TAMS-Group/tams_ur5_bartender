#!/bin/bash

# Start Browser

# use default browser
fileFragment=$"*tams_ur5_bartender_gui/public_html/index.html"
fullPath=$(find ../ -path $fileFragment)
xdg-open $fullPath &
