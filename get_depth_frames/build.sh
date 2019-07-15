#!/bin/bash
cd ..
git fetch origin master
git reset --hard origin/master
cd get_depth_frames
rm -R build
mkdir build && cd build
cmake ..
make
if [ $? -eq 0 ]; then
    ./GetDepthFrames
fi
