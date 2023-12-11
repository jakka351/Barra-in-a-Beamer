#! /bin/bash
sudo killall cangen
cangen can0 -I 207 -D r -L 8 -g 100 &
cangen can0 -I 230 -D r -L 8 -g 100 &
cangen can0 -I 427 -D r -L 8 -g 100 &

