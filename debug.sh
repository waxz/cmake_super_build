#!/bin/bash

#"sudo apt-get install valgrind"

path=$(date +'%H-%M-%S')
echo "write log to valgrind-$path.log"
G_SLICE=always-malloc G_DEBUG=gc-friendly  valgrind -v --tool=memcheck --leak-check=full --num-callers=40 --log-file=valgrind-$path.log $@
