#!/bin/bash

docker build . \
             -t x64_noetic/strategic_node \
             --build-arg UID=${UID} \
             --build-arg GID=${UID}
