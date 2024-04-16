#!/usr/bin/bash
(tmux send-keys -t ={skillbot_session}: C-c) && (tmux a -t ={skillbot_session} || true)