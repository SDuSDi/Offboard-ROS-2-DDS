#! /bin/bash
set -e 

touch ~/.tmux.conf
echo "set -g mouse" >> ~/.tmux.conf

./tmux/start.sh