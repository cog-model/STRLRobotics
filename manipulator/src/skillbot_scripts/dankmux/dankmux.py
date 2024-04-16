#!/usr/bin/python3
import libtmux
import argparse
import math
import sys
import subprocess
def parsing_args():
    parser = argparse.ArgumentParser('dankmux', description='programm for creating or manage tmux sessinon in weird way')
    parser.add_argument('-n', '--name', help='set tmux session name', default='dankmux')
    parser.add_argument('-m', '--modify', help='modify mode to create create new windows in existing session', action='store_true')
    parser.add_argument('-d', '--detached', action='store_true', default=False, help='run terminal in background')
    parser.add_argument('shape1', nargs='?', type=int, help='horizontal amount of new windows or summary amount of new windows if [shape2] not specified')
    parser.add_argument('shape2', nargs='?', type=int, help='vertical amount of new windows')
    arguments = parser.parse_args()
    if arguments.shape1 is None: 
        arguments.remainder = 0
        arguments.shape1 = 2
        arguments.shape2 = 2
    elif arguments.shape2 is None:
        arguments.remainder = arguments.shape1 % int(math.sqrt(arguments.shape1)) 
        arguments.shape2 = int(math.sqrt(arguments.shape1)) 
        arguments.shape1 = arguments.shape1//arguments.shape2
    else:
        arguments.remainder = 0
    return arguments

def main():
    arguments = parsing_args()
    tmux_server = libtmux.server.Server(socket_path='/tmp/tmux-1000/default')
    tmux_session = tmux_server.attach_session(arguments.name) if arguments.modify else tmux_server.new_session()
    print(f'Attach to this session use:')
    print(f'tmux attach-session -t {tmux_session.name}')
    scale1 =100/(arguments.shape1)
    scale2 = 100/(arguments.shape2)
    panes = [tmux_session.attached_window.attached_pane]
    for i in range(arguments.shape1-1):
        panes.append(tmux_session.attached_window.split_window(vertical=True, percent=scale1))
    for pane in panes:
        for i in range(arguments.shape2-1):
            pane : libtmux.pane.Pane = pane
            pane.split_window(vertical=False, percent=scale2)
            
    if not arguments.detached:
        subprocess.run([f'tmux attach-session -t {tmux_session.name}'], stderr=sys.stderr, stdout=sys.stdout, stdin=sys.stdin, shell=True)
        
    

if __name__ == '__main__':
    main()