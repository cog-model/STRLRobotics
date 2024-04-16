#!/usr/bin/python3
import libtmux
import argparse
import json
import math
import os
import rospkg
from typing import List
rospak = rospkg.RosPack()
pkg_path = rospak.get_path('skillbot_scripts')
def parsing_args():
    parser = argparse.ArgumentParser('dankmuxcfg', description='programm for creating or manage tmux sessinon in weird way')
    parser.add_argument('cfg', nargs='?',  help=f'path to cfg, by default search file in {os.path.join(pkg_path,"/config/")}, default value \'default.json\'')
    parser.add_argument('-d', action='store_true', help='start session detached')
    arguments = parser.parse_args()
    if arguments.cfg is None:
        arguments.cfg = os.path.join(pkg_path,'config/default.json')
    
    return arguments

def split_window(window : libtmux.window.Window, shape1, shape2) -> List[libtmux.pane.Pane]:
    scale1 =100/(shape1)
    scale2 = 100/(shape2)
    panes_tmux = [window.attached_pane]
    for i in range(shape1-1):
        panes_tmux.append(window.split_window(vertical=True, percent=scale1))
        pane: libtmux.pane.Pane = panes_tmux[-1]
    new_panes = []
    for pane in panes_tmux:
        for i in range(shape2-1):
            pane : libtmux.pane.Pane = pane
            new_panes.append( pane.split_window(vertical=False, percent=scale2))

    panes_tmux += new_panes
    return panes_tmux

def load_window(session : libtmux.session.Session, window_cfg):
    shape2 = int(math.sqrt(len(window_cfg))) 
    shape1 = len(window_cfg)//shape2

    panes_tmux = split_window(session.attached_window, shape1, shape2)
    
    for pane, cmd in zip(panes_tmux, window_cfg):
        endcmd = cmd['cmd'].split(';')[-1]
        pane.cmd('select-pane', '-T', f'{endcmd}')
        print(f'starting {endcmd}')
        pane.send_keys(cmd['cmd'], suppress_history=True)
        pane.enter()
        
    

def main():
    arguments = parsing_args()
    if os.path.exists(arguments.cfg):
        cfg_path = arguments.cfg
    elif os.path.exists(os.path.join(pkg_path, 'config', arguments.cfg)):
        cfg_path = os.path.join(pkg_path, 'config', arguments.cfg)
        
    with open(cfg_path, 'r') as fd:
            config : dict = json.load(fd)
    for name, windows in config.items():
        tmux_server = libtmux.server.Server()
        if tmux_server.has_session(name):
            tmux_server.kill_session(name)
        tmux_session =  tmux_server.new_session(name)
        for window_name, panes in windows.items():
            tmux_session.new_window(window_name, attach=True)
            load_window(tmux_session, panes)
          
        if not arguments.d:
            tmux_server.attach_session(name)
    print(f'tmux session started, attach with:\ntmux attach-session -t {name}')
        
if __name__ == '__main__':
    main()