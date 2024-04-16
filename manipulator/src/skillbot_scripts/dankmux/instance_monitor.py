import os
import time
import sys
import argparse

def move (y, x):
    print("\033[%d;%dH" % (y, x))
# python3 f; echo $? >> file & echo $! > file && fg
# ps -o stat --no-headerss --pid <pid>
# (trap 'echo $? >> file' 2; python3 f; echo $? >> file; trap 2) & echo $! > file && fg 
def save_cursor_pos():
    print('\033[s', end='')
def restore_cusrsor():
    print('\033[u', end='')

def parse_args():
    parser = argparse.ArgumentParser('dankmux instance monitor')
    parser.add_argument('session', help='name or number of tmux session that will be monitored')
    parser.add_argument('pdexpath', help='path to directory where dankmux2 create files with pid and exit codes')
    rs = parser.parse_args()
    return rs

def add_line(log, name, return_code, width):
    status = {
         -2: 'UNKNWN',
         -1: 'RUNNING',
          0: 'END', 
          1: 'DEAD',
        130: 'KILLED', 
    }
    if not status.get(return_code):
        status = status[-2]
    else:
        status = status[return_code]

    line_max = int(40*widht/100)
    print(f'[{status:^{len("RUNNING")}}] {name[:line_max]:^{line_max}} | {log[:line_max]:^{line_max}} | {return_code:^3}')
    
i = 0
anim2= '🕐🕑🕒🕓🕔🕕🕖🕗🕘🕙🕚🕛'
anim = ['\\','|', '/', '-', '|', '/', '-']

while True:
    widht, height = os.get_terminal_size()
    save_cursor_pos()
    print(f'[{anim2[i % len(anim2)]}]')
    add_line('hi'*82000, 'hi2', 1, widht)
    add_line('hi', 'hi2', 1, widht)
    restore_cusrsor()
    i += 1
    time.sleep(0.1)
    # raise Exception('fsdf')
    
    