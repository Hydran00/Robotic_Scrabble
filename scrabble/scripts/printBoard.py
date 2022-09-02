#! /usr/bin/env python
from texttable import Texttable
import os
"""@package docstring
Documentation for this module.
Print board state into stdout
"""

f = open(os.path.expanduser('~')+"/catkin_ws/src/scrabble/scripts/board.txt")
board = (f.read()).split('\n')
t = Texttable()
fl = []
fl.append(['0','01','02','03','04','05','06','07','08','09','10','11','12','13','14','15','16','17'])
for i,s in enumerate(board):
    arr = [i+1]
    for j,c in enumerate(s):
        if s[j] == '#':
            arr += " "
        else:
            arr += s[j]
    fl.append(arr)
t.add_rows(fl)
print(t.draw())
