#!/usr/bin/python

'''
 Copyright [2017] Max Planck Society. All rights reserved.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

'''
Use example: ipython
             run display.py -i <path_to_datafile>
'''

import sys, getopt
from helpers import Graphics

def main(argv):
    inputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print 'display.py -i <path_to_datafile>'
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print 'display.py -i <path_to_datafile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg

    motion = Graphics()
    motion.show_motion(inputfile)

if __name__ == "__main__":
   main(sys.argv[1:])
