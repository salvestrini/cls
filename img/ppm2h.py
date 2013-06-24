#!/usr/bin/python

import sys
import os
import array
import string

try:
        filename_input  = sys.argv[1]
        filename_output = sys.argv[2]

        assert(filename_input  is not None)
        assert(filename_output is not None)

        fh = open(filename_input)
        
        strz = '0x'
        count = 0
        temp = ''
        
        for x in fh:
            if count == 2:
                strz = strz + '\n' + ',0x'
                count = 0
            else:
                count = count + 1
            numeric = int(x)/17
            temp    = hex(numeric)
            strz    = strz + temp[2]

        fh = open(filename_output, 'w')
        fh.write(strz + '')
except Exception, e:
        print("Got exception: " + str(e))
