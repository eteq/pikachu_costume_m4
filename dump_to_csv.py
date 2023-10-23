#!/usr/bin/env python3

import struct
from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('input', type=Path)
parser.add_argument('output', type=Path)

args = parser.parse_args()

elem_names = 'qw,qx,qy,qz,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z'.split(',')
elem_lists = {nm:[] for nm in elem_names}

with args.input.open('rb') as f:
    n = 0
    block = f.read(256)
    while block != b'':
        hdr = block[0]
        if hdr == 1:
            # data block

            offset = 1
            while (offset + 2*len(elem_names)) < 256:
                for nm in elem_names:
                    
                    elem_lists[nm].append(struct.unpack('<h', block[offset:(offset+2)])[0])
                    offset += 2

            if len(block) != 256:
                print(f'Warning: block {n} is not 256 bytes long')
            n += 1
        block = f.read(256)
print(f'Read {n} blocks fron {args.input}')  

print(f'Writing to csv file {args.output}')
with args.output.open('w') as f:
    f.write(','.join(elem_names) + '\n')
    for i in range(len(elem_lists['qw'])):
        f.write(','.join([str(elem_lists[nm][i]) for nm in elem_names]) + '\n')

