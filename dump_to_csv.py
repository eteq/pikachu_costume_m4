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
timestamps = []
fiforems = []

with args.input.open('rb') as f:
    n = 0
    page = f.read(256)
    while page != b'':
        ts = page[:4]
        fiforem = page[-2:]
        offset = 4
        while (offset + 2*len(elem_names)) < 254:
            for nm in elem_names:
                elem_lists[nm].append(struct.unpack('<h', page[offset:(offset+2)])[0])
                offset += 2
            timestamps.append(struct.unpack('<I', ts)[0])
            fiforems.append(struct.unpack('<H', fiforem)[0])

        if len(page) != 256:
            print(f'Warning: page {n} is not 256 bytes long')
        n += 1
        page = f.read(256)
print(f'Read {n} pages fron {args.input}')  

print(f'Writing to csv file {args.output}')
with args.output.open('w') as f:
    f.write('TIME,' + ','.join(elem_names) + ',REMINFIFO\n')
    for i in range(len(elem_lists['qw'])):
        f.write(str(timestamps[i]/32768.) + ',')
        f.write(','.join([str(elem_lists[nm][i]) for nm in elem_names]))
        f.write(',' + str(fiforems[i]) + '\n')

