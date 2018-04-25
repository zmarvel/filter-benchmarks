
import random as rand
import argparse as ap
import sys
from pathlib import Path

import numpy as np

rand.seed(649)

n = 32000//8
ys = np.array([rand.uniform(-1, 1) for _ in range(n)], dtype=np.float32)

parser = ap.ArgumentParser(description=('Noise generator for filter testing. '
                                        'Output a binary or a C source file.'))
parser.add_argument('-b', '--binary', action='store_true', default=True)
parser.add_argument('-c', '--c-source', action='store_true')
parser.add_argument('-l', '--little-endian', action='store_true')
parser.add_argument('-I', '--include-dir', type=str,
                    help="Header file destination directory")
parser.add_argument('out_file')

args = parser.parse_args()



out = Path(args.out_file)

c = args.c_source
if c:
    b = False
    hout_dir = Path(args.include_dir)
    hout = hout_dir / out.with_suffix('.h').name

if c and b:
    print('Cannot specify both --binary and --c-source.')
    parser.print_usage()
    sys.exit(-1)


#if not args.little_endian:
#    ys.byteswap(inplace=True)

C_TEMPLATE = """#include "{}"

const unsigned char filterInputRaw[{}*4] = {{
{}
}};

const float *const filterInput = (float *)filterInputRaw;"""

H_TEMPLATE = """#define FILTER_BLOCK_SIZE {}

extern const float *const filterInput;
"""
#extern const int filterBlockSize;
#const int filterBlockSize = {};

if c:
    #print([str(f) for f in ys[:5]])
    with out.open('w') as f:
        f.write(C_TEMPLATE.format(hout.name, n, ',\n'.join(map(hex, ys.tobytes()))))
    with hout.open('w') as f:
        f.write(H_TEMPLATE.format(n))
else:
    with out.open('wb') as f:
        f.write(ys.tobytes())
