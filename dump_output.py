
import sys

import pyopenocd.nm as nm
import pyopenocd.ocd as ocd

st = nm.SymbolTable("build/filter-benchmarks.elf")
input_addr, _ = st.lookup("filterInputRaw")
output_addr, _ = st.lookup("filterOutput")

print('input:', hex(input_addr))
print('output:', hex(output_addr))

if len(sys.argv) < 2:
    print('argument expected')
    sys.exit(-1)

with ocd.OpenOCD() as o:
    o.halt()
    o.dump_image(sys.argv[1], output_addr, 4000*4)
    o.resume()
