
import sys

import pyopenocd.nm as nm
import pyopenocd.ocd as ocd

st = nm.SymbolTable("build/filter-benchmarks.elf")
input_addr, _ = st.lookup("filterInputRaw")
output_addr, _ = st.lookup("filterOutput")

print('input:', hex(input_addr))
print('output:', hex(output_addr))

with ocd.OpenOCD() as o:
    if o.verify_image(sys.argv[1], input_addr):
        print('Verify OK')
    else:
        print('Verify failed')
