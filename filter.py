
from time import sleep

import pyopenocd.ocd as ocd
import pyopenocd.nm as nm

filter_funs = [
    'arm_fir_f32',
    'arm_fir_q31',
    'arm_biquad_cascade_df2T_f32',
    'arm_biquad_cas_df1_32x64_q31',
]

st = nm.SymbolTable("build/filter-benchmarks.elf")
fir_f32_addr, _ = st.lookup("arm_fir_f32")
#fir_q31_addr, _ = st.lookup("arm_fir_q31")
output_addr, _ = st.lookup("filterOutput")
#reset_pin_addr, _ = st.lookup("LL_GPIO_ResetOutputPin")
delay_addr, _ = st.lookup("LL_mDelay")
to_float_addr, _ = st.lookup("arm_q31_to_float")

print('FIR f32:', hex(fir_f32_addr))
#print('FIR q31:', hex(fir_q31_addr))
print('output:', hex(output_addr))

with ocd.OpenOCD(verbose=True) as o:
    o.reset(halt=True)


    addr, _ = st.lookup(filter_funs[0])
    o.set_breakpoint(addr)
    o.resume()
    # Wait until we hit the start of the filter
    o.wait_halt()
    o.delete_breakpoint(addr)

    addr, _ = st.lookup(filter_funs[1])
    o.set_breakpoint(addr)
    # Run the filter and stop before the next one
    o.resume()
    o.wait_halt()
    o.delete_breakpoint(addr)
    o.dump_image(filter_funs[0]+'.bin', output_addr, 4004*4)

    addr, _ = st.lookup(filter_funs[2])
    o.set_breakpoint(addr)
    # Run the filter and stop before the next one
    o.resume()
    o.wait_halt()
    o.delete_breakpoint(addr)
    o.dump_image(filter_funs[1]+'.bin', output_addr, 4004*4)

    addr, _ = st.lookup(filter_funs[3])
    o.set_breakpoint(addr)
    # Run the filter and stop before the next one
    o.resume()
    o.wait_halt()
    o.delete_breakpoint(addr)
    o.dump_image(filter_funs[2]+'.bin', output_addr, 4004*4)

    addr, _ = st.lookup('LL_mDelay')
    o.set_breakpoint(addr)
    # Run the filter and stop before the next one
    o.resume()
    o.wait_halt()
    o.delete_breakpoint(addr)
    o.dump_image(filter_funs[3]+'.bin', output_addr, 4004*4)


    o.resume()

