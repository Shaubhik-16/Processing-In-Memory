Designed a 128×256 Compute SRAM in Verilog RTL based on Wang et al. (JSSC 2020), implementing the full Processing-in-Memory (PIM) hierarchy
from 8T transposable bit cells and pseudo-differential sense amplifiers through a SIMD bit-serial full-adder array with per-row carry and tag latches
for vector arithmetic. 
Extended the architecture to support 32-bit IEEE-754 floating-point multiplication entirely within the memory array, reusing the existing datapath
hardware with a caller-controlled scratch memory scheme and a multi-phase FSM covering sign, exponent, mantissa multiply, and normalisation
