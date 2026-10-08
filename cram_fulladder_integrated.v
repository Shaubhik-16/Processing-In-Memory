`timescale 1ns / 1ps

// =============================================================================
// cram_full_adder.v
//
// Combinational 1-bit full adder.
// This module was referenced in the original memory_bank but not provided
// separately.  Included here for completeness; if you already have it in
// your project simply omit this file.
// =============================================================================

module cram_full_adder (
    input  wire a,
    input  wire b,
    input  wire cin,
    output wire s,
    output wire cout
);
    assign s    = a ^ b ^ cin;
    assign cout = (a & b) | (b & cin) | (a & cin);
endmodule
