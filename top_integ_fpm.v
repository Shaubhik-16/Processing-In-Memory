


`include "decoder_fpm.v"
`include "mem_bank_fpm.v"

module cram_top (
    input        clk,
    input        rst,
    input [31:0] inst,
    output       op_done
);
    wire [3:0] scratch_zone;  // inst[31:28] — FP scratch base selector
    wire [3:0] op_code;
    wire [7:0] A, B, D;

    instruction_decoder dec (
        .inst         (inst),
        .scratch_zone (scratch_zone),
        .op_code      (op_code),
        .col_addr_A   (A),
        .col_addr_B   (B),
        .col_addr_D   (D)
    );

    memory_bank bank (
        .clk          (clk),
        .rst          (rst),
        .op_code      (op_code),
        .scratch_zone (scratch_zone),
        .col_addr_A   (A),
        .col_addr_B   (B),
        .col_addr_D   (D),
        .op_done      (op_done)
    );

endmodule
