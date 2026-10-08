

module instruction_decoder (
    input  [31:0] inst,
    output [3:0]  scratch_zone,  // inst[31:28] — FP scratch base selector
    output [3:0]  op_code,       // 4'b0001=ADD  4'b0010=MUL  4'b0011=FPMUL
    output [7:0]  col_addr_A,
    output [7:0]  col_addr_B,
    output [7:0]  col_addr_D
);
    assign scratch_zone = inst[31:28];
    assign op_code      = inst[27:24];
    assign col_addr_A   = inst[23:16];
    assign col_addr_B   = inst[15:8];
    assign col_addr_D   = inst[7:0];
endmodule
