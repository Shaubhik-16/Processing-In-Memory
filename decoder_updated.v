
module instruction_decoder(
    input [31:0] inst,
    output [3:0] op_code, 
    output [7:0] col_addr_A,
    output [7:0] col_addr_B,
    output [7:0] col_addr_D
);
      // Instruction [27:24] = Opcode (4 bits)
    assign enable   = inst[31:28];
    assign op_code    = inst[27:24];
    assign col_addr_A = inst[23:16];
    assign col_addr_B = inst[15:8];
    assign col_addr_D = inst[7:0];
endmodule
