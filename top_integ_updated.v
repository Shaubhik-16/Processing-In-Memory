module cram_top(
    input clk,
    input rst,
    input [31:0] inst,
    output op_done
);
    wire [3:0] op_code;  
    wire [7:0] A, B, D;

    instruction_decoder dec (
        .inst(inst),
        .op_code(op_code),
        .col_addr_A(A),
        .col_addr_B(B),
        .col_addr_D(D)
    );

    memory_bank bank (
        .clk(clk),
        .rst(rst),
        .op_code(op_code),
        .col_addr_A(A),
        .col_addr_B(B),
        .col_addr_D(D),
        .op_done(op_done)
    );
endmodule
