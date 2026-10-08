`timescale 1ns / 1ps

module memory_bank #(parameter ds=3)(
    input            clk,
    input            rst,
    input [3:0]      op_code,
    input [7:0]      col_addr_A,
    input [7:0]      col_addr_B,
    input [7:0]      col_addr_D,
    output reg       op_done
);

    // 128 x 256 bit memory
    reg memory [0:127][0:255];

    // Row control
    reg carry [0:127];
    reg tag   [0:127];

    reg [1:0] add_state;
    reg [2:0] mul_state;
    reg [2:0] bit_pos;

    integer i;

    // Address calculation
    wire [7:0] A_curr_addr;
    wire [7:0] B_curr_addr;
    wire [7:0] D_curr_addr;

    assign A_curr_addr = ~op_code[1] ? (col_addr_A + bit_pos[1:0]) : (col_addr_A + bit_pos[0]);
    assign B_curr_addr = ~op_code[1] ? (col_addr_B + bit_pos[1:0]) : (col_addr_D + bit_pos[2:1]);
    assign D_curr_addr = ~op_code[1] ? (col_addr_D + bit_pos[1:0]) : (col_addr_D + bit_pos[2:1]);

    // SIMD operand selection
    wire [127:0] A_sel;
    wire [127:0] B_sel;
    wire [127:0] sum;
    wire [127:0] cout;

    genvar r;

    generate
        for(r=0; r<128; r=r+1) begin : SIMD_ROWS
            assign A_sel[r] = memory[r][A_curr_addr];
            assign B_sel[r] = memory[r][B_curr_addr];

            cram_full_adder FA (
                .a(A_sel[r]),
                .b(B_sel[r]),
                .cin(carry[r]),
                .s(sum[r]),
                .cout(cout[r])
            );
        end
    endgenerate


    always @(posedge clk or posedge rst) begin

        if(rst) begin

            for(i=0;i<128;i=i+1) begin
                carry[i] <= 0;
                tag[i]   <= 1;
            end

            add_state <= 0;
            mul_state <= 0;
            bit_pos   <= 0;
            op_done   <= 0;

        end
        else begin

            op_done <= 0;

            case(op_code)

            // ADDITION
            4'b0001: begin

                case(add_state)

                2'b00: begin
                    for(i=0;i<128;i=i+1)
                        carry[i] <= 0;

                    bit_pos   <= 0;
                    add_state <= 2'b01;
                end

                2'b01: begin
                    for(i=0;i<128;i=i+1) begin
                        if(tag[i])
                            memory[i][D_curr_addr] <= sum[i];

                        carry[i] <= cout[i];
                    end

                    bit_pos   <= 1;
                    add_state <= 2'b10;
                end

                2'b10: begin
                    for(i=0;i<128;i=i+1) begin
                        if(tag[i])
                            memory[i][D_curr_addr] <= sum[i];

                        carry[i] <= cout[i];
                    end

                    bit_pos   <= 2;
                    add_state <= 2'b11;
                end

                2'b11: begin
                    for(i=0;i<128;i=i+1) begin
                        if(tag[i])
                            memory[i][D_curr_addr] <= sum[i];

                        carry[i] <= cout[i];
                    end

                    add_state <= 0;
                    op_done   <= 1;
                end

                endcase
            end


            // MULTIPLICATION
            4'b0010: begin

                case(mul_state)

                3'b000: begin

                    for(i=0;i<128;i=i+1) begin
                        carry[i] <= 0;
                        tag[i]   <= 0;

                        memory[i][col_addr_D]   <= 0;
                        memory[i][col_addr_D+1] <= 0;
                        memory[i][col_addr_D+2] <= 0;
                        memory[i][col_addr_D+3] <= 0;
                    end

                    mul_state <= 3'b001;
                end

                3'b001: begin
                    for(i=0;i<128;i=i+1)
                        tag[i] <= memory[i][col_addr_B];

                    bit_pos   <= 0;
                    mul_state <= 3'b010;
                end

                3'b010: begin
                    for(i=0;i<128;i=i+1) begin
                        if(tag[i])
                            memory[i][D_curr_addr] <= sum[i];

                        carry[i] <= cout[i];
                    end

                    bit_pos   <= 3;
                    mul_state <= 3'b011;
                end

                3'b011: begin
                    for(i=0;i<128;i=i+1) begin
                        if(tag[i])
                            memory[i][D_curr_addr] <= sum[i];

                        carry[i] <= cout[i];
                    end

                    mul_state <= 3'b100;
                end

                3'b100: begin
                    for(i=0;i<128;i=i+1)
                        tag[i] <= memory[i][col_addr_B+1];

                    bit_pos   <= 2;
                    mul_state <= 3'b101;
                end

                3'b101: begin
                    for(i=0;i<128;i=i+1) begin
                        if(tag[i])
                            memory[i][D_curr_addr] <= sum[i];

                        carry[i] <= cout[i];
                    end

                    bit_pos   <= 5;
                    mul_state <= 3'b110;
                end

                3'b110: begin
                    for(i=0;i<128;i=i+1) begin
                        if(tag[i])
                            memory[i][D_curr_addr] <= sum[i];

                        carry[i] <= cout[i];
                    end

                    bit_pos   <= 7;
                    mul_state <= 3'b111;
                end

                3'b111: begin
                    for(i=0;i<128;i=i+1)
                        if(tag[i])
                            memory[i][D_curr_addr] <= carry[i];

                    mul_state <= 0;
                    op_done   <= 1;
                end

                endcase
            end

            endcase
        end
    end

endmodule
