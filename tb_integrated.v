`timescale 1ns / 1ps

// =============================================================================
// tb_integrated.v  —  Testbench for cram_top with 8T + SA integration
//
// Tests
//   1. 3-bit addition:  A=5 (101), B=6 (110) → sum=11 (011 + carry=1)
//   2. 2-bit multiplication: A=3 (11), B=2 (10) → product=6 (0110)
//
// Initialisation uses the HWL port on bit_cell_8t_array directly,
// accessed via the hierarchy path dut.bank.u_cells.
// =============================================================================

`include "top_integ_updated.v"

module tb_integrated;

    reg         clk, rst;
    reg [31:0]  inst;
    wire        op_done;

    // DUT
    cram_top dut (
        .clk     (clk),
        .rst     (rst),
        .inst    (inst),
        .op_done (op_done)
    );

    // 10 ns clock
    initial clk = 0;
    always #5 clk = ~clk;

    // -------------------------------------------------------------------
    // Helper: write one bit into all 128 rows of a given column
    //   via the HWL port on u_cells using force/release (portable)
    // -------------------------------------------------------------------
    task write_col_all_rows;
        input [7:0]  col;
        input        val;
        integer rr;
        begin
            for (rr = 0; rr < 128; rr = rr + 1)
                dut.bank.u_cells.storage[rr][col] = val;
        end
    endtask

    // -------------------------------------------------------------------
    // Helper: read back bit from all 128 rows of a given column
    //   Returns 1 only if ALL rows hold expected value.
    // -------------------------------------------------------------------
    function all_rows_match;
        input [7:0] col;
        input       expected;
        integer rr;
        begin
            all_rows_match = 1;
            for (rr = 0; rr < 128; rr = rr + 1)
                if (dut.bank.u_cells.storage[rr][col] !== expected)
                    all_rows_match = 0;
        end
    endfunction

    // -------------------------------------------------------------------
    // Bookkeeping
    // -------------------------------------------------------------------
    integer pass_count, fail_count;

    task check;
        input [200*8-1:0] label;
        input             got;
        input             expected;
        begin
            if (got === expected) begin
                $display("  PASS  %0s", label);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL  %0s  got=%b expected=%b", label, got, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------
    // Issue one instruction and wait for op_done
    // -------------------------------------------------------------------
    task issue_and_wait;
        input [31:0] instruction;
        begin
            @(posedge clk); #1;
            inst = instruction;
            @(posedge op_done);
            @(posedge clk); #1;
            inst = 32'h0;
        end
    endtask

    // -------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------
    initial begin
        $dumpfile("tb_integrated.vcd");
        $dumpvars(0, tb_integrated);

        pass_count = 0;
        fail_count = 0;

        rst  = 1;
        inst = 0;
        repeat(4) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // ================================================================
        // TEST 1: 3-bit addition  A=5 (101), B=6 (110) → 011 + carry=1
        //
        // Memory layout (column addresses):
        //   col 0 = A[0]=1, col 1 = A[1]=0, col 2 = A[2]=1
        //   col 8 = B[0]=0, col 9 = B[1]=1, col 10= B[2]=1
        //   col 16= D[0],   col 17= D[1],   col 18= D[2]
        //
        // Instruction word: [27:24]=opcode  [23:16]=RA  [15:8]=RB  [7:0]=RD
        // ADD opcode = 4'b0001 = 4'h1
        // ================================================================
        $display("\n=== TEST 1: 3-bit addition  A=5+B=6 ===");

        // Load A bits (all 128 rows get the same value)
        write_col_all_rows(8'd0,  1'b1);  // A[0] = 1
        write_col_all_rows(8'd1,  1'b0);  // A[1] = 0
        write_col_all_rows(8'd2,  1'b1);  // A[2] = 1
        // Load B bits
        write_col_all_rows(8'd8,  1'b0);  // B[0] = 0
        write_col_all_rows(8'd9,  1'b1);  // B[1] = 1
        write_col_all_rows(8'd10, 1'b1);  // B[2] = 1
        // Clear destination
        write_col_all_rows(8'd16, 1'b0);
        write_col_all_rows(8'd17, 1'b0);
        write_col_all_rows(8'd18, 1'b0);

        // Issue ADD instruction:
        //   enable=4'h0, opcode=4'h1, RA=8'd0, RB=8'd8, RD=8'd16
        issue_and_wait({4'h0, 4'h1, 8'd0, 8'd8, 8'd16});

        // Expected:  5+6=11 = 4'b1011
        //   D[0]=1, D[1]=1, D[2]=0, carry=1
        check("ADD D[0]=1 (all rows)", all_rows_match(8'd16, 1'b1), 1'b1);
        check("ADD D[1]=1 (all rows)", all_rows_match(8'd17, 1'b1), 1'b1);
        check("ADD D[2]=0 (all rows)", all_rows_match(8'd18, 1'b0), 1'b1);
        check("ADD carry=1 row0",  dut.bank.carry[0],  1'b1);
        check("ADD carry=1 row127",dut.bank.carry[127],1'b1);

        // ================================================================
        // TEST 2: 2-bit multiplication  A=3 (11), B=2 (10) → 6 (0110)
        //
        // Memory layout:
        //   col 30 = A[0]=1, col 31 = A[1]=1
        //   col 40 = B[0]=0, col 41 = B[1]=1
        //   cols 50..53 = product accumulator (init to 0)
        //
        // MUL opcode = 4'b0010 = 4'h2
        // ================================================================
        $display("\n=== TEST 2: 2-bit multiply A=3 x B=2 ===");

        write_col_all_rows(8'd30, 1'b1);  // A[0] = 1
        write_col_all_rows(8'd31, 1'b1);  // A[1] = 1
        write_col_all_rows(8'd40, 1'b0);  // B[0] = 0  (2 = 10b)
        write_col_all_rows(8'd41, 1'b1);  // B[1] = 1

        // Reset
        @(posedge clk); rst = 1; @(posedge clk); rst = 0; @(posedge clk);

        // Issue MUL instruction:
        //   enable=4'h0, opcode=4'h2, RA=8'd30, RB=8'd40, RD=8'd50
        issue_and_wait({4'h0, 4'h2, 8'd30, 8'd40, 8'd50});

        // Expected: 3×2=6 = 4'b0110
        //   prod[0]=0, prod[1]=1, prod[2]=1, prod[3]=0
        check("MUL prod[0]=0", all_rows_match(8'd50, 1'b0), 1'b1);
        check("MUL prod[1]=1", all_rows_match(8'd51, 1'b1), 1'b1);
        check("MUL prod[2]=1", all_rows_match(8'd52, 1'b1), 1'b1);
        check("MUL prod[3]=0", all_rows_match(8'd53, 1'b0), 1'b1);

        // ================================================================
        // Summary
        // ================================================================
        $display("\n=== RESULTS: %0d passed, %0d failed ===",
                 pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else                 $display("SOME TESTS FAILED");

        $finish;
    end

    initial begin
        #500000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
