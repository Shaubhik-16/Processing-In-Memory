// =============================================================================
// bit_cell_8t_array.v
//
// Behavioural model of the 128x256 array of 8T transposable bit cells.
// This module is a direct drop-in replacement for the flat
//   reg memory [0:127][0:255];
// that was inside memory_bank.
//
// The 8T cell has two independent access ports:
//
//   PORT 1 — HWL (horizontal word-line): conventional row-parallel access.
//     Activate hwl_row → reads/writes one full 256-bit row at a time.
//     Used by testbenches and higher-level initialisation.
//
//   PORT 2 — CWL (vertical compute word-line): bit-serial compute access.
//     Activate a column address → the single bit at (all rows, that column)
//     is driven onto the 128-bit CBL bus.
//     Two simultaneous CWL activations (cwl_col_A and cwl_col_B) produce
//     a wired-AND on CBL:  CBL[r] = cell[r][A] AND cell[r][B]
//     This is the fundamental in-memory AND that feeds the sense amplifier.
//
// Write-back path:
//     After the sense amplifier and full-adder produce a result, the
//     memory_bank write-back logic asserts cwl_we with data on cwl_wdata
//     and column address on cwl_col_D.  Only rows where the tag latch is 1
//     are written (the tag gate lives in memory_bank, not here).
//
// Pre-charge:
//     pch=1 forces CBL and CBLB to all-ones.  This must be asserted for
//     at least one clock cycle before any CWL read.
// =============================================================================

module bit_cell_8t_array #(
    parameter ROWS = 128,
    parameter COLS = 256
)(
    input  wire                     clk,
    input  wire                     rst,

    // ---- CWL compute read port ------------------------------------------
    // Two column addresses are activated simultaneously; the array drives
    // CBL[r] = cell[r][cwl_col_A] AND cell[r][cwl_col_B]  for all rows r.
    input  wire                     pch,           // pre-charge: force CBL=1
    input  wire                     cwl_en,        // activate CWL read
    input  wire [7:0]               cwl_col_A,     // column address operand A
    input  wire [7:0]               cwl_col_B,     // column address operand B

    output wire [ROWS-1:0]          cbl,           // compute bit-line (wired-AND)
    output wire [ROWS-1:0]          cblb,          // complement bus

    // ---- CWL write-back port --------------------------------------------
    // Write cwl_wdata[r] into cell[r][cwl_col_D] for every r where
    // cwl_we[r]=1.  The tag gate (tag[r]) is applied outside this module.
    input  wire                     cwl_we,        // write-back enable strobe
    input  wire [7:0]               cwl_col_D,     // destination column
    input  wire [ROWS-1:0]          cwl_wdata,     // per-row write data

    // ---- HWL conventional port ------------------------------------------
    // Full 256-bit row read/write, used for initialisation and SRAM-mode
    // access by the testbench or system bus.
    input  wire [$clog2(ROWS)-1:0]  hwl_row,       // row address
    input  wire                     hwl_we,        // write enable
    input  wire [COLS-1:0]          hwl_wdata,     // write data (full row)
    output reg  [COLS-1:0]          hwl_rdata      // read data (full row)
);

    // -----------------------------------------------------------------------
    // Storage  — same dimensions as the original reg memory[0:127][0:255]
    // -----------------------------------------------------------------------
    reg storage [0:ROWS-1][0:COLS-1];

    integer r, c;

    // -----------------------------------------------------------------------
    // HWL write and read
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (hwl_we) begin
            for (c = 0; c < COLS; c = c + 1)
                storage[hwl_row][c] <= hwl_wdata[c];
        end
        // Combinational read below (hwl_rdata is registered for timing)
        for (c = 0; c < COLS; c = c + 1)
            hwl_rdata[c] <= storage[hwl_row][c];
    end

    // -----------------------------------------------------------------------
    // CWL write-back
    //   cwl_we is a single-cycle strobe.  cwl_wdata[r] carries the
    //   per-row result.  The tag gate is applied by the caller
    //   (memory_bank already does: if(tag[i]) memory[i][D] <= sum[i]).
    //   Here we simply honour cwl_we and cwl_wdata directly.
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (cwl_we) begin
            for (r = 0; r < ROWS; r = r + 1) begin
                if (cwl_wdata[r])           // per-row data (tag already applied)
                    storage[r][cwl_col_D] <= 1'b1;
                else
                    storage[r][cwl_col_D] <= 1'b0;
            end
        end
    end

    // -----------------------------------------------------------------------
    // CWL read — wired-AND model
    //
    // In the physical circuit, activating two CWLs simultaneously causes both
    // cells to pull down the shared CBL bus; the result is an open-drain AND.
    //
    //   cbl[r]  = cell[r][A] AND cell[r][B]   (true)
    //   cblb[r] = ~cell[r][A] AND ~cell[r][B] (complement)
    //
    // Pre-charge (pch=1) forces both buses to 1, resetting them between cycles.
    // -----------------------------------------------------------------------
    genvar gr;
    generate
        for (gr = 0; gr < ROWS; gr = gr + 1) begin : gen_cbl
            assign cbl[gr]  = pch ? 1'b1 :
                              (cwl_en ? (storage[gr][cwl_col_A] &  storage[gr][cwl_col_B]) : 1'b1);
            assign cblb[gr] = pch ? 1'b1 :
                              (cwl_en ? (~storage[gr][cwl_col_A] & ~storage[gr][cwl_col_B]) : 1'b1);
        end
    endgenerate

    // -----------------------------------------------------------------------
    // Reset — clear all storage
    // -----------------------------------------------------------------------
    integer ri, ci;
    always @(posedge clk) begin
        if (rst) begin
            for (ri = 0; ri < ROWS; ri = ri + 1)
                for (ci = 0; ci < COLS; ci = ci + 1)
                    storage[ri][ci] <= 1'b0;
        end
    end

endmodule
