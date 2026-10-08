

// =============================================================================
// sense_amp_row.v
//
// One pseudo-differential sense amplifier per row — 128 in total.
// Sits between the bit_cell_8t_array CBL buses and the SIMD full-adder row
// that already exists in memory_bank.
//
// Paper reference: Section III-B
//   "The pseudo-differential sense amplifiers are used at the end of CBL and
//    CBLB, allowing for early sensing of results at a much smaller bitline
//    voltage swing."
//
// How it plugs into the existing memory_bank
// ------------------------------------------
// Before this module existed, memory_bank read operands directly:
//   A_sel[r] = memory[r][A_curr_addr]   (wire into reg array)
//   B_sel[r] = memory[r][B_curr_addr]
//
// After integration, those same A_sel / B_sel wires are driven by sa_out[]:
//   sa_out[r] = latched(CBL[r])   →  replaces A_sel/B_sel feeds into the FA
//
// IMPORTANT: the full-adder already computes  sum = A_sel XOR B_sel XOR cin.
// After SA integration A_sel and B_sel come from the SA instead of directly
// from memory, but the FA logic itself does NOT change.
//
// Operand extraction from the SA
// --------------------------------
// When two CWLs are active simultaneously (cwl_A and cwl_B):
//   cbl[r]  = cell[r][A] AND cell[r][B]
//   cblb[r] = ~cell[r][A] AND ~cell[r][B]
//
// The SA captures this and the near-memory NOR gate reconstructs:
//   A XOR B = NOR(cbl, cblb)
//
// However the existing full adder already does:
//   sum  = A XOR B XOR cin
//   cout = (A AND B) OR ((A XOR B) AND cin)
//
// So we need to expose BOTH A AND B  AND  A XOR B to the FA.
// We do this by exposing sa_out (= A AND B) and sa_xor (= A XOR B)
// and letting memory_bank substitute them into the existing FA feeds:
//   A_sel[r] → sa_xor[r]    (XOR is the 'a' input; b input becomes carry)
//   OR keep the FA as-is and pass  a=sa_out, b=sa_xor, cin=carry
//   which gives:  sum  = sa_out XOR sa_xor XOR carry
//                      = (A AND B) XOR (A XOR B) XOR carry
//                      = A XOR B XOR carry  ✓ (since AND XOR XOR = XOR for single bits)
// That identity holds, so the simplest path is:
//   A_sel[r] = sa_out[r]    (A AND B)
//   B_sel[r] = sa_xor[r]    (A XOR B)
// and the carry-in is unchanged.  The FA then computes the correct sum.
//
// Timing
// ------
//   Cycle N-1:  pch=1 (pre-charge CBL/CBLB)
//   Cycle N  :  cwl_en=1 (CWL_A and CWL_B fire; bitlines discharge)
//   Cycle N  :  sa_en=1  (SA latches at end of same cycle, or cycle N+1)
//   Cycle N+1:  sa_out / sa_xor are stable → FA computes → memory_bank writes back
// =============================================================================

module sense_amp_row #(
    parameter ROWS = 128
)(
    input  wire                clk,
    input  wire                rst,

    // ---- from bit_cell_8t_array -----------------------------------------
    input  wire [ROWS-1:0]     cbl,        // compute bit-line bus (A AND B)
    input  wire [ROWS-1:0]     cblb,       // complement bus       (~A AND ~B)

    // ---- timing control -------------------------------------------------
    input  wire                sa_en,      // strobe: latch CBL/CBLB this cycle

    // ---- outputs to memory_bank SIMD FA row -----------------------------
    output reg  [ROWS-1:0]     sa_out,     // latched (A AND B)
    output reg  [ROWS-1:0]     sa_xor,     // latched (A XOR B) = NOR(cbl, cblb)
    output reg                 sa_valid    // outputs stable (1 cycle after sa_en)
);

    // -----------------------------------------------------------------------
    // Per-row pseudo-differential latch
    // Pre-charge state: both buses are 1 → sa_out=1, sa_xor=0 (pch phase)
    // After bitline discharge: cbl and cblb settle to their logic levels.
    // -----------------------------------------------------------------------
    integer r;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sa_out   <= {ROWS{1'b0}};
            sa_xor   <= {ROWS{1'b0}};
            sa_valid <= 1'b0;
        end
        else if (sa_en) begin
            for (r = 0; r < ROWS; r = r + 1) begin
                // Latch the differential result for each row
                sa_out[r] <=  cbl[r];            // A AND B
                sa_xor[r] <= ~(cbl[r] | cblb[r]); // NOR → A XOR B
            end
            sa_valid <= 1'b1;
        end
        else begin
            // De-assert valid when SA is not strobing (between cycles / pch)
            sa_valid <= 1'b0;
        end
    end

endmodule
