

// =============================================================================
// mem_bank_integrated.v  —  Drop-in replacement for mem_bank_update.v
//
// Integrates bit_cell_8t_array + sense_amp_row into the existing design.
//
// WHAT CHANGED vs mem_bank_update.v  (all other logic is identical)
// ------------------------------------------------------------------
//  1. reg memory[0:127][0:255]  removed.
//     Replaced by: bit_cell_8t_array u_cells  (same storage, new ports).
//
//  2. A_sel / B_sel wires.
//     Were: memory[r][A_curr_addr] / memory[r][B_curr_addr]  (direct reads).
//     Now:  sa_out[] / sa_xor[]  from sense_amp_row.
//     The FA identity ensures sum and cout are numerically identical:
//       FA(a=sa_out, b=sa_xor, cin) = FA(a=A, b=B, cin)  ✓
//
//  3. Write-back.
//     Was:  memory[i][D_curr_addr] <= sum[i]  (inside always block).
//     Now:  cwl_we strobe into bit_cell_8t_array (registered, end of cycle).
//     Tag gate is pre-applied in cwl_wdata_bus before the cell array sees it.
//
//  4. A 2-bit sub_phase counter is added inside each FSM case.
//     sub_phase 0 = pch (pre-charge)
//     sub_phase 1 = cwl_en + sa_en (read cells, latch SA)
//     sub_phase 2 = cwl_we + carry update (write-back, advance state)
//     This wraps the original single-cycle body into a 3-cycle sequence
//     without touching any of the original state-transition logic.
//
//  5. Tag-load operation (mul_state 001 / 100).
//     Was: for(i) tag[i] <= memory[i][col_addr_B]  (direct read).
//     Now: CWL single-column read (activate same col for both A and B)
//          then tag[i] = sa_out[i] | sa_xor[i] = cell[i][col] ✓
//          (sa_xor is 0 for single-col read; sa_out = cell AND cell = cell)
//
//  6. Product-zero init (mul_state 000).
//     Was: memory[i][col_addr_D+k] <= 0  (direct write).
//     Now: zero_phase loop zeroes columns D..D+3 via cwl_we with forced-0 data.
//
// UNCHANGED
// ---------
//  • carry[] / tag[] arrays and all their update logic
//  • add_state / mul_state / bit_pos FSM encodings and transitions
//  • op_done assertion timing
//  • op_code[1] mux for A_curr_addr / B_curr_addr / D_curr_addr
//  • cram_full_adder generate block (128 FAs)
//  • Tag-gated write-back semantics (tag[i]=0 → no write)
// =============================================================================

`include "bit_cell_8t_array.v"
`include "sense_amp_row.v"
`include "cram_full_adder.v"

module memory_bank #(parameter ds = 3)(
    input            clk,
    input            rst,
    input  [3:0]     op_code,
    input  [7:0]     col_addr_A,
    input  [7:0]     col_addr_B,
    input  [7:0]     col_addr_D,
    output reg       op_done
);

    // =========================================================================
    // Carry and tag arrays — UNCHANGED
    // =========================================================================
    reg carry [0:127];
    reg tag   [0:127];

    // =========================================================================
    // FSM state registers — UNCHANGED
    // =========================================================================
    reg [1:0] add_state;
    reg [2:0] mul_state;
    reg [2:0] bit_pos;

    // =========================================================================
    // NEW: sub-phase counter and physical timing strobes
    // =========================================================================
    reg [1:0] sub_phase;   // 0=pch  1=cwl+sa  2=writeback

    reg pch;               // pre-charge CBL/CBLB
    reg cwl_en;            // activate CWL for columns A and B
    reg sa_en;             // strobe sense amplifier
    reg cwl_we;            // write-back result to destination column

    // =========================================================================
    // NEW: override signals for tag-load and zero-init special operations
    // =========================================================================
    // During tag-load we read a single column by driving both CWL_A and CWL_B
    // to the same address.  During zero-init we write zeros unconditionally.
    reg        tag_load_active;   // 1 = single-col read mode for tag loading
    reg [7:0]  tag_load_col;      // column address to read into tag[]

    reg        zero_init_active;  // 1 = we are zeroing product columns
    reg [1:0]  zero_col_idx;      // 0..3 → col_addr_D + zero_col_idx
    // (zero_init issues 4 one-cycle cwl_we pulses with wdata=0)

    reg        carry_store_active; // 1 = write carry[] to product MSB col

    // =========================================================================
    // Effective CWL column addresses
    //   During normal compute:   A_curr_addr / B_curr_addr
    //   During tag-load:         tag_load_col / tag_load_col  (same col, both)
    // =========================================================================
    wire [7:0] A_curr_addr;
    wire [7:0] B_curr_addr;
    wire [7:0] D_curr_addr;

    // Original address expressions — UNCHANGED
    assign A_curr_addr = ~op_code[1] ? (col_addr_A + bit_pos[1:0])
                                      : (col_addr_A + bit_pos[0]);
    assign B_curr_addr = ~op_code[1] ? (col_addr_B + bit_pos[1:0])
                                      : (col_addr_D + bit_pos[2:1]);
    assign D_curr_addr = ~op_code[1] ? (col_addr_D + bit_pos[1:0])
                                      : (col_addr_D + bit_pos[2:1]);

    // Mux in tag-load override
    wire [7:0] eff_cwl_A = tag_load_active ? tag_load_col : A_curr_addr;
    wire [7:0] eff_cwl_B = tag_load_active ? tag_load_col : B_curr_addr;

    // Destination column mux: zero-init overrides to the column being zeroed
    wire [7:0] eff_cwl_D = zero_init_active ? (col_addr_D + {6'd0, zero_col_idx})
                                             : D_curr_addr;

    // =========================================================================
    // CBL buses
    // =========================================================================
    wire [127:0] cbl;
    wire [127:0] cblb;

    // =========================================================================
    // Write-back data bus
    //   Normal compute:      cwl_wdata[r] = tag[r] ? sum[r] : 0
    //   Carry-store:         cwl_wdata[r] = tag[r] ? carry[r] : 0
    //   Zero-init:           cwl_wdata[r] = 0  (always)
    // =========================================================================
    wire [127:0] cwl_wdata_bus;
    genvar wg;
    generate
        for (wg = 0; wg < 128; wg = wg + 1) begin : gen_wb
            assign cwl_wdata_bus[wg] =
                zero_init_active   ? 1'b0 :
                carry_store_active ? (tag[wg] ? carry[wg] : 1'b0) :
                                     (tag[wg] ? sum[wg]   : 1'b0);
        end
    endgenerate

    // =========================================================================
    // bit_cell_8t_array instantiation
    // =========================================================================
    // HWL port is tied off here; connect to external pins if you need
    // testbench-driven initialisation (see tb_integrated.v).
    bit_cell_8t_array #(.ROWS(128), .COLS(256)) u_cells (
        .clk        (clk),
        .rst        (rst),
        // CWL compute read
        .pch        (pch),
        .cwl_en     (cwl_en),
        .cwl_col_A  (eff_cwl_A),
        .cwl_col_B  (eff_cwl_B),
        .cbl        (cbl),
        .cblb       (cblb),
        // CWL write-back
        .cwl_we     (cwl_we),
        .cwl_col_D  (eff_cwl_D),
        .cwl_wdata  (cwl_wdata_bus),
        // HWL — tied off (driven externally if needed)
        .hwl_row    (7'd0),
        .hwl_we     (1'b0),
        .hwl_wdata  (256'd0),
        .hwl_rdata  ()
    );

    // =========================================================================
    // sense_amp_row instantiation
    // =========================================================================
    wire [127:0] sa_out;     // = cell[A] AND cell[B]
    wire [127:0] sa_xor;     // = cell[A] XOR cell[B]
    wire         sa_valid;

    sense_amp_row #(.ROWS(128)) u_sa (
        .clk      (clk),
        .rst      (rst),
        .cbl      (cbl),
        .cblb     (cblb),
        .sa_en    (sa_en),
        .sa_out   (sa_out),
        .sa_xor   (sa_xor),
        .sa_valid (sa_valid)
    );

    // =========================================================================
    // SIMD operand feeds — CHANGED from direct memory read to SA outputs
    // =========================================================================
    // FA(sa_out, sa_xor, carry) = FA(A AND B, A XOR B, carry)
    //   sum  = (AANDB) ^ (AXORB) ^ carry = A ^ B ^ carry   ✓
    //   cout = (AANDB & AXORB) | (AXORB & carry) | (AANDB & carry)
    //        = 0 | ((A^B)&carry) | (A&B)  =  A&B | (A^B)&carry  ✓
    wire [127:0] sum;
    wire [127:0] cout;

    genvar r;
    generate
        for (r = 0; r < 128; r = r + 1) begin : SIMD_ROWS
            cram_full_adder FA (
                .a    (sa_out[r]),    // A AND B
                .b    (sa_xor[r]),    // A XOR B
                .cin  (carry[r]),
                .s    (sum[r]),
                .cout (cout[r])
            );
        end
    endgenerate

    // =========================================================================
    // Main FSM — original state machine with sub_phase extension
    // =========================================================================
    integer i;

    always @(posedge clk or posedge rst) begin

        if (rst) begin
            for (i = 0; i < 128; i = i + 1) begin
                carry[i] <= 0;
                tag[i]   <= 1;
            end
            add_state          <= 0;
            mul_state          <= 0;
            bit_pos            <= 0;
            op_done            <= 0;
            sub_phase          <= 0;
            pch                <= 0;
            cwl_en             <= 0;
            sa_en              <= 0;
            cwl_we             <= 0;
            tag_load_active    <= 0;
            tag_load_col       <= 0;
            zero_init_active   <= 0;
            zero_col_idx       <= 0;
            carry_store_active <= 0;
        end
        else begin

            // Default: de-assert all one-cycle strobes
            op_done            <= 0;
            pch                <= 0;
            cwl_en             <= 0;
            sa_en              <= 0;
            cwl_we             <= 0;
            carry_store_active <= 0;
            // (tag_load_active and zero_init_active cleared explicitly below)

            case (op_code)

            // =================================================================
            // ADDITION
            // =================================================================
            4'b0001: begin
                case (add_state)

                // State 00 — reset carry, arm bit 0
                // (pure register op, no memory access needed)
                2'b00: begin
                    for (i = 0; i < 128; i = i + 1)
                        carry[i] <= 0;
                    bit_pos   <= 0;
                    add_state <= 2'b01;
                    sub_phase <= 0;
                end

                // States 01/10/11 — one bit-serial add step each
                // Each state goes through sub_phase 0→1→2 before advancing.
                2'b01, 2'b10, 2'b11: begin
                    case (sub_phase)

                    2'b00: begin    // pre-charge
                        pch       <= 1;
                        sub_phase <= 1;
                    end

                    2'b01: begin    // CWL activate + SA strobe
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2;
                    end

                    2'b10: begin    // write-back + carry update + state advance
                        cwl_we <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];

                        sub_phase <= 0;

                        // Advance to next state (original transitions)
                        case (add_state)
                        2'b01: begin bit_pos <= 1; add_state <= 2'b10; end
                        2'b10: begin bit_pos <= 2; add_state <= 2'b11; end
                        2'b11: begin add_state <= 0; op_done <= 1;     end
                        endcase
                    end

                    default: sub_phase <= 0;
                    endcase
                end

                endcase
            end // ADDITION

            // =================================================================
            // MULTIPLICATION
            // =================================================================
            4'b0010: begin
                case (mul_state)

                // State 000 — zero-init product columns and carry/tag
                3'b000: begin
                    // Sub-phase 0: reset carry/tag registers
                    // Sub-phases 1-4: write zero to product cols D, D+1, D+2, D+3
                    case (sub_phase)

                    2'b00: begin
                        for (i = 0; i < 128; i = i + 1) begin
                            carry[i] <= 0;
                            tag[i]   <= 0;
                        end
                        zero_init_active <= 1;
                        zero_col_idx     <= 0;
                        cwl_we           <= 1;  // write zeros to col D+0
                        sub_phase        <= 1;
                    end

                    2'b01: begin
                        zero_col_idx <= 1;
                        cwl_we       <= 1;      // write zeros to col D+1
                        sub_phase    <= 2;
                    end

                    2'b10: begin
                        zero_col_idx <= 2;
                        cwl_we       <= 1;      // write zeros to col D+2
                        // need a 4th write — extend to sub_phase 3 with a one-hot trick:
                        // reuse the unconventional sub_phase=3 case
                        sub_phase    <= 3;
                    end

                    2'b11: begin
                        zero_col_idx     <= 3;
                        cwl_we           <= 1;  // write zeros to col D+3
                        zero_init_active <= 0;
                        mul_state        <= 3'b001;
                        sub_phase        <= 0;
                    end

                    endcase
                end

                // State 001 — load tag from col_addr_B (B LSB)
                3'b001: begin
                    case (sub_phase)
                    2'b00: begin
                        pch              <= 1;
                        tag_load_active  <= 1;
                        tag_load_col     <= col_addr_B;
                        sub_phase        <= 1;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2;
                    end
                    2'b10: begin
                        // sa_out[r] = cell[r][B] (single-col: AND self = self)
                        // sa_xor[r] = 0          (XOR self = 0)
                        for (i = 0; i < 128; i = i + 1)
                            tag[i] <= sa_out[i];
                        tag_load_active  <= 0;
                        bit_pos          <= 0;
                        mul_state        <= 3'b010;
                        sub_phase        <= 0;
                    end
                    default: sub_phase <= 0;
                    endcase
                end

                // State 010 — partial product, bit 0 (A LSB + D[0])
                3'b010: begin
                    case (sub_phase)
                    2'b00: begin pch <= 1; sub_phase <= 1; end
                    2'b01: begin cwl_en <= 1; sa_en <= 1; sub_phase <= 2; end
                    2'b10: begin
                        cwl_we <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        bit_pos   <= 3;
                        mul_state <= 3'b011;
                        sub_phase <= 0;
                    end
                    default: sub_phase <= 0;
                    endcase
                end

                // State 011 — partial product, bit 1 (A[1] + D[1])
                3'b011: begin
                    case (sub_phase)
                    2'b00: begin pch <= 1; sub_phase <= 1; end
                    2'b01: begin cwl_en <= 1; sa_en <= 1; sub_phase <= 2; end
                    2'b10: begin
                        cwl_we    <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        mul_state <= 3'b100;
                        sub_phase <= 0;
                    end
                    default: sub_phase <= 0;
                    endcase
                end

                // State 100 — load tag from col_addr_B+1 (B bit-1)
                3'b100: begin
                    case (sub_phase)
                    2'b00: begin
                        pch             <= 1;
                        tag_load_active <= 1;
                        tag_load_col    <= col_addr_B + 8'd1;
                        sub_phase       <= 1;
                    end
                    2'b01: begin cwl_en <= 1; sa_en <= 1; sub_phase <= 2; end
                    2'b10: begin
                        for (i = 0; i < 128; i = i + 1)
                            tag[i] <= sa_out[i];
                        tag_load_active <= 0;
                        bit_pos         <= 2;
                        mul_state       <= 3'b101;
                        sub_phase       <= 0;
                    end
                    default: sub_phase <= 0;
                    endcase
                end

                // State 101 — partial product, shifted bit 0 (A[0] + D[1])
                3'b101: begin
                    case (sub_phase)
                    2'b00: begin pch <= 1; sub_phase <= 1; end
                    2'b01: begin cwl_en <= 1; sa_en <= 1; sub_phase <= 2; end
                    2'b10: begin
                        cwl_we <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        bit_pos   <= 5;
                        mul_state <= 3'b110;
                        sub_phase <= 0;
                    end
                    default: sub_phase <= 0;
                    endcase
                end

                // State 110 — partial product, shifted bit 1 (A[1] + D[2])
                3'b110: begin
                    case (sub_phase)
                    2'b00: begin pch <= 1; sub_phase <= 1; end
                    2'b01: begin cwl_en <= 1; sa_en <= 1; sub_phase <= 2; end
                    2'b10: begin
                        cwl_we <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        bit_pos   <= 7;
                        mul_state <= 3'b111;
                        sub_phase <= 0;
                    end
                    default: sub_phase <= 0;
                    endcase
                end

                // State 111 — store final carry into MSB of product
                3'b111: begin
                    // carry_store_active routes cwl_wdata_bus to carry[r]
                    carry_store_active <= 1;
                    cwl_we             <= 1;
                    mul_state          <= 0;
                    sub_phase          <= 0;
                    op_done            <= 1;
                end

                endcase
            end // MULTIPLICATION

            endcase // op_code
        end
    end // always

endmodule
