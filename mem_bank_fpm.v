



`include "bit_cell_8t_array.v"
`include "sense_amp_row.v"
`include "cram_full_adder.v"

module memory_bank #(parameter ds = 3)(
    input            clk,
    input            rst,
    input  [3:0]     op_code,
    input  [3:0]     scratch_zone,   // FP scratch base = col_addr_D + (scratch_zone<<4)
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
    // FP MULTIPLY state registers — NEW
    // =========================================================================
    reg [3:0] fpm_state;       // top-level FP multiply phase (see header)
    reg [4:0] mant_bit_idx;    // outer loop: which B_mant bit (0..23)
    reg [4:0] fp_inner_bit;    // inner loop: which A_mant/accum bit (0..23)
    reg [5:0] norm_offset;     // 23 or 24 — start col offset into accumulator
                               // for the mantissa copy (set by normalise check)
    reg [3:0] fp_bit_ctr;      // general 4-bit counter reused across FP phases
                               // (exp add loop 0..7, bias sub loop 0..7,
                               //  accum-zero loop 0..47, mant-copy loop 0..22)
    reg       norm_exp_adj;    // 1 = bit47 was set; need to add 1 to R_exp

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
    //   During FP multiply:      fpm_cwl_A / fpm_cwl_B / fpm_cwl_D
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

    // -------------------------------------------------------------------------
    // FP multiply column address computations
    //
    // Scratch 3 (accumulator) base = col_addr_D + 32
    // Inverted-bias constant pool  = cols 248..255  (permanent)
    //
    // fpm_cwl_A : source column A for current FP sub-operation
    // fpm_cwl_B : source column B (or same as A for single-col reads)
    // fpm_cwl_D : destination column for current FP sub-operation
    // -------------------------------------------------------------------------
    reg [7:0] fpm_cwl_A;
    reg [7:0] fpm_cwl_B;
    reg [7:0] fpm_cwl_D;

    // Accumulator base address (scratch 3 — caller controlled via scratch_zone)
    // accum_base = col_addr_D + (scratch_zone << 4)
    // scratch_zone=4'h2 → D+32  (minimum safe: D+48 needed for 32-col result field)
    // scratch_zone=4'h3 → D+48, scratch_zone=4'h4 → D+64, etc.
    wire [7:0] accum_base = col_addr_D + {scratch_zone, 4'b0000};

    // Mux: FP override takes priority, then tag-load, then normal compute
    wire fpm_active = (op_code == 4'b0011);

    wire [7:0] eff_cwl_A = fpm_active      ? fpm_cwl_A :
                           tag_load_active  ? tag_load_col : A_curr_addr;
    wire [7:0] eff_cwl_B = fpm_active      ? fpm_cwl_B :
                           tag_load_active  ? tag_load_col : B_curr_addr;

    // Destination column mux: zero-init overrides to the column being zeroed
    wire [7:0] eff_cwl_D = fpm_active        ? fpm_cwl_D :
                           zero_init_active   ? (col_addr_D + {6'd0, zero_col_idx})
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
    //   FP XOR (sign):       cwl_wdata[r] = sa_xor[r]  (no tag gate — all rows)
    //   FP unconditional:    cwl_wdata[r] = sum[r]  (no tag gate — all rows)
    //   FP carry-store:      cwl_wdata[r] = carry[r]  (no tag gate — all rows)
    // =========================================================================
    reg fpm_xor_wb;        // 1 = write sa_xor directly (sign XOR result)
    reg fpm_uncond_wb;     // 1 = write sum without tag gate (constant writes)
    reg fpm_carry_wb;      // 1 = write carry without tag gate (exp +1 adjust)

    wire [127:0] cwl_wdata_bus;
    genvar wg;
    generate
        for (wg = 0; wg < 128; wg = wg + 1) begin : gen_wb
            assign cwl_wdata_bus[wg] =
                zero_init_active   ? 1'b0 :
                fpm_xor_wb         ? sa_xor[wg] :
                fpm_uncond_wb      ? sum[wg] :
                fpm_carry_wb       ? carry[wg] :
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
            // FP registers
            fpm_state          <= 0;
            mant_bit_idx       <= 0;
            fp_inner_bit       <= 0;
            norm_offset        <= 0;
            fp_bit_ctr         <= 0;
            norm_exp_adj       <= 0;
            fpm_cwl_A          <= 0;
            fpm_cwl_B          <= 0;
            fpm_cwl_D          <= 0;
            fpm_xor_wb         <= 0;
            fpm_uncond_wb      <= 0;
            fpm_carry_wb       <= 0;
        end
        else begin

            // Default: de-assert all one-cycle strobes
            op_done            <= 0;
            pch                <= 0;
            cwl_en             <= 0;
            sa_en              <= 0;
            cwl_we             <= 0;
            carry_store_active <= 0;
            fpm_xor_wb         <= 0;
            fpm_uncond_wb      <= 0;
            fpm_carry_wb       <= 0;
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

                // State 01 — add LSB (bit_pos=0)
                2'b01: begin
                    case (sub_phase)
                    2'b00: begin                    // pre-charge
                        pch       <= 1;
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin                    // CWL activate + SA strobe
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin                    // write-back + carry + advance
                        cwl_we <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        bit_pos   <= 3'd1;
                        add_state <= 2'b10;
                        sub_phase <= 2'b00;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                // State 10 — add mid bit (bit_pos=1)
                2'b10: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        cwl_we <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        bit_pos   <= 3'd2;
                        add_state <= 2'b11;
                        sub_phase <= 2'b00;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                // State 11 — add MSB (bit_pos=2), assert op_done
                2'b11: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        cwl_we <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        add_state <= 2'b00;
                        sub_phase <= 2'b00;
                        op_done   <= 1;
                    end
                    default: sub_phase <= 2'b00;
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

            // =================================================================
            // FP32 MULTIPLICATION   opcode 4'b0011
            //
            // Column address contract (all offsets from col_addr_A/B/D):
            //   A: [0]=sign  [1..8]=exp  [9..32]=mant(incl implicit-1 at [9])
            //   B: same layout
            //   D: [0]=R_sign  [1..8]=R_exp  [9..31]=R_mant  [32..79]=accum
            //   cols 248..255 = inverted-bias constant pool (permanent)
            //
            // sub_phase 0/1/2 = pch / cwl+sa / writeback  — same as all ops
            // =================================================================
            4'b0011: begin
                case (fpm_state)

                // -------------------------------------------------------------
                // State 4'h0  FPM_INIT
                // Phase A (sub_phase 00..01 looping): zero accumulator cols
                //   accum_base+0 through accum_base+47  (48 cols)
                //   uses zero_init_active → cwl_wdata_bus=0 unconditionally
                //   chained write loop: each sub_phase 01 advances one col
                // Phase B (sub_phase 10..11): write inverted bias to cols 248-255
                //   ~8'h7F = 8'b10000000
                //   Bits 0..6 (cols 248..254) = 0 → already zeroed in phase A
                //   since we zero accum then those cols stay 0 independently.
                //   Only col 255 (bit 7) needs a 1 written.
                //
                //   HOW col 255 gets written to 1 (no carry involvement):
                //   sub_phase 10: assert pch so CBL is pre-charged to 1.
                //   sub_phase 11: assert cwl_en+sa_en — BUT fpm_cwl_A and
                //     fpm_cwl_B both point to col 255 which holds 0. So after
                //     CWL discharge CBL would be 0, not 1.
                //   Better approach: skip the CWL read entirely. We know we
                //   want to write 1 unconditionally to col 255. Use the
                //   zero_init_active=0 + fpm_carry_wb path — BUT with carry
                //   properly set to 1 for all rows BEFORE the write cycle,
                //   not within the same cycle (fixing the NB assignment race
                //   from the previous version).
                //
                //   Correct sequence:
                //   sub_phase 10: set carry[all]=1, set fpm_cwl_D=255,
                //                 advance to sub_phase 11
                //   sub_phase 11: assert cwl_we with fpm_carry_wb=1
                //                 → cwl_wdata_bus[r] = carry[r] = 1 for all r
                //                 carry[] is then reset to 0 before leaving
                //
                //   This is correct because carry[all] is set in sub_phase 10
                //   (clock edge N), so it is stable 1 when cwl_we fires in
                //   sub_phase 11 (clock edge N+1). No same-cycle race.
                // -------------------------------------------------------------
                4'h0: begin
                    case (sub_phase)

                    2'b00: begin
                        // Initialise working registers
                        for (i = 0; i < 128; i = i + 1) begin
                            carry[i] <= 0;
                            tag[i]   <= 1;
                        end
                        mant_bit_idx     <= 0;
                        fp_inner_bit     <= 0;
                        norm_exp_adj     <= 0;
                        fp_bit_ctr       <= 0;
                        // Start zeroing accumulator: col accum_base + 0
                        zero_init_active <= 1;
                        fpm_cwl_D        <= accum_base;
                        cwl_we           <= 1;
                        sub_phase        <= 2'b01;
                    end

                    2'b01: begin
                        // Chain through accumulator cols 1..47
                        // (col 0 written in sub_phase 00 above)
                        if (fp_bit_ctr < 4'd14) begin
                            fp_bit_ctr <= fp_bit_ctr + 4'd1;
                            fpm_cwl_D  <= accum_base + {3'b0, fp_bit_ctr} + 8'd1;
                            cwl_we     <= 1;
                            // Stay in sub_phase 01 — chained zero-writes
                            // (no PCH needed between zero-write cycles)
                        end
                        else begin
                            // Accumulator zeroed (cols 0..47 done via 15 iterations
                            // of fp_bit_ctr 0..14, each writing 3 cols:
                            // actually fp_bit_ctr counts 0..14 = 15 steps, but
                            // we wrote col+0 in sub_phase 00 then cols +1..+47
                            // which is 47 more → 48 total  ✓)
                            // Now prepare col-255 write: set carry=1 for all rows
                            zero_init_active <= 0;
                            for (i = 0; i < 128; i = i + 1)
                                carry[i] <= 1;    // will be stable next cycle
                            fpm_cwl_D  <= 8'd255;
                            sub_phase  <= 2'b10;
                        end
                    end

                    2'b10: begin
                        // carry[all] is now 1 (set last cycle, stable)
                        // Write carry[r]=1 to col 255 for all rows
                        fpm_carry_wb <= 1;
                        cwl_we       <= 1;
                        sub_phase    <= 2'b11;
                    end

                    2'b11: begin
                        // Col 255 now holds 1 for all rows (inv-bias bit 7 done)
                        // Reset carry to 0 before leaving INIT
                        fpm_carry_wb <= 0;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= 0;
                        fp_bit_ctr <= 0;
                        fpm_state  <= 4'h1;
                        sub_phase  <= 2'b00;
                    end

                    endcase
                end

                // -------------------------------------------------------------
                // State 4'h1  FPM_SIGN
                // R_sign = A_sign XOR B_sign
                // Single CWL read activating col_addr_A (A_sign) and
                // col_addr_B (B_sign) simultaneously → SA gives XOR on sa_xor.
                // Write sa_xor to col_addr_D (R_sign) without tag gate.
                // -------------------------------------------------------------
                4'h1: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        fpm_cwl_A <= col_addr_A;       // A sign bit
                        fpm_cwl_B <= col_addr_B;       // B sign bit
                        fpm_cwl_D <= col_addr_D;       // R sign destination
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        // sa_xor[r] = A_sign[r] XOR B_sign[r]
                        fpm_xor_wb <= 1;    // write sa_xor to R_sign col
                        cwl_we     <= 1;
                        fpm_state  <= 4'h2;
                        sub_phase  <= 2'b00;
                        fp_bit_ctr <= 0;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= 0;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                // -------------------------------------------------------------
                // State 4'h2  FPM_EXP_ADD
                // 8-bit bit-serial addition: R_exp = A_exp + B_exp  (in-place)
                // fp_bit_ctr counts bits 0..7 (LSB first).
                // RA = col_addr_A+1+fp_bit_ctr  (A exponent bits)
                // RB = col_addr_B+1+fp_bit_ctr  (B exponent bits)
                // RD = col_addr_D+1+fp_bit_ctr  (R exponent — written directly,
                //      scratch 1 eliminated; holds raw sum during this phase)
                // All 128 rows execute in SIMD.  No tag gate (fpm_uncond_wb).
                // -------------------------------------------------------------
                4'h2: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        fpm_cwl_A <= col_addr_A + 8'd1 + {4'b0, fp_bit_ctr};
                        fpm_cwl_B <= col_addr_B + 8'd1 + {4'b0, fp_bit_ctr};
                        fpm_cwl_D <= col_addr_D + 8'd1 + {4'b0, fp_bit_ctr};
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        fpm_uncond_wb <= 1;    // write sum to R_exp bit, no tag
                        cwl_we        <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        if (fp_bit_ctr == 4'd7) begin
                            // Finished 8-bit exp add; move to bias subtract
                            fp_bit_ctr <= 0;
                            for (i = 0; i < 128; i = i + 1)
                                carry[i] <= 1;  // carry=1 for two's complement sub
                            fpm_state  <= 4'h3;
                        end
                        else begin
                            fp_bit_ctr <= fp_bit_ctr + 4'd1;
                        end
                        sub_phase <= 2'b00;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                // -------------------------------------------------------------
                // State 4'h3  FPM_EXP_BIAS
                // In-place bias subtraction: R_exp = R_exp + inv_bias + 1
                // Subtraction via two's complement: add ~127 with carry=1.
                // ~8'h7F = 8'b10000000
                //   bit 0..6 of inv-bias = 0  (cols 248..254, already zeroed)
                //   bit 7    of inv-bias = 1  (col 255, written in INIT)
                //
                // RA = col_addr_D+1+fp_bit_ctr  (current R_exp bit — source)
                // RB = 248+fp_bit_ctr            (inv-bias bit from constant pool)
                // RD = col_addr_D+1+fp_bit_ctr  (same col — in-place update)
                //
                // Since RA==RD (in-place), the read and write address are the
                // same column.  In bit_cell_8t_array this is handled correctly:
                // the CWL read happens in sub_phase 1, the CWL write-back
                // happens in sub_phase 2 (next clock edge), so there is no
                // read-write collision within the same cycle.
                // -------------------------------------------------------------
                4'h3: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        fpm_cwl_A <= col_addr_D + 8'd1 + {4'b0, fp_bit_ctr};
                        fpm_cwl_B <= 8'd248       + {4'b0, fp_bit_ctr};
                        fpm_cwl_D <= col_addr_D + 8'd1 + {4'b0, fp_bit_ctr};
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        fpm_uncond_wb <= 1;
                        cwl_we        <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        if (fp_bit_ctr == 4'd7) begin
                            fp_bit_ctr <= 0;
                            for (i = 0; i < 128; i = i + 1)
                                carry[i] <= 0;
                            fpm_state <= 4'h4;
                        end
                        else begin
                            fp_bit_ctr <= fp_bit_ctr + 4'd1;
                        end
                        sub_phase <= 2'b00;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                // -------------------------------------------------------------
                // State 4'h4  FPM_MANT_TAG
                // Load tag[] from B_mant bit [mant_bit_idx].
                // Single-column CWL read: both eff_cwl_A and eff_cwl_B point
                // to the same column → sa_out[r] = cell[r][col] (AND self).
                // tag[r] ← sa_out[r]
                // Also reset carry[] for the upcoming partial-product add.
                // -------------------------------------------------------------
                4'h4: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        // B_mant bit k is at col_addr_B+9+mant_bit_idx
                        // (col_addr_B+9 is the implicit-1 LSB of B mantissa)
                        fpm_cwl_A <= col_addr_B + 8'd9 + {3'b0, mant_bit_idx};
                        fpm_cwl_B <= col_addr_B + 8'd9 + {3'b0, mant_bit_idx};
                        fpm_cwl_D <= 8'd0;     // unused this cycle
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= 0;
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        for (i = 0; i < 128; i = i + 1)
                            tag[i] <= sa_out[i];   // sa_out = B_mant[k] AND B_mant[k] = B_mant[k]
                        fp_inner_bit <= 0;
                        fpm_state    <= 4'h5;
                        sub_phase    <= 2'b00;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                // -------------------------------------------------------------
                // State 4'h5  FPM_MANT_ADD
                // Inner loop: add A_mant bit [fp_inner_bit] into
                // accum[(mant_bit_idx + fp_inner_bit)] with carry propagation.
                //
                // RA = col_addr_A + 9 + fp_inner_bit   (A_mant source bit)
                // RB = accum_base  + mant_bit_idx + fp_inner_bit (accumulator)
                // RD = same as RB  (in-place accumulate)
                //
                // Tag gate is ACTIVE here (tag[r] = B_mant[k]):
                // rows where B_mant bit k = 0 have tag=0 → write-back suppressed
                // → those rows' accumulator bits are unchanged (correct behaviour
                //   for shift-and-add: partial product is zero when multiplier=0).
                //
                // Normal cwl_wdata_bus (tag-gated sum) is used — no override needed.
                // -------------------------------------------------------------
                4'h5: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        fpm_cwl_A <= col_addr_A + 8'd9 + {3'b0, fp_inner_bit};
                        fpm_cwl_B <= accum_base + {3'b0, mant_bit_idx} + {3'b0, fp_inner_bit};
                        fpm_cwl_D <= accum_base + {3'b0, mant_bit_idx} + {3'b0, fp_inner_bit};
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        cwl_we <= 1;    // tag-gated sum write (normal path)
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        if (fp_inner_bit == 5'd23) begin
                            // Done with all 24 A_mant bits for this B_mant bit
                            // Store final carry into accum[mant_bit_idx+24]
                            fpm_cwl_D  <= accum_base + {3'b0, mant_bit_idx} + 8'd24;
                            carry_store_active <= 1;
                            cwl_we     <= 1;
                            fpm_state  <= 4'h6;   // advance outer loop
                        end
                        else begin
                            fp_inner_bit <= fp_inner_bit + 5'd1;
                        end
                        sub_phase <= 2'b00;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                // -------------------------------------------------------------
                // State 4'h6  FPM_MANT_NEXT
                // Advance outer mantissa loop counter.
                // If mant_bit_idx < 23: reset carry, go back to tag-load (4'h4).
                // If mant_bit_idx == 23: all partial products done; go to
                // normalisation check (4'h7).
                // -------------------------------------------------------------
                4'h6: begin
                    carry_store_active <= 0;
                    if (mant_bit_idx == 5'd23) begin
                        mant_bit_idx <= 0;
                        fp_bit_ctr   <= 0;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= 0;
                        fpm_state <= 4'h7;
                    end
                    else begin
                        mant_bit_idx <= mant_bit_idx + 5'd1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= 0;
                        fpm_state <= 4'h4;
                    end
                    sub_phase <= 2'b00;
                end

                // -------------------------------------------------------------
                // State 4'h7  FPM_NORM_CHECK
                // Read accumulator bit 47 (accum_base + 47) to decide
                // normalisation shift.
                //
                // IEEE-754 mantissa product is 48 bits wide (24×24).
                // Bit 47 is the MSB:
                //   bit47 = 1 → product is of form 1x.xxx... (overflow of 1.xxx)
                //               take bits [46:24] as stored mantissa (23 bits)
                //               increment R_exp by 1
                //               norm_offset = 24  (start reading from accum+24)
                //   bit47 = 0 → product is of form 01.xxx...
                //               take bits [45:23] as stored mantissa (23 bits)
                //               R_exp unchanged
                //               norm_offset = 23  (start reading from accum+23)
                //
                // Single-column read into tag[]; norm_exp_adj and norm_offset
                // are derived from the uniform tag value (tag[0] is taken as
                // representative since all 128 rows execute the same FP op).
                // -------------------------------------------------------------
                4'h7: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        fpm_cwl_A <= accum_base + 8'd47;  // accum bit 47
                        fpm_cwl_B <= accum_base + 8'd47;  // same col (single read)
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        // sa_out[0] = accum_bit47 AND accum_bit47 = accum_bit47
                        // Use row 0 as representative (all rows are identical FP operands)
                        norm_exp_adj <= sa_out[0];
                        norm_offset  <= sa_out[0] ? 6'd24 : 6'd23;
                        // If bit47=1 we need to add 1 to R_exp
                        // Stage that: set carry=1 and go to exp-adjust before copy
                        if (sa_out[0]) begin
                            // Add 1 to R_exp: add 8'b1 to R_exp cols in-place
                            // Reuse EXP_ADD loop structure: set carry=1, B=0 cols
                            // achieves R_exp = R_exp + 0 + 1 = R_exp + 1
                            for (i = 0; i < 128; i = i + 1)
                                carry[i] <= 1;
                            fp_bit_ctr <= 0;
                            fpm_state  <= 4'hA;  // go to exp +1 adjust state
                        end
                        else begin
                            fp_bit_ctr <= 0;
                            fpm_state  <= 4'h8;  // skip exp adjust, go to copy
                        end
                        sub_phase <= 2'b00;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                // -------------------------------------------------------------
                // State 4'h8  FPM_MANT_COPY
                // Copy 23 bits from accum[norm_offset .. norm_offset+22]
                // into R_mant cols [col_addr_D+9 .. col_addr_D+31].
                // fp_bit_ctr counts 0..22.
                //
                // RA = accum_base + norm_offset + fp_bit_ctr  (source)
                // RB = same (single-col read → sa_out = source bit)
                // RD = col_addr_D + 9 + fp_bit_ctr            (dest)
                //
                // Write sa_out unconditionally (fpm_uncond_wb with sum path):
                // We need to write the raw bit value, not sum.
                // Use fpm_xor_wb=0, fpm_uncond_wb=0, fpm_carry_wb=0.
                // Instead: set carry=0, then fpm_uncond_wb writes sum[r].
                // With carry=0: sum = (A AND B) XOR (A XOR B) XOR 0 = A XOR B
                //                                 where A=sa_out, B=sa_xor
                // For single-col read: sa_out = bit, sa_xor = 0
                //   sum = (bit AND bit) XOR (bit XOR bit) XOR 0 = bit XOR 0 = bit ✓
                // So fpm_uncond_wb with carry=0 correctly copies the bit.
                // -------------------------------------------------------------
                4'h8: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        fpm_cwl_A <= accum_base + {2'b0, norm_offset} + {2'b0, fp_bit_ctr[4:0]};
                        fpm_cwl_B <= accum_base + {2'b0, norm_offset} + {2'b0, fp_bit_ctr[4:0]};
                        fpm_cwl_D <= col_addr_D + 8'd9 + {3'b0, fp_bit_ctr};
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        fpm_uncond_wb <= 1;   // write sum (= source bit) to R_mant
                        cwl_we        <= 1;
                        // carry unchanged (stays 0, set in norm_check or exp_adj)
                        if (fp_bit_ctr == 4'd22) begin
                            fpm_state  <= 4'h9;
                            fp_bit_ctr <= 0;
                        end
                        else begin
                            fp_bit_ctr <= fp_bit_ctr + 4'd1;
                        end
                        sub_phase <= 2'b00;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                // -------------------------------------------------------------
                // State 4'h9  FPM_DONE
                // Assert op_done and reset FP state machine.
                // -------------------------------------------------------------
                4'h9: begin
                    op_done   <= 1;
                    fpm_state <= 0;
                    sub_phase <= 0;
                    for (i = 0; i < 128; i = i + 1) begin
                        carry[i] <= 0;
                        tag[i]   <= 1;  // restore tag default (all-enabled)
                    end
                end

                // -------------------------------------------------------------
                // State 4'hA  FPM_EXP_ADJ
                // Add 1 to R_exp in-place (normalisation overflow correction).
                // carry[] was set to 1 in FPM_NORM_CHECK when bit47=1.
                // Add 0 + carry=1 to each R_exp bit in turn → increments R_exp.
                // RB = inv-bias col — BUT we want to add 0, not inv-bias.
                // Use a zero-col: cols 248..254 happen to be 0 (inv-bias bits
                // 0..6 are all 0 from INIT).  Point RB to col 248 for all bits.
                // Bit 7 of inv-bias (col 255) is 1 — do NOT use col 255 here.
                // Use col 248 (which is 0) for RB throughout.
                // RA = RD = col_addr_D+1+fp_bit_ctr, RB = col 248 (=0)
                // Result: sum = R_exp_bit XOR 0 XOR carry = R_exp_bit XOR carry
                //   This propagates the carry through the exponent correctly.
                // -------------------------------------------------------------
                4'hA: begin
                    case (sub_phase)
                    2'b00: begin
                        pch       <= 1;
                        fpm_cwl_A <= col_addr_D + 8'd1 + {4'b0, fp_bit_ctr};
                        fpm_cwl_B <= 8'd248;     // constant zero column
                        fpm_cwl_D <= col_addr_D + 8'd1 + {4'b0, fp_bit_ctr};
                        sub_phase <= 2'b01;
                    end
                    2'b01: begin
                        cwl_en    <= 1;
                        sa_en     <= 1;
                        sub_phase <= 2'b10;
                    end
                    2'b10: begin
                        fpm_uncond_wb <= 1;
                        cwl_we        <= 1;
                        for (i = 0; i < 128; i = i + 1)
                            carry[i] <= cout[i];
                        if (fp_bit_ctr == 4'd7) begin
                            fp_bit_ctr <= 0;
                            for (i = 0; i < 128; i = i + 1)
                                carry[i] <= 0;
                            fpm_state  <= 4'h8;  // proceed to mantissa copy
                        end
                        else begin
                            fp_bit_ctr <= fp_bit_ctr + 4'd1;
                        end
                        sub_phase <= 2'b00;
                    end
                    default: sub_phase <= 2'b00;
                    endcase
                end

                default: fpm_state <= 4'h0;
                endcase
            end // FP32 MULTIPLICATION

            endcase // op_code
        end
    end // always

endmodule
