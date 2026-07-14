// SafeDot stage-0 synthesis wrapper (docs/SafeDot.md sec. 9.1).
//
// Registers every input, then drives transdot_decomp_multiplier_w6_4lane_dp_piped
// with pipe_en tied high, so the reg-to-reg paths seen by synthesis are the
// multiplier's real combinational depth. The core's own pipe stage provides
// the output registers; the SafeDot alarm (when enabled) is a registered
// output of the core already.
//
// High-frequency targets (0.8 / 0.6 ns): SAFEDOT_EXTRA_STAGES appends that
// many extra register banks on every output as retiming seeds —
// compile_ultra -retime pulls them backward through the multiplier cone to
// split it into 1 + SAFEDOT_EXTRA_STAGES stages. Functional equivalence
// under retiming is the tool's sequential-equivalence guarantee (SVF/FM).
//
// Two elaboration tops for the A/B experiment:
//   safedot_stage0_base : SAFEDOT_CHECK = 0 (baseline)
//   safedot_stage0_mod3 : SAFEDOT_CHECK = 1 (mod-3 residue shadow)
`ifndef SAFEDOT_EXTRA_STAGES
`define SAFEDOT_EXTRA_STAGES 0
`endif
module safedot_stage0_core #(
  parameter bit ENABLE_CHECKER = 1'b0
)(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        dp_enable_i,
  input  logic        simd_enable_i,
  input  logic        is_fp8_i,
  input  logic        is_fp4_i,

  input  logic [5:0]  shamt_lane0_i,
  input  logic [5:0]  shamt_lane1_i,
  input  logic [4:0]  shamt_lane2_i,
  input  logic [4:0]  shamt_lane3_i,

  input  logic        sign_lane0_i,
  input  logic        sign_lane1_i,
  input  logic        sign_lane2_i,
  input  logic        sign_lane3_i,

  input  logic [8:0]  fp4_y_mag0_i,
  input  logic [8:0]  fp4_y_mag1_i,
  input  logic [8:0]  fp4_y_mag2_i,
  input  logic [8:0]  fp4_y_mag3_i,

  input  logic [23:0] mantissa_a_i,
  input  logic [23:0] mantissa_b_i,

  output logic [47:0] product_non_dp_o,
  output logic [47:0] product_dp_o,
  output logic [49:0] product_int_dp_o,
  output logic        sign_o,
  output logic [3:0]  safedot_alarm_o
);

  logic        dp_enable_q, simd_enable_q, is_fp8_q, is_fp4_q;
  logic [5:0]  shamt_lane0_q, shamt_lane1_q;
  logic [4:0]  shamt_lane2_q, shamt_lane3_q;
  logic        sign_lane0_q, sign_lane1_q, sign_lane2_q, sign_lane3_q;
  logic [8:0]  fp4_y_mag0_q, fp4_y_mag1_q, fp4_y_mag2_q, fp4_y_mag3_q;
  logic [23:0] mantissa_a_q, mantissa_b_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dp_enable_q   <= 1'b0;
      simd_enable_q <= 1'b0;
      is_fp8_q      <= 1'b0;
      is_fp4_q      <= 1'b0;
      shamt_lane0_q <= '0;
      shamt_lane1_q <= '0;
      shamt_lane2_q <= '0;
      shamt_lane3_q <= '0;
      sign_lane0_q  <= 1'b0;
      sign_lane1_q  <= 1'b0;
      sign_lane2_q  <= 1'b0;
      sign_lane3_q  <= 1'b0;
      fp4_y_mag0_q  <= '0;
      fp4_y_mag1_q  <= '0;
      fp4_y_mag2_q  <= '0;
      fp4_y_mag3_q  <= '0;
      mantissa_a_q  <= '0;
      mantissa_b_q  <= '0;
    end else begin
      dp_enable_q   <= dp_enable_i;
      simd_enable_q <= simd_enable_i;
      is_fp8_q      <= is_fp8_i;
      is_fp4_q      <= is_fp4_i;
      shamt_lane0_q <= shamt_lane0_i;
      shamt_lane1_q <= shamt_lane1_i;
      shamt_lane2_q <= shamt_lane2_i;
      shamt_lane3_q <= shamt_lane3_i;
      sign_lane0_q  <= sign_lane0_i;
      sign_lane1_q  <= sign_lane1_i;
      sign_lane2_q  <= sign_lane2_i;
      sign_lane3_q  <= sign_lane3_i;
      fp4_y_mag0_q  <= fp4_y_mag0_i;
      fp4_y_mag1_q  <= fp4_y_mag1_i;
      fp4_y_mag2_q  <= fp4_y_mag2_i;
      fp4_y_mag3_q  <= fp4_y_mag3_i;
      mantissa_a_q  <= mantissa_a_i;
      mantissa_b_q  <= mantissa_b_i;
    end
  end

  logic [47:0] core_product_non_dp;
  logic [47:0] core_product_dp;
  logic [49:0] core_product_int_dp;
  logic        core_sign;
  logic [3:0]  core_alarm;

  transdot_decomp_multiplier_w6_4lane_dp_piped #(
    .PRECISION_BITS ( 24 ),
    .SAFEDOT_CHECK  ( ENABLE_CHECKER )
  ) i_mult (
    .clk_i            ( clk_i ),
    .rst_ni           ( rst_ni ),
    .pipe_en          ( 1'b1 ),
    .dp_enable_i      ( dp_enable_q ),
    .simd_enable_i    ( simd_enable_q ),
    .is_fp8           ( is_fp8_q ),
    .is_fp4           ( is_fp4_q ),
    .shamt_lane0      ( shamt_lane0_q ),
    .shamt_lane1      ( shamt_lane1_q ),
    .shamt_lane2      ( shamt_lane2_q ),
    .shamt_lane3      ( shamt_lane3_q ),
    .sign_lane0       ( sign_lane0_q ),
    .sign_lane1       ( sign_lane1_q ),
    .sign_lane2       ( sign_lane2_q ),
    .sign_lane3       ( sign_lane3_q ),
    .fp4_y_mag0       ( fp4_y_mag0_q ),
    .fp4_y_mag1       ( fp4_y_mag1_q ),
    .fp4_y_mag2       ( fp4_y_mag2_q ),
    .fp4_y_mag3       ( fp4_y_mag3_q ),
    .mantissa_a       ( mantissa_a_q ),
    .mantissa_b       ( mantissa_b_q ),
    .product_non_dp_o ( core_product_non_dp ),
    .product_dp_o     ( core_product_dp ),
    .sign_out         ( core_sign ),
    .product_int_dp_o ( core_product_int_dp ),
    .safedot_alarm_o  ( core_alarm )
  );

  // Retiming-seed register banks for high-frequency targets. Deliberately
  // reset-free: datapath pipeline registers need no reset, and reset-free
  // registers give DC register retiming (set_optimize_registers, enabled by
  // the flow when SAFEDOT_STAGES > 0) full freedom to balance them backward
  // through the multiplier cone.
  localparam int unsigned ExtraStages = `SAFEDOT_EXTRA_STAGES;
  // SeedDepth avoids a zero-size array in the untaken generate branch, which
  // VCS ignores but Presto elaborates (ELAB-914).
  localparam int unsigned SeedDepth = (ExtraStages == 0) ? 1 : ExtraStages;
  generate if (ExtraStages > 0) begin : g_pipe_seed
    logic [47:0] pnd_q [SeedDepth];
    logic [47:0] pdp_q [SeedDepth];
    logic [49:0] pint_q [SeedDepth];
    logic        sgn_q [SeedDepth];
    logic [3:0]  alm_q [SeedDepth];
    always_ff @(posedge clk_i) begin
      pnd_q[0]  <= core_product_non_dp;
      pdp_q[0]  <= core_product_dp;
      pint_q[0] <= core_product_int_dp;
      sgn_q[0]  <= core_sign;
      alm_q[0]  <= core_alarm;
      for (int s = 1; s < ExtraStages; s++) begin
        pnd_q[s]  <= pnd_q[s-1];
        pdp_q[s]  <= pdp_q[s-1];
        pint_q[s] <= pint_q[s-1];
        sgn_q[s]  <= sgn_q[s-1];
        alm_q[s]  <= alm_q[s-1];
      end
    end
    assign product_non_dp_o = pnd_q[ExtraStages-1];
    assign product_dp_o     = pdp_q[ExtraStages-1];
    assign product_int_dp_o = pint_q[ExtraStages-1];
    assign sign_o           = sgn_q[ExtraStages-1];
    assign safedot_alarm_o  = alm_q[ExtraStages-1];
  end else begin : g_no_seed
    assign product_non_dp_o = core_product_non_dp;
    assign product_dp_o     = core_product_dp;
    assign product_int_dp_o = core_product_int_dp;
    assign sign_o           = core_sign;
    assign safedot_alarm_o  = core_alarm;
  end endgenerate

endmodule


module safedot_stage0_base (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        dp_enable_i,
  input  logic        simd_enable_i,
  input  logic        is_fp8_i,
  input  logic        is_fp4_i,
  input  logic [5:0]  shamt_lane0_i,
  input  logic [5:0]  shamt_lane1_i,
  input  logic [4:0]  shamt_lane2_i,
  input  logic [4:0]  shamt_lane3_i,
  input  logic        sign_lane0_i,
  input  logic        sign_lane1_i,
  input  logic        sign_lane2_i,
  input  logic        sign_lane3_i,
  input  logic [8:0]  fp4_y_mag0_i,
  input  logic [8:0]  fp4_y_mag1_i,
  input  logic [8:0]  fp4_y_mag2_i,
  input  logic [8:0]  fp4_y_mag3_i,
  input  logic [23:0] mantissa_a_i,
  input  logic [23:0] mantissa_b_i,
  output logic [47:0] product_non_dp_o,
  output logic [47:0] product_dp_o,
  output logic [49:0] product_int_dp_o,
  output logic        sign_o,
  output logic [3:0]  safedot_alarm_o
);
  safedot_stage0_core #(.ENABLE_CHECKER(1'b0)) i_core (.*);
endmodule


module safedot_stage0_mod3 (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        dp_enable_i,
  input  logic        simd_enable_i,
  input  logic        is_fp8_i,
  input  logic        is_fp4_i,
  input  logic [5:0]  shamt_lane0_i,
  input  logic [5:0]  shamt_lane1_i,
  input  logic [4:0]  shamt_lane2_i,
  input  logic [4:0]  shamt_lane3_i,
  input  logic        sign_lane0_i,
  input  logic        sign_lane1_i,
  input  logic        sign_lane2_i,
  input  logic        sign_lane3_i,
  input  logic [8:0]  fp4_y_mag0_i,
  input  logic [8:0]  fp4_y_mag1_i,
  input  logic [8:0]  fp4_y_mag2_i,
  input  logic [8:0]  fp4_y_mag3_i,
  input  logic [23:0] mantissa_a_i,
  input  logic [23:0] mantissa_b_i,
  output logic [47:0] product_non_dp_o,
  output logic [47:0] product_dp_o,
  output logic [49:0] product_int_dp_o,
  output logic        sign_o,
  output logic [3:0]  safedot_alarm_o
);
  safedot_stage0_core #(.ENABLE_CHECKER(1'b1)) i_core (.*);
endmodule
