// SafeDot stage-1 synthesis wrapper: addend-datapath A/B tops.
//
// Wraps transdot_decomp_addend_datapath_piped_combined_product_dp (the
// FMA-facing variant, instantiated with the FMA's parameterization: fp32
// scalar / fp16 SIMD / fp8 E4M3 lanes) with registered inputs and outputs
// so synthesis sees the real reg-to-reg stages:
//   T   : payload group (mantissa_c*, shamt*, modes) -> internal shifter reg
//   T+1 : cycle-2 group (product, effective_subtraction, tentative_sign),
//         registered once more in the wrapper to honor the FMA's port
//         timing contract; module outputs are combinational here
//   T+2 : wrapper output registers (the FMA's mid-pipe equivalent)
//
// A/B: one top; the shadow is enabled per synthesis run via
// +define+SAFEDOT_CHECK_EN+SAFEDOT_FMA_CHECK (the module parameter defaults
// track the macro), so the base run simply omits SAFEDOT_FMA_CHECK.
module safedot_addend_core (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        dp_enable_i,
  input  logic        fp4_enable_i,
  input  logic        simd_enable_i,
  input  logic        is_fp8_i,

  input  logic [23:0] mantissa_c_i,
  input  logic [47:0] product_comb_i,
  input  logic [47:0] product_dp_i,
  input  logic [6:0]  addend_shamt_i,
  input  logic        effective_subtraction_i,
  input  logic        tentative_sign_i,

  input  logic [10:0] mantissa_c_simd_i,
  input  logic [5:0]  addend_shamt_simd_i,
  input  logic        effective_subtraction_simd_i,
  input  logic        tentative_sign_simd_i,

  input  logic [3:0]  mantissa_c_fp8_1_i,
  input  logic [4:0]  addend_shamt_fp8_1_i,
  input  logic        effective_subtraction_fp8_1_i,
  input  logic        tentative_sign_fp8_1_i,

  input  logic [3:0]  mantissa_c_fp8_2_i,
  input  logic [4:0]  addend_shamt_fp8_2_i,
  input  logic        effective_subtraction_fp8_2_i,
  input  logic        tentative_sign_fp8_2_i,

  output logic        sticky_before_add_o,
  output logic [75:0] sum_o,
  output logic        final_sign_o,
  output logic        sticky_before_add_simd_o,
  output logic [36:0] sum_simd_o,
  output logic        final_sign_simd_o,
  output logic        sticky_before_add_fp8_1_o,
  output logic [15:0] sum_fp8_1_o,
  output logic        final_sign_fp8_1_o,
  output logic        sticky_before_add_fp8_2_o,
  output logic [15:0] sum_fp8_2_o,
  output logic        final_sign_fp8_2_o,
  output logic [3:0]  safedot_alarm_o
);

  // ---- T-side input registers (payload group + modes)
  logic        dp_q, fp4_q, simd_q, fp8_q;
  logic [23:0] mc_q;
  logic [10:0] mcs_q;
  logic [3:0]  mc1_q, mc2_q;
  logic [6:0]  sh_q;
  logic [5:0]  shs_q;
  logic [4:0]  sh1_q, sh2_q;
  // cycle-2 group: registered TWICE (arrives at the module one cycle after
  // the payload group, mirroring the FMA)
  logic [47:0] pc_q, pc_qq, pd_q, pd_qq;
  logic        es_q, es_qq, ts_q, ts_qq;
  logic        ess_q, ess_qq, tss_q, tss_qq;
  logic        es1_q, es1_qq, ts1_q, ts1_qq;
  logic        es2_q, es2_qq, ts2_q, ts2_qq;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      {dp_q, fp4_q, simd_q, fp8_q} <= '0;
      mc_q  <= '0; mcs_q <= '0; mc1_q <= '0; mc2_q <= '0;
      sh_q  <= '0; shs_q <= '0; sh1_q <= '0; sh2_q <= '0;
      pc_q  <= '0; pc_qq <= '0; pd_q <= '0; pd_qq <= '0;
      {es_q, es_qq, ts_q, ts_qq}     <= '0;
      {ess_q, ess_qq, tss_q, tss_qq} <= '0;
      {es1_q, es1_qq, ts1_q, ts1_qq} <= '0;
      {es2_q, es2_qq, ts2_q, ts2_qq} <= '0;
    end else begin
      {dp_q, fp4_q, simd_q, fp8_q} <= {dp_enable_i, fp4_enable_i, simd_enable_i, is_fp8_i};
      mc_q  <= mantissa_c_i;  mcs_q <= mantissa_c_simd_i;
      mc1_q <= mantissa_c_fp8_1_i; mc2_q <= mantissa_c_fp8_2_i;
      sh_q  <= addend_shamt_i; shs_q <= addend_shamt_simd_i;
      sh1_q <= addend_shamt_fp8_1_i; sh2_q <= addend_shamt_fp8_2_i;
      pc_q  <= product_comb_i;  pc_qq <= pc_q;
      pd_q  <= product_dp_i;    pd_qq <= pd_q;
      es_q  <= effective_subtraction_i;      es_qq <= es_q;
      ts_q  <= tentative_sign_i;             ts_qq <= ts_q;
      ess_q <= effective_subtraction_simd_i; ess_qq <= ess_q;
      tss_q <= tentative_sign_simd_i;        tss_qq <= tss_q;
      es1_q <= effective_subtraction_fp8_1_i; es1_qq <= es1_q;
      ts1_q <= tentative_sign_fp8_1_i;        ts1_qq <= ts1_q;
      es2_q <= effective_subtraction_fp8_2_i; es2_qq <= es2_q;
      ts2_q <= tentative_sign_fp8_2_i;        ts2_qq <= ts2_q;
    end
  end

  logic        c_st, c_fs, c_sts, c_fss, c_st1, c_fs1, c_st2, c_fs2;
  logic [75:0] c_sum;
  logic [36:0] c_sums;
  logic [15:0] c_sum1, c_sum2;
  logic [3:0]  c_alarm;

  transdot_decomp_addend_datapath_piped_combined_product_dp #(
    .SUPER_MAN_BITS      ( 23 ),
    .SUPER_MAN_BITS_SIMD ( 10 ),
    .SUPER_MAN_BITS_FP8  ( 3 )
  ) i_addend (
    .clk_i                   ( clk_i ),
    .rst_ni                  ( rst_ni ),
    .pipe_en                 ( 1'b1 ),
    .dp_enable_i             ( dp_q ),
    .fp4_enable_i            ( fp4_q ),
    .mantissa_c_i            ( mc_q ),
    .product_comb_i          ( pc_qq ),
    .product_dp_i            ( pd_qq ),
    .addend_shamt_i          ( sh_q ),
    .effective_subtraction_i ( es_qq ),
    .tentative_sign_i        ( ts_qq ),
    .sticky_before_add_o     ( c_st ),
    .sum_o                   ( c_sum ),
    .final_sign_o            ( c_fs ),
    .simd_enable_i           ( simd_q ),
    .is_fp8                  ( fp8_q ),
    .mantissa_c_simd_i            ( mcs_q ),
    .addend_shamt_simd_i          ( shs_q ),
    .effective_subtraction_simd_i ( ess_qq ),
    .tentative_sign_simd_i        ( tss_qq ),
    .sticky_before_add_simd_o     ( c_sts ),
    .sum_simd_o                   ( c_sums ),
    .final_sign_simd_o            ( c_fss ),
    .mantissa_c_fp8_1_i            ( mc1_q ),
    .addend_shamt_fp8_1_i          ( sh1_q ),
    .effective_subtraction_fp8_1_i ( es1_qq ),
    .tentative_sign_fp8_1_i        ( ts1_qq ),
    .sticky_before_add_fp8_1_o     ( c_st1 ),
    .sum_fp8_1_o                   ( c_sum1 ),
    .final_sign_fp8_1_o            ( c_fs1 ),
    .mantissa_c_fp8_2_i            ( mc2_q ),
    .addend_shamt_fp8_2_i          ( sh2_q ),
    .effective_subtraction_fp8_2_i ( es2_qq ),
    .tentative_sign_fp8_2_i        ( ts2_qq ),
    .sticky_before_add_fp8_2_o     ( c_st2 ),
    .sum_fp8_2_o                   ( c_sum2 ),
    .final_sign_fp8_2_o            ( c_fs2 ),
    .safedot_alarm_o               ( c_alarm )
  );

  // Output registers (T+2), the FMA mid-pipe equivalent.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      {sticky_before_add_o, final_sign_o}             <= '0;
      {sticky_before_add_simd_o, final_sign_simd_o}   <= '0;
      {sticky_before_add_fp8_1_o, final_sign_fp8_1_o} <= '0;
      {sticky_before_add_fp8_2_o, final_sign_fp8_2_o} <= '0;
      sum_o <= '0; sum_simd_o <= '0; sum_fp8_1_o <= '0; sum_fp8_2_o <= '0;
      safedot_alarm_o <= '0;
    end else begin
      sticky_before_add_o       <= c_st;
      final_sign_o              <= c_fs;
      sum_o                     <= c_sum;
      sticky_before_add_simd_o  <= c_sts;
      final_sign_simd_o         <= c_fss;
      sum_simd_o                <= c_sums;
      sticky_before_add_fp8_1_o <= c_st1;
      final_sign_fp8_1_o        <= c_fs1;
      sum_fp8_1_o               <= c_sum1;
      sticky_before_add_fp8_2_o <= c_st2;
      final_sign_fp8_2_o        <= c_fs2;
      sum_fp8_2_o               <= c_sum2;
      safedot_alarm_o           <= c_alarm;
    end
  end

endmodule
