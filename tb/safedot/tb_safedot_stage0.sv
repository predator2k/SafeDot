// SafeDot stage-0 checker testbench.
//
// Property 1 (no false alarms, docs/SafeDot.md sec. 6.4 / 9.4): with no fault
// injected, safedot_alarm_o must stay 0 for random and directed stimulus in
// every execution mode.
// Property 2 (sanity of detection): forcing single-bit corruptions on
// representative internal nets raises an alarm on the affected operation.
//
// Mode legality mirrors the FMA operand packer: in fp8/int4 DP mode every
// 6-bit mantissa segment carries a 4-bit payload (top two bits zero).
`timescale 1ns/1ps

`ifndef SAFEDOT_CMP_STAGES
`define SAFEDOT_CMP_STAGES 1
`endif

module tb_safedot_stage0;

  localparam int unsigned NUM_RANDOM_CYCLES = 300000;

  logic        clk, rst_n;
  logic        dp_enable, simd_enable, is_fp8, is_fp4;
  logic [5:0]  shamt_lane0, shamt_lane1;
  logic [4:0]  shamt_lane2, shamt_lane3;
  logic        sign_lane0, sign_lane1, sign_lane2, sign_lane3;
  logic [8:0]  fp4_y_mag0, fp4_y_mag1, fp4_y_mag2, fp4_y_mag3;
  logic [23:0] mantissa_a, mantissa_b;

  logic [47:0] product_non_dp, product_dp;
  logic [49:0] product_int_dp;
  logic        sign_out;
  logic [3:0]  alarm;

  int unsigned n_errors  = 0;
  int unsigned n_checked = 0;
  bit          expect_alarm = 1'b0;   // set during fault injection

  transdot_decomp_multiplier_w6_4lane_dp_piped #(
    .PRECISION_BITS ( 24 ),
    .SAFEDOT_CHECK  ( 1'b1 )
  ) dut (
    .clk_i            ( clk ),
    .rst_ni           ( rst_n ),
    .pipe_en          ( 1'b1 ),
    .dp_enable_i      ( dp_enable ),
    .simd_enable_i    ( simd_enable ),
    .is_fp8           ( is_fp8 ),
    .is_fp4           ( is_fp4 ),
    .shamt_lane0      ( shamt_lane0 ),
    .shamt_lane1      ( shamt_lane1 ),
    .shamt_lane2      ( shamt_lane2 ),
    .shamt_lane3      ( shamt_lane3 ),
    .sign_lane0       ( sign_lane0 ),
    .sign_lane1       ( sign_lane1 ),
    .sign_lane2       ( sign_lane2 ),
    .sign_lane3       ( sign_lane3 ),
    .fp4_y_mag0       ( fp4_y_mag0 ),
    .fp4_y_mag1       ( fp4_y_mag1 ),
    .fp4_y_mag2       ( fp4_y_mag2 ),
    .fp4_y_mag3       ( fp4_y_mag3 ),
    .mantissa_a       ( mantissa_a ),
    .mantissa_b       ( mantissa_b ),
    .product_non_dp_o ( product_non_dp ),
    .product_dp_o     ( product_dp ),
    .sign_out         ( sign_out ),
    .product_int_dp_o ( product_int_dp ),
    .safedot_alarm_o  ( alarm )
  );

  always #0.5 clk = ~clk;

  // Scalar-mode golden cross-check of the harness plumbing.
  logic [23:0] gold_a_q, gold_b_q;
  logic        gold_scalar_q;
  always @(posedge clk) begin
    gold_a_q      <= mantissa_a;
    gold_b_q      <= mantissa_b;
    gold_scalar_q <= rst_n && !dp_enable && !simd_enable && !is_fp8 && !is_fp4;
    if (gold_scalar_q && !expect_alarm) begin
      if (product_non_dp !== gold_a_q * gold_b_q) begin
        $display("[%0t] TB_ERROR: scalar product mismatch: %h * %h -> %h (exp %h)",
                 $time, gold_a_q, gold_b_q, product_non_dp, gold_a_q * gold_b_q);
        n_errors++;
      end
    end
  end

  // Give the pipeline a few cycles after reset before judging the alarm
  // (also covers configurations whose seed registers are reset-free).
  int unsigned warm_cnt = 0;
  always @(posedge clk) begin
    if (!rst_n) warm_cnt <= 0;
    else if (warm_cnt < 8) warm_cnt <= warm_cnt + 1;
  end

  // Alarm monitor: outside injection windows the alarm must be 0.
  always @(posedge clk) begin
    if (rst_n && warm_cnt >= 6 && !expect_alarm) begin
      if (alarm !== 4'b0) begin
        $display("[%0t] TB_ERROR: false alarm %b (dp=%0b simd=%0b fp8=%0b fp4=%0b sh0=%0d sh1=%0d sh2=%0d sh3=%0d sg=%b%b%b%b a=%h b=%h m0..3=%h %h %h %h)",
                 $time, alarm, dp_enable, simd_enable, is_fp8, is_fp4,
                 shamt_lane0, shamt_lane1, shamt_lane2, shamt_lane3,
                 sign_lane0, sign_lane1, sign_lane2, sign_lane3,
                 mantissa_a, mantissa_b,
                 fp4_y_mag0, fp4_y_mag1, fp4_y_mag2, fp4_y_mag3);
        n_errors++;
      end
      n_checked++;
    end
  end

  typedef enum int {
    M_SCALAR, M_SIMD_FP16, M_SIMD_FP8, M_DP_FP16, M_DP_FP8, M_DP_FP4, M_RAND
  } mode_e;

  task automatic drive_random(input mode_e m);
    logic [23:0] a, b;
    a = $urandom();
    b = $urandom();
    dp_enable   = 1'b0;
    simd_enable = 1'b0;
    is_fp8      = 1'b0;
    is_fp4      = 1'b0;
    case (m)
      M_SCALAR:    ;
      M_SIMD_FP16: simd_enable = 1'b1;
      M_SIMD_FP8:  begin simd_enable = 1'b1; is_fp8 = 1'b1; end
      M_DP_FP16:   dp_enable = 1'b1;
      M_DP_FP8:    begin dp_enable = 1'b1; is_fp8 = 1'b1; end
      M_DP_FP4:    begin dp_enable = 1'b1; is_fp4 = 1'b1; is_fp8 = $urandom_range(1); end
      M_RAND: begin
        dp_enable   = $urandom_range(1);
        simd_enable = $urandom_range(1);
        is_fp8      = $urandom_range(1);
        is_fp4      = $urandom_range(1);
      end
    endcase
    // FMA packer invariants: 4-bit lane payloads in fp8/int4 DP mode;
    // leading-zero 12-bit halves ({1'b0, 11-bit payload}) in fp16/int8 DP
    // mode, which bound pp3_res below 2^22 (the shadow's in-lane sign
    // derivation relies on this).
    if (dp_enable && is_fp8 && !is_fp4) begin
      a = {2'b00, a[21:18], 2'b00, a[15:12], 2'b00, a[9:6], 2'b00, a[3:0]};
      b = {2'b00, b[21:18], 2'b00, b[15:12], 2'b00, b[9:6], 2'b00, b[3:0]};
    end else if (dp_enable && !is_fp8 && !is_fp4) begin
      a = {1'b0, a[22:12], 1'b0, a[10:0]};
      b = {1'b0, b[22:12], 1'b0, b[10:0]};
    end
    mantissa_a = a;
    mantissa_b = b;
    // Shift amounts: bias toward the interesting low range, sprinkle extremes.
    case ($urandom_range(3))
      0: begin
        shamt_lane0 = $urandom_range(63);
        shamt_lane1 = $urandom_range(63);
      end
      default: begin
        shamt_lane0 = $urandom_range(25);
        shamt_lane1 = $urandom_range(25);
      end
    endcase
    shamt_lane2 = $urandom_range(31);
    shamt_lane3 = $urandom_range(31);
    sign_lane0  = $urandom_range(1);
    sign_lane1  = $urandom_range(1);
    sign_lane2  = $urandom_range(1);
    sign_lane3  = $urandom_range(1);
    // FP4 magnitudes with a healthy dose of zeros (negate-of-zero corner).
    fp4_y_mag0 = ($urandom_range(7) == 0) ? 9'd0 : $urandom_range(511);
    fp4_y_mag1 = ($urandom_range(7) == 0) ? 9'd0 : $urandom_range(511);
    fp4_y_mag2 = ($urandom_range(7) == 0) ? 9'd0 : $urandom_range(511);
    fp4_y_mag3 = ($urandom_range(7) == 0) ? 9'd0 : $urandom_range(511);
  endtask

  task automatic drive_directed(input int idx);
    // Baseline: benign scalar op.
    dp_enable = 1'b0; simd_enable = 1'b0; is_fp8 = 1'b0; is_fp4 = 1'b0;
    mantissa_a = 24'h800001; mantissa_b = 24'hFFFFFF;
    shamt_lane0 = '0; shamt_lane1 = '0; shamt_lane2 = '0; shamt_lane3 = '0;
    sign_lane0 = 1'b0; sign_lane1 = 1'b0; sign_lane2 = 1'b0; sign_lane3 = 1'b0;
    fp4_y_mag0 = '0; fp4_y_mag1 = '0; fp4_y_mag2 = '0; fp4_y_mag3 = '0;
    case (idx)
      0: begin mantissa_a = '0; mantissa_b = '0; end
      1: begin mantissa_a = '1; mantissa_b = '1; end
      2: begin // fp16 DP, both lanes shifted to zero, signs negate
        dp_enable = 1'b1;
        mantissa_a = 24'hFFFFFF; mantissa_b = 24'hFFFFFF;
        shamt_lane0 = 6'd63; shamt_lane1 = 6'd63;
        sign_lane0 = 1'b1; sign_lane1 = 1'b1;
      end
      3: begin // fp16 DP, exact cancellation: same product, opposite signs
        dp_enable = 1'b1;
        mantissa_a = 24'hABCABC; mantissa_b = 24'hDEFDEF; // hi half == lo half
        shamt_lane0 = 6'd0; shamt_lane1 = 6'd0;
        sign_lane0 = 1'b0; sign_lane1 = 1'b1;
      end
      4: begin // fp8 DP, all lanes zero payload, negating signs
        dp_enable = 1'b1; is_fp8 = 1'b1;
        mantissa_a = 24'h0; mantissa_b = 24'h0;
        sign_lane0 = 1'b1; sign_lane1 = 1'b1; sign_lane2 = 1'b1; sign_lane3 = 1'b1;
        shamt_lane0 = 6'd1; shamt_lane1 = 6'd3; shamt_lane2 = 5'd5; shamt_lane3 = 5'd7;
      end
      5: begin // fp8 DP, lanes shifted fully out then negated
        dp_enable = 1'b1; is_fp8 = 1'b1;
        mantissa_a = {2'b00, 4'hF, 2'b00, 4'hF, 2'b00, 4'hF, 2'b00, 4'hF};
        mantissa_b = {2'b00, 4'hF, 2'b00, 4'hF, 2'b00, 4'hF, 2'b00, 4'hF};
        shamt_lane0 = 6'd31; shamt_lane1 = 6'd31; shamt_lane2 = 5'd31; shamt_lane3 = 5'd31;
        sign_lane0 = 1'b1; sign_lane1 = 1'b0; sign_lane2 = 1'b1; sign_lane3 = 1'b0;
      end
      6: begin // fp4 DP: zero magnitudes with negate signs
        dp_enable = 1'b1; is_fp4 = 1'b1;
        fp4_y_mag0 = 9'd0; fp4_y_mag1 = 9'd0; fp4_y_mag2 = 9'd0; fp4_y_mag3 = 9'd0;
        sign_lane0 = 1'b1; sign_lane1 = 1'b1; sign_lane2 = 1'b1; sign_lane3 = 1'b1;
      end
      7: begin // fp4 DP: exact cancellation of two lanes
        dp_enable = 1'b1; is_fp4 = 1'b1;
        fp4_y_mag0 = 9'd170; fp4_y_mag1 = 9'd170; fp4_y_mag2 = 9'd0; fp4_y_mag3 = 9'd0;
        sign_lane0 = 1'b0; sign_lane1 = 1'b1; sign_lane2 = 1'b0; sign_lane3 = 1'b1;
      end
      8: begin // fp16 DP with odd/even shift parities
        dp_enable = 1'b1;
        mantissa_a = 24'h92_4924; mantissa_b = 24'h6D_B6DB;
        shamt_lane0 = 6'd1; shamt_lane1 = 6'd2;
        sign_lane0 = 1'b1; sign_lane1 = 1'b0;
      end
      9: begin // simd fp8
        simd_enable = 1'b1; is_fp8 = 1'b1;
        mantissa_a = 24'hFFFFFF; mantissa_b = 24'hFFFFFF;
      end
      default: ;
    endcase
  endtask

  // Fault injection bookkeeping.
  int unsigned n_inject = 0, n_detected = 0;

  task automatic inject_and_check(input string label);
    // The caller has already applied the force. Hold stable operands for one
    // capture edge, then observe the alarm register. The caller must release
    // the force after this task returns and then call settle().
    expect_alarm = 1'b1;
    @(negedge clk);          // forced value present before the capture edge
    @(posedge clk);          // corrupted values captured into output regs
    @(negedge clk);
    @(posedge clk);          // alarm register reflects the corrupted op...
    if (`SAFEDOT_CMP_STAGES == 2) @(posedge clk);  // ...one cycle later with
                                                   // the pipelined compare
    #0.1;
    n_inject++;
    if (alarm !== 4'b0) begin
      n_detected++;
      $display("[%0t] inject %-28s -> alarm=%b (detected)", $time, label, alarm);
    end else begin
      $display("[%0t] inject %-28s -> alarm=0000 (NOT detected)", $time, label);
    end
  endtask

  // Flush the pipeline after the force is released, then re-arm the monitor.
  task automatic settle();
    @(negedge clk);
    repeat (3) @(posedge clk);
    #0.1 expect_alarm = 1'b0;
    @(negedge clk);
  endtask

  logic [11:0] t_pp12;
  logic [23:0] t_pp3;
  logic [49:0] t_fs;
  logic [47:0] t_prod;
  logic        t_sign;
  logic [1:0]  t_r2;

  initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    drive_directed(0);
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    // ---------------- directed corners ----------------
    for (int d = 0; d < 10; d++) begin
      drive_directed(d);
      repeat (3) @(negedge clk);
    end

    // ---------------- random sweep ----------------
    for (int unsigned i = 0; i < NUM_RANDOM_CYCLES; i++) begin
      mode_e m;
      case (i % 8)
        0: m = M_SCALAR;
        1: m = M_SIMD_FP16;
        2: m = M_SIMD_FP8;
        3: m = M_DP_FP16;
        4: m = M_DP_FP8;
        5: m = M_DP_FP4;
        default: m = M_RAND;
      endcase
      drive_random(m);
      @(negedge clk);
    end

    $display("TB: fault-free phase done: %0d cycles checked, %0d errors",
             n_checked, n_errors);

    // ---------------- fault injection sanity ----------------
    // 1) partial product corruption, scalar mode
    drive_directed(1);
    repeat (2) @(negedge clk);
    t_pp12 = dut.pp[0][0] ^ 12'h004;
    force dut.pp[0][0] = t_pp12;
    inject_and_check("pp[0][0]^bit2 (scalar)");
    release dut.pp[0][0];
    settle();

    // 2) partial product corruption, fp8 DP lane path. Use a small shift so
    // the corrupted bit survives into the product (a bit that is entirely
    // shifted out cannot corrupt the result, and with the negate-of-zero
    // path active the checker correctly stays silent on it).
    drive_directed(5);
    shamt_lane2 = 5'd3;
    repeat (2) @(negedge clk);
    t_pp12 = dut.pp[2][2] ^ 12'h001;
    force dut.pp[2][2] = t_pp12;
    inject_and_check("pp[2][2]^bit0 (dp fp8)");
    release dut.pp[2][2];
    settle();

    // 3) pp3_res corruption, scalar mode
    drive_directed(8);
    // make it scalar to exercise pp3_res[1]
    dp_enable = 1'b0;
    repeat (2) @(negedge clk);
    t_pp3 = dut.pp3_res[1] ^ 24'h000020;
    force dut.pp3_res[1] = t_pp3;
    inject_and_check("pp3_res[1]^bit5 (scalar)");
    release dut.pp3_res[1];
    settle();

    // 4) compressor sum low bit
    drive_directed(3);
    repeat (2) @(negedge clk);
    t_fs = dut.final_sum ^ 50'h80;
    force dut.final_sum = t_fs;
    inject_and_check("final_sum^bit7 (dp fp16)");
    release dut.final_sum;
    settle();

    // 5) compressor sum sign bit (negate decision + int/nondp truncation)
    drive_directed(8);
    repeat (2) @(negedge clk);
    t_fs = dut.final_sum ^ (50'h1 << 49);
    force dut.final_sum = t_fs;
    inject_and_check("final_sum^bit49 (dp fp16)");
    release dut.final_sum;
    settle();

    // 6) output register corruption
    drive_directed(1);
    repeat (2) @(negedge clk);
    @(posedge clk); #0.1;
    t_prod = dut.product_non_dp_q ^ 48'h10;
    force dut.product_non_dp_q = t_prod;
    inject_and_check("product_non_dp_q^bit4");
    release dut.product_non_dp_q;
    settle();

    // 7) sign register corruption (dp mode, nonzero sum)
    drive_directed(8);
    repeat (2) @(negedge clk);
    @(posedge clk); #0.1;
    t_sign = ~dut.sign_out_q;
    force dut.sign_out_q = t_sign;
    inject_and_check("sign_out_q flip (dp fp16)");
    release dut.sign_out_q;
    settle();

    // 8) checker-internal state corruption (fail-safe direction).
    // Segment residues must be nonzero or the flip is masked by mul3-by-0
    // (all-ones segments are 63 == 0 mod 3).
    drive_directed(1);
    mantissa_a = 24'h000001; mantissa_b = 24'h000001;
    repeat (2) @(negedge clk);
    @(posedge clk); #0.1;
    t_r2 = dut.g_safedot.sd_ra_seg_q[0] ^ 2'b01;
    force dut.g_safedot.sd_ra_seg_q[0] = t_r2;
    inject_and_check("sd_ra_seg_q[0]^1 (checker)");
    release dut.g_safedot.sd_ra_seg_q[0];
    settle();

    // 9) dp output register corruption (dp mode, so the v4-consolidated fp
    // extraction tree is reading product_dp_q on this operation).
    drive_directed(8);
    repeat (2) @(negedge clk);
    @(posedge clk); #0.1;
    t_prod = dut.product_dp_q ^ 48'h10;
    force dut.product_dp_q = t_prod;
    inject_and_check("product_dp_q^bit4 (dp fp16)");
    release dut.product_dp_q;
    settle();

    $display("TB: injection phase: %0d/%0d detected", n_detected, n_inject);

    if (n_errors == 0 && n_detected == n_inject)
      $display("TB: PASS (%0d fault-free cycles, %0d/%0d injections detected)",
               n_checked, n_detected, n_inject);
    else
      $display("TB: FAIL (%0d errors, %0d/%0d injections detected)",
               n_errors, n_detected, n_inject);
    $finish;
  end

endmodule
