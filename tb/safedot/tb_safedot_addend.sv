// SafeDot stage-1: addend-datapath residue shadow testbench.
//
// Property 1 (no false alarms): with no fault injected, safedot_alarm_o
// stays 0 for random stimulus in every mode (scalar / FP16-SIMD /
// FP8-SIMD), including the fp8 foreign-bit discard corners (small shamts).
// Property 2 (detection): representative injections on the shifter
// register (kept and sticky regions), both adders, the sticky flag, the
// final sign, and shadow-internal state raise the alarm.
//
// The DUT is driven per the FMA's port timing contract: mantissa/shamt of
// op N at cycle T, product/effective_subtraction/tentative_sign of op N at
// T+1 (the TB delays that group by one cycle), outputs combinational at
// T+1, alarm registered at T+2. Mode signals are stable within a batch and
// batches are separated by idle gaps (the main datapath itself requires
// mode stability across T/T+1).
//
// Architectural shift-amount bounds (FMA exponent datapath saturation):
// scalar <= 76, fp16 <= 37, fp8 <= 19. The TB respects them; beyond these
// the main datapath itself discards payload bits.
module tb_safedot_addend;

  logic clk = 0, rst_n;
  always #5 clk = ~clk;

  // payload-group inputs (cycle T)
  logic        simd, fp8;
  logic [23:0] mc;
  logic [10:0] mcs;
  logic [3:0]  mc1, mc2;
  logic [6:0]  sh;
  logic [5:0]  shs;
  logic [4:0]  sh1, sh2;
  // cycle-2 group (op of cycle T-1)
  logic [75:0] ps;
  logic [36:0] pss;
  logic [15:0] ps1, ps2;
  logic        es, ess, es1, es2;
  logic        ts, tss, ts1, ts2;

  logic        st_o, sts_o, st1_o, st2_o;
  logic [75:0] sum_o;
  logic [36:0] sums_o;
  logic [15:0] sum1_o, sum2_o;
  logic        fs_o, fss_o, fs1_o, fs2_o;
  logic [3:0]  alarm;

  // FMA configuration: fp32 scalar / fp16 SIMD / fp8 (E4M3, 4-bit
  // precision) lanes — the module's internal slice constants assume it.
  transdot_decomp_addend_datapath_piped #(
    .SAFEDOT_CHECK (1'b1),
    .SUPER_MAN_BITS (23),
    .SUPER_MAN_BITS_SIMD (10),
    .SUPER_MAN_BITS_FP8 (3)
  ) dut (
    .clk_i(clk), .rst_ni(rst_n), .pipe_en(1'b1),
    .mantissa_c_i(mc), .product_shifted_i(ps), .addend_shamt_i(sh),
    .effective_subtraction_i(es), .tentative_sign_i(ts),
    .sticky_before_add_o(st_o), .sum_o(sum_o), .final_sign_o(fs_o),
    .simd_enable_i(simd), .is_fp8(fp8),
    .mantissa_c_simd_i(mcs), .product_shifted_simd_i(pss), .addend_shamt_simd_i(shs),
    .effective_subtraction_simd_i(ess), .tentative_sign_simd_i(tss),
    .sticky_before_add_simd_o(sts_o), .sum_simd_o(sums_o), .final_sign_simd_o(fss_o),
    .mantissa_c_fp8_1_i(mc1), .product_shifted_fp8_1_i(ps1), .addend_shamt_fp8_1_i(sh1),
    .effective_subtraction_fp8_1_i(es1), .tentative_sign_fp8_1_i(ts1),
    .sticky_before_add_fp8_1_o(st1_o), .sum_fp8_1_o(sum1_o), .final_sign_fp8_1_o(fs1_o),
    .mantissa_c_fp8_2_i(mc2), .product_shifted_fp8_2_i(ps2), .addend_shamt_fp8_2_i(sh2),
    .effective_subtraction_fp8_2_i(es2), .tentative_sign_fp8_2_i(ts2),
    .sticky_before_add_fp8_2_o(st2_o), .sum_fp8_2_o(sum2_o), .final_sign_fp8_2_o(fs2_o),
    .safedot_alarm_o(alarm)
  );

  int unsigned n_checked = 0, n_errors = 0;
  int unsigned warm_cnt = 0;
  bit expect_alarm = 1'b0;

  always @(posedge clk) begin
    if (!rst_n) warm_cnt <= 0;
    else if (warm_cnt < 10) warm_cnt <= warm_cnt + 1;
  end

  always @(posedge clk) begin
    if (rst_n && warm_cnt >= 6 && !expect_alarm) begin
      n_checked++;
      if (alarm !== 4'b0) begin
        n_errors++;
        if (n_errors <= 20)
          $display("[%0t] TB_ERROR: false alarm %b (simd=%0b fp8=%0b sh=%0d shs=%0d sh1=%0d sh2=%0d mc=%h mcs=%h mc1=%h mc2=%h es=%b)",
                   $time, alarm, simd, fp8, sh, shs, sh1, sh2, mc, mcs, mc1, mc2,
                   {es, ess, es1, es2});
      end
    end
  end

  // random shamts within architectural bounds, biased toward the
  // foreign-bit discard corners (0/1/2)
  function automatic logic [6:0] rnd_sh(input int unsigned bound);
    if (($urandom() % 4) == 0) return 7'($urandom() % 3);
    return 7'($urandom() % (bound + 1));
  endfunction

  task automatic drive_op();
    // payload group for op N
    mc  = $urandom();
    mcs = $urandom();
    mc1 = $urandom();
    mc2 = $urandom();
    if (simd && fp8) begin
      sh  = rnd_sh(16); shs = 6'(rnd_sh(16)); sh1 = 5'(rnd_sh(16)); sh2 = 5'(rnd_sh(16));
    end else if (simd) begin
      sh  = rnd_sh(37); shs = 6'(rnd_sh(37)); sh1 = '0; sh2 = '0;
    end else begin
      sh  = rnd_sh(76); shs = '0; sh1 = '0; sh2 = '0;
    end
    // cycle-2 group is driven fresh each cycle too: it belongs to the
    // PREVIOUS op, and since every op in a batch is random, driving new
    // randoms here is equivalent to delaying a queue by one cycle.
    ps  = {$urandom(), $urandom(), $urandom()};
    pss = {$urandom(), $urandom()};
    ps1 = $urandom();
    ps2 = $urandom();
    {es, ess, es1, es2} = $urandom();
    {ts, tss, ts1, ts2} = $urandom();
  endtask

  task automatic settle();
    expect_alarm = 1'b1;
    repeat (6) @(negedge clk);
    #0.1 expect_alarm = 1'b0;
  endtask

  // mode changes only behind an idle gap (the main-path contract); keeps
  // the alarm monitor masked across the transition.
  task automatic set_mode(input logic m_simd, input logic m_fp8);
    expect_alarm = 1'b1;
    // park the shamts inside every mode's architectural range before the
    // mode flip (an op launched under the old shamt but the new mode is
    // not a legal pipeline state)
    sh = '0; shs = '0; sh1 = '0; sh2 = '0;
    repeat (3) @(negedge clk);
    simd = m_simd; fp8 = m_fp8;
    repeat (3) @(negedge clk);
    #0.1 expect_alarm = 1'b0;
  endtask

  int unsigned n_inject = 0, n_detected = 0;
  task automatic inject_and_check(input string label);
    bit seen = 1'b0;
    expect_alarm = 1'b1;
    repeat (4) begin
      @(negedge clk);
      if (alarm !== 4'b0) seen = 1'b1;
    end
    n_inject++;
    if (seen) begin
      n_detected++;
      $display("[%0t] inject %-32s -> detected (alarm=%b)", $time, label, alarm);
    end else begin
      $display("[%0t] inject %-32s -> NOT detected", $time, label);
    end
  endtask

  logic [99:0] t_reg;
  logic [77:0] t_sum;
  logic        t_bit;
  logic [1:0]  t_res;

  initial begin
    rst_n = 0; simd = 0; fp8 = 0;
    mc = '0; mcs = '0; mc1 = '0; mc2 = '0;
    sh = '0; shs = '0; sh1 = '0; sh2 = '0;
    ps = '0; pss = '0; ps1 = '0; ps2 = '0;
    {es, ess, es1, es2} = '0; {ts, tss, ts1, ts2} = '0;
    repeat (6) @(negedge clk);
    rst_n = 1;
    repeat (4) @(negedge clk);

    // ---------------- fault-free phase: mode batches ----------------
    for (int b = 0; b < 300; b++) begin
      int unsigned m = $urandom() % 3;
      // idle gap at mode change (main-path contract); park shamts in-range
      {es, ess, es1, es2} = '0;
      sh = '0; shs = '0; sh1 = '0; sh2 = '0;
      repeat (3) @(negedge clk);
      simd = (m != 0);
      fp8  = (m == 2);
      repeat (2) @(negedge clk);
      repeat (1000) begin
        drive_op();
        @(negedge clk);
      end
    end
    $display("TB: fault-free phase done: %0d cycles checked, %0d errors",
             n_checked, n_errors);

    // ---------------- injections ----------------
    // 1) shifter register, scalar kept region (layer A)
    set_mode(1'b0, 1'b0); drive_op(); sh = 7'd10; repeat (3) @(negedge clk);
    t_reg = dut.addend_shift_full ^ (100'h1 << 60);
    force dut.addend_shift_full = t_reg;
    inject_and_check("shift reg kept bit (scalar)");
    release dut.addend_shift_full; settle();

    // 2) shifter register, scalar STICKY region (layer A covers sticky bits)
    drive_op(); sh = 7'd70; repeat (3) @(negedge clk);
    t_reg = dut.addend_shift_full ^ (100'h1 << 5);
    force dut.addend_shift_full = t_reg;
    inject_and_check("shift reg sticky bit (scalar)");
    release dut.addend_shift_full; settle();

    // 3) shifter register, fp8-mode quarter q2 (layer A, fp8 segmentation)
    set_mode(1'b1, 1'b1); drive_op(); repeat (3) @(negedge clk);
    t_reg = dut.addend_shift_full ^ (100'h1 << 65);
    force dut.addend_shift_full = t_reg;
    inject_and_check("shift reg q2 bit (fp8)");
    release dut.addend_shift_full; settle();

    // 4) positive adder output (layer B)
    set_mode(1'b0, 1'b0); drive_op(); es = 1'b0; repeat (3) @(negedge clk);
    t_sum = dut.sum_pos_merge ^ (78'h1 << 33);
    force dut.sum_pos_merge = t_sum;
    inject_and_check("sum_pos_merge bit (scalar)");
    release dut.sum_pos_merge; settle();

    // 5) negative adder output while selected (layer B): subtraction with a
    // large addend (sh=0) and tiny product so sum_carry=0 selects sum_neg.
    drive_op();
    sh = '0; mc = 24'hFFFFFF; es = 1'b1; ps = 76'h5;
    repeat (3) @(negedge clk);
    t_sum = dut.sum_neg_merge ^ (78'h1 << 20);
    force dut.sum_neg_merge = t_sum;
    inject_and_check("sum_neg_merge bit (selected)");
    release dut.sum_neg_merge; settle();

    // 6) sticky flag corruption (layer C; also perturbs the carry-inject)
    drive_op(); sh = 7'd70; repeat (3) @(negedge clk);
    t_bit = ~dut.sticky_before_add;
    force dut.sticky_before_add = t_bit;
    inject_and_check("sticky_before_add flip");
    release dut.sticky_before_add; settle();

    // 7) final sign corruption (layer D)
    drive_op(); es = 1'b1; repeat (3) @(negedge clk);
    t_bit = ~dut.final_sign;
    force dut.final_sign = t_bit;
    inject_and_check("final_sign flip");
    release dut.final_sign; settle();

    // 8) shadow-internal state (fail-safe direction): payload residue tap
    drive_op(); mc = 24'h000001; repeat (3) @(negedge clk);
    t_res = dut.g_safedot.sd_r_mc24_q ^ 2'b01;
    force dut.g_safedot.sd_r_mc24_q = t_res;
    inject_and_check("sd_r_mc24_q^1 (checker)");
    release dut.g_safedot.sd_r_mc24_q; settle();

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
