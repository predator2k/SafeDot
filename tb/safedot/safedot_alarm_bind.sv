// SafeDot FMA-level bring-up: bind an alarm monitor into every instance of
// the piped multiplier and require zero alarms across fault-free stimulus.
// Used with the existing FMA regression compiled with
//   +define+SAFEDOT_CHECK_EN+SAFEDOT_FMA_CHECK
// This exercises the shadow against the real operand packer, i.e. it tests
// the packer invariants the checker relies on (fp8/int4 4-bit lane payloads,
// fp16/int8 leading-zero halves) with architectural stimulus.
module safedot_alarm_monitor (
  input logic       clk_i,
  input logic       rst_ni,
  input logic [3:0] alarm
);
  int unsigned warm    = 0;
  int unsigned checked = 0;
  int unsigned alarms  = 0;

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      warm <= 0;
    end else begin
      if (warm < 12) warm <= warm + 1;
      if (warm >= 10) begin
        checked++;
        if (alarm !== 4'b0) begin
          alarms++;
          if (alarms <= 20)
            $display("[%0t] SAFEDOT_ALARM %b in %m", $time, alarm);
        end
      end
    end
  end

  final begin
    if (alarms == 0)
      $display("SAFEDOT_MONITOR PASS: %0d cycles checked, 0 alarms (%m)", checked);
    else
      $display("SAFEDOT_MONITOR FAIL: %0d cycles checked, %0d alarms (%m)",
               checked, alarms);
  end
endmodule

bind transdot_decomp_multiplier_w6_4lane_dp_piped safedot_alarm_monitor
  u_safedot_alarm_mon (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .alarm  (safedot_alarm_o)
  );

// Stage-1: same monitor on the addend-datapath shadow (its alarm output is
// unconnected inside the FMA, so the bind is the observation point).
bind transdot_decomp_addend_datapath_piped safedot_alarm_monitor
  u_safedot_addend_alarm_mon (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .alarm  (safedot_alarm_o)
  );
