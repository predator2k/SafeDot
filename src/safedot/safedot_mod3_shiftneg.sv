// SafeDot: mod-3 residue prediction for a truncating right shift followed by
// an optional two's-complement negate, mirroring the compare_shift_xor lane
// structure of the TransDot DP multiplier:
//
//   x     = x_live * 2^LO          (payload placed LO bits up in an FW field)
//   q     = x >> shamt             (shifted-out low bits discarded)
//   y     = neg ? (~q + 1) : q     (negate in the FW-bit field)
//
// Identity 5: r(q) = (r(x) - r(d)) * 2^(shamt mod 2), d = discarded bits
//             = (x_live & mask) * 2^LO with mask covering max(shamt-LO,0) bits.
// Identity 4: r(-q mod 2^FW) = (1 - r(q)) mod 3 for q != 0 and even FW;
//             q == 0 (all payload discarded or zero payload) maps to 0 and is
//             caught with a zero flag on the kept payload bits.
//
// Only live payload bits cross the port boundary (the flow compiles with
// boundary optimization disabled, so constant port bits would not be pruned).
// rx_i is the shadow-predicted residue of the FIELD value x; x_live_i is
// tapped from the main path only for the discarded-segment extraction and
// the zero flag.
module safedot_mod3_shiftneg #(
  parameter int unsigned FW = 24,  // full field width; must be even
  parameter int unsigned LW = 8,   // live payload width
  parameter int unsigned LO = 14,  // payload offset within the field
  parameter int unsigned SW = 5    // shift-amount width
)(
  input  logic [LW-1:0]           x_live_i,
  input  safedot_mod3_pkg::res3_t rx_i,
  input  logic [SW-1:0]           shamt_i,
  input  logic                    neg_i,
  output safedot_mod3_pkg::res3_t r_o,
  // q == 0 flag (all payload discarded or zero payload); with leading-zero
  // payload packing this also gives the lane's two's-complement sign:
  // sign(y) == neg_i && !q_zero_o.
  output logic                    q_zero_o
);
  import safedot_mod3_pkg::*;

  localparam bit LO_ODD = (LO % 2) == 1;

  initial begin
    if (FW % 2 != 0) $fatal(1, "safedot_mod3_shiftneg: FW must be even");
  end

  // Effective shift into the payload: max(shamt - LO, 0), saturating.
  logic [SW:0]   sdiff;
  logic [SW-1:0] s_eff;
  assign sdiff = {1'b0, shamt_i} - (SW+1)'(LO);
  assign s_eff = sdiff[SW] ? '0 : sdiff[SW-1:0];

  logic [LW-1:0] mask_live, disc_live, kept_live;
  assign mask_live = ~({LW{1'b1}} << s_eff);  // s_eff >= LW -> all ones
  assign disc_live = x_live_i & mask_live;
  assign kept_live = x_live_i & ~mask_live;

  res3_t r_disc_live, r_disc, r_shift;
  safedot_mod3_reduce #(.W(LW)) u_rdisc (
    .x_i(disc_live),
    .r_o(r_disc_live)
  );
  assign r_disc  = sd_mulpow2(r_disc_live, LO_ODD);
  assign r_shift = sd_mulpow2(sd_sub3(rx_i, r_disc), shamt_i[0]);

  logic q_zero;
  assign q_zero = ~(|kept_live);

  assign r_o      = neg_i ? (q_zero ? 2'd0 : sd_sub3(2'd1, r_shift)) : r_shift;
  assign q_zero_o = q_zero;
endmodule
