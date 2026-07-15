// SafeDot: residue prediction (modulus m = 2^K - 1; module name kept from
// the K = 2 stage-0 probe) for a truncating right shift followed by an
// optional two's-complement negate, mirroring the compare_shift_xor lane
// structure of the TransDot DP multiplier:
//
//   x     = x_live * 2^LO          (payload placed LO bits up in an FW field)
//   q     = x >> shamt             (shifted-out low bits discarded)
//   y     = neg ? (~q + 1) : q     (negate in the FW-bit field)
//
// Identity 5: r(q) = (r(x) - r(d)) * 2^(-shamt mod K), d = discarded bits
//             = (x_live & mask) * 2^LO with mask covering max(shamt-LO,0) bits.
//             Both x and d carry the same 2^LO placement, so
//             r(x) - r(d) = (r(x_live) - r(disc_live)) * 2^(LO mod K)
//             and the rotation is applied once, after the subtraction.
// Identity 4: r(-q mod 2^FW) = (2^(FW mod K) - r(q)) mod m for q != 0;
//             q == 0 (all payload discarded or zero payload) maps to 0 and is
//             caught with a zero flag on the kept payload bits.
//
// Only live payload bits cross the port boundary (the flow compiles with
// boundary optimization disabled, so constant port bits would not be pruned).
// rx_i is the shadow-predicted residue of the live PAYLOAD value (x_live,
// i.e. WITHOUT the 2^LO placement — for K = 2 the distinction is invisible
// because LO is even in both TransDot uses); x_live_i is tapped from the
// main path only for the discarded-segment extraction and the zero flag.
module safedot_mod3_shiftneg #(
  parameter int unsigned FW = 24,  // full field width
  parameter int unsigned LW = 8,   // live payload width
  parameter int unsigned LO = 14,  // payload offset within the field
  parameter int unsigned SW = 5,   // shift-amount width
  parameter int unsigned K  = 2    // modulus 2^K - 1
)(
  input  logic [LW-1:0] x_live_i,
  input  logic [K-1:0]  rx_i,
  input  logic [SW-1:0] shamt_i,
  input  logic          neg_i,
  output logic [K-1:0]  r_o,
  // q == 0 flag (all payload discarded or zero payload); with leading-zero
  // payload packing this also gives the lane's two's-complement sign:
  // sign(y) == neg_i && !q_zero_o.
  output logic          q_zero_o
);
  localparam int unsigned SD_K = K;
  `include "safedot_res.svh"

  // Effective shift into the payload: max(shamt - LO, 0), saturating.
  logic [SW:0]   sdiff;
  logic [SW-1:0] s_eff;
  assign sdiff = {1'b0, shamt_i} - (SW+1)'(LO);
  assign s_eff = sdiff[SW] ? '0 : sdiff[SW-1:0];

  logic [LW-1:0] mask_live, disc_live, kept_live;
  assign mask_live = ~({LW{1'b1}} << s_eff);  // s_eff >= LW -> all ones
  assign disc_live = x_live_i & mask_live;
  assign kept_live = x_live_i & ~mask_live;

  // Redundant digit is fine here: r_disc_live only feeds canonicalization-
  // tolerant helpers.
  sd_res_t r_disc_live, r_shift;
  safedot_mod3_reduce #(.W(LW), .K(K), .CANON(1'b0)) u_rdisc (
    .x_i(disc_live),
    .r_o(r_disc_live)
  );

  // shamt mod K for the rotate (a bit-slice when K is a power of two).
  localparam int unsigned MW = $clog2(K > 1 ? K : 2);
  logic [MW-1:0] s_modk;
  assign s_modk = MW'(shamt_i % K);
  assign r_shift = sd_rord(sd_rols(sd_sub(rx_i, r_disc_live), LO), s_modk);

  logic q_zero;
  assign q_zero = ~(|kept_live);

  assign r_o      = neg_i ? (q_zero ? '0 : sd_sub(sd_pow2(FW), r_shift))
                          : r_shift;
  assign q_zero_o = q_zero;
endmodule
