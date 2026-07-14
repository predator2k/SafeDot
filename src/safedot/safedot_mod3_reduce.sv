// SafeDot: mod-3 residue extraction tree.
//
// Splits the input at an even bit position (2^even == 1 mod 3, so both
// halves carry unit weight) and reduces recursively; leaves are 2-bit
// digits folded with end-around-carry adds. Depth is O(log2 W).
// Constant-zero input slices (e.g. the low 12 bits of {x, 12'd0}) are
// pruned by synthesis automatically, so callers can pass padded vectors.
module safedot_mod3_reduce #(
  parameter int unsigned W = 8
)(
  input  logic [W-1:0]           x_i,
  output safedot_mod3_pkg::res3_t r_o
);
  import safedot_mod3_pkg::*;

  generate
    if (W <= 2) begin : g_leaf
      logic [1:0] pad;
      assign pad = 2'(x_i);
      assign r_o = sd_canon3(pad);
    end else begin : g_split
      // Even split point, roughly half.
      localparam int unsigned LOW = ((W / 2 + 1) / 2) * 2;
      res3_t r_lo, r_hi;
      safedot_mod3_reduce #(.W(LOW)) u_lo (
        .x_i(x_i[LOW-1:0]),
        .r_o(r_lo)
      );
      safedot_mod3_reduce #(.W(W - LOW)) u_hi (
        .x_i(x_i[W-1:LOW]),
        .r_o(r_hi)
      );
      assign r_o = sd_add3(r_lo, r_hi);
    end
  endgenerate
endmodule
