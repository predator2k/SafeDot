// SafeDot: mod-3 residue extraction tree.
//
// Splits the input at an even bit position (2^even == 1 mod 3, so both
// halves carry unit weight) and reduces recursively; leaves are 2-bit
// digits folded with end-around-carry adds. Depth is O(log2 W).
// Constant-zero input slices (e.g. the low 12 bits of {x, 12'd0}) are
// pruned by synthesis automatically, so callers can pass padded vectors.
//
// Default implementation is the redundant-form tree: internal nodes stay in
// the redundant digit domain ({0,1,2,3} with 3 == 0, closed under EAC
// addition), so leaves are plain wires and per-node canonicalization
// disappears; the digits are canonicalized once at the public root (or not
// at all with CANON = 0, for consumers that only feed the digit into the
// canonicalization-tolerant pkg helpers). +define+SAFEDOT_TREE_LEGACY
// restores the v3 canonicalize-at-every-node tree as the A/B reference.
module safedot_mod3_reduce #(
  parameter int unsigned W = 8,
  // 1: emit the canonical {0,1,2} encoding (required when the result is
  //    compared with == / !=). 0: emit a redundant digit.
  parameter bit CANON = 1'b1
)(
  input  logic [W-1:0]           x_i,
  output safedot_mod3_pkg::res3_t r_o
);
  import safedot_mod3_pkg::*;

`ifdef SAFEDOT_TREE_LEGACY
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
`else
  generate
    if (W <= 2) begin : g_leaf
      logic [1:0] pad;
      assign pad = 2'(x_i);
      assign r_o = CANON ? sd_canon3(pad) : pad;
    end else begin : g_split
      // Even split point, roughly half.
      localparam int unsigned LOW = ((W / 2 + 1) / 2) * 2;
      res3_t r_lo, r_hi;
      safedot_mod3_reduce #(.W(LOW), .CANON(1'b0)) u_lo (
        .x_i(x_i[LOW-1:0]),
        .r_o(r_lo)
      );
      safedot_mod3_reduce #(.W(W - LOW), .CANON(1'b0)) u_hi (
        .x_i(x_i[W-1:LOW]),
        .r_o(r_hi)
      );
      assign r_o = CANON ? sd_add3(r_lo, r_hi) : sd_add3_r(r_lo, r_hi);
    end
  endgenerate
`endif
endmodule
