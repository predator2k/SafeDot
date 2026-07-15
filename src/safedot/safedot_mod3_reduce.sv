// SafeDot: residue extraction tree for modulus m = 2^K - 1 (K = 2, i.e.
// mod 3, is the verified stage-0 default; the module name is kept from the
// mod-3 probe).
//
// Splits the input at bit positions that are multiples of K (2^(multiple
// of K) == 1 mod m, so both halves carry unit weight) and reduces
// recursively; leaves are K-bit digits. Depth is O(log W). Constant-zero
// input slices (e.g. the low 12 bits of {x, 12'd0}) are pruned by
// synthesis automatically, so callers can pass padded vectors.
//
// Default implementation is the redundant-form tree: internal nodes stay in
// the redundant digit domain ({0 .. 2^K-1} with all-ones == 0, closed under
// EAC addition), so leaves are plain wires and per-node canonicalization
// disappears; the digits are canonicalized once at the public root (or not
// at all with CANON = 0, for consumers that only feed the digit into the
// canonicalization-tolerant helpers). +define+SAFEDOT_TREE_LEGACY
// restores the v3 canonicalize-at-every-node tree (the 0.8 ns closure
// configuration for K = 2).
module safedot_mod3_reduce #(
  parameter int unsigned W = 8,
  parameter int unsigned K = 2,
  // 1: emit the canonical encoding (required when the result is compared
  //    with == / !=). 0: emit a redundant digit.
  parameter bit CANON = 1'b1
)(
  input  logic [W-1:0] x_i,
  output logic [K-1:0] r_o
);
  localparam int unsigned SD_K = K;
  `include "safedot_res.svh"

  generate
    if (W <= K) begin : g_leaf
      logic [K-1:0] pad;
      assign pad = K'(x_i);
`ifdef SAFEDOT_TREE_LEGACY
      assign r_o = sd_canon(pad);
`else
      assign r_o = CANON ? sd_canon(pad) : pad;
`endif
    end else begin : g_split
      // Split at a multiple of K, roughly half.
      localparam int unsigned LOW = (((W / 2) + K - 1) / K) * K;
      logic [K-1:0] r_lo, r_hi;
      safedot_mod3_reduce #(.W(LOW), .K(K), .CANON(1'b0)) u_lo (
        .x_i(x_i[LOW-1:0]),
        .r_o(r_lo)
      );
      safedot_mod3_reduce #(.W(W - LOW), .K(K), .CANON(1'b0)) u_hi (
        .x_i(x_i[W-1:LOW]),
        .r_o(r_hi)
      );
`ifdef SAFEDOT_TREE_LEGACY
      assign r_o = sd_add(r_lo, r_hi);
`else
      assign r_o = CANON ? sd_add(r_lo, r_hi) : sd_addr(r_lo, r_hi);
`endif
    end
  endgenerate
endmodule
