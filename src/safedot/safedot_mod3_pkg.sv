// SafeDot: mod-3 residue arithmetic primitives (SafeDot.md sec. 4).
//
// Residues live in 2 bits. Both 2'b00 and 2'b11 represent zero (one's-
// complement style); sd_canon3 maps to the canonical {0,1,2} encoding.
// All helpers accept non-canonical inputs and return canonical results.
//
// Key identities used throughout (m = 3, k = 2):
//   2^e mod 3 = 1 for even e, 2 for odd e  (so x2 == negate, since 2 == -1)
//   r(x * 2^e)      = rotate: multiply by 2 iff e odd          (identity 3)
//   r(-x mod 2^n)   = (2^n mod 3 - r(x)) mod 3 for x != 0, n even -> (1 - r(x));
//                     r(0) = 0 must be special-cased (zero flag) (identity 4)
//   r(x >> s)       = (r(x) - r(shifted-out bits)) * 2^(s mod 2) (identity 5)
package safedot_mod3_pkg;

  typedef logic [1:0] res3_t;

  // Canonicalize: 3 == 0 (mod 3).
  function automatic res3_t sd_canon3(input logic [1:0] x);
    return (x == 2'b11) ? 2'b00 : x;
  endfunction

  // (a + b) mod 3 via 2-bit end-around-carry addition.
  function automatic res3_t sd_add3(input logic [1:0] a, input logic [1:0] b);
    logic [2:0] s;
    logic [1:0] f;
    s = {1'b0, a} + {1'b0, b};       // max 6
    f = s[1:0] + {1'b0, s[2]};       // end-around carry, max 3
    return sd_canon3(f);
  endfunction

  // (-x) mod 3: 0->0, 1->2, 2->1 (bit swap; 3 canonicalizes to 0).
  function automatic res3_t sd_neg3(input logic [1:0] x);
    return sd_canon3({x[0], x[1]});
  endfunction

  // (a - b) mod 3.
  function automatic res3_t sd_sub3(input logic [1:0] a, input logic [1:0] b);
    return sd_add3(a, sd_neg3(b));
  endfunction

  // (a * b) mod 3.
  function automatic res3_t sd_mul3(input logic [1:0] a, input logic [1:0] b);
    res3_t ac, bc;
    ac = sd_canon3(a);
    bc = sd_canon3(b);
    if (ac == 2'd0 || bc == 2'd0) return 2'd0;
    return (ac[0] ^ bc[0]) ? 2'd2 : 2'd1;
  endfunction

  // r * 2^e mod 3 given the parity of e: x2 iff odd (2 == -1 mod 3).
  function automatic res3_t sd_mulpow2(input logic [1:0] r, input logic e_odd);
    return e_odd ? sd_neg3(r) : sd_canon3(r);
  endfunction

endpackage
