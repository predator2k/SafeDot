// SafeDot: K-generic low-cost residue arithmetic for modulus m = 2^K - 1.
//
// SystemVerilog packages cannot take parameters, so these primitives are
// written against a localparam and included INSIDE a parameterized scope
// (module or generate block) that first declares:
//
//   localparam int unsigned SD_K = <digit width>;   // m = 2^SD_K - 1
//
// Digits are SD_K bits in one's-complement style: both all-0 and all-1
// encode zero (x ≡ x mod m for any K-bit pattern). "Canonical" means the
// all-1 pattern is folded to 0 so digits compare with ==.
//
// Identity cheat sheet (all used by the shadow, K=2 forms in parentheses):
//   2^e mod m         = 2^(e mod K)            -> x2^e = rotate-left by e mod K
//                                                  (K=2: swap iff e odd)
//   -x mod m          = ~x                     (K=2: bit swap == complement)
//   -q mod 2^FW       : r = 2^(FW mod K) - r(q) for q != 0, r(0) = 0
//                                                  (K=2, FW even: 1 - r)
//   x = q*2^s + d     : r(q) = (r(x) - r(d)) * 2^(-s mod K)
//                       = rotate-right by s mod K (K=2: x2 iff s odd)
//   sign extension    : bits {2^a, 2^(a+1)} of a sign-extended lane add
//                       sign * (2^(a mod K) + 2^((a+1) mod K)) mod m
//                       (K=2: 1 + 2 = 3 == 0, i.e. free — mod-3 special!)
//
// End-around-carry addition WITHOUT canonicalization is closed on the
// redundant digit domain {0 .. 2^K-1}: a + b <= 2m, and after folding the
// carry (worth +1 == +2^K - m) the result is <= m. Trees exploit this by
// canonicalizing once at the root (see safedot_mod3_reduce).

typedef logic [SD_K-1:0] sd_res_t;

// Canonicalize: all-ones (== m) folds to 0.
function automatic sd_res_t sd_canon(input logic [SD_K-1:0] x);
  return (&x) ? '0 : x;
endfunction

// (a + b) as a redundant digit: end-around-carry add, no canonicalization.
function automatic sd_res_t sd_addr(input logic [SD_K-1:0] a,
                                    input logic [SD_K-1:0] b);
  logic [SD_K:0] s;
  s = {1'b0, a} + {1'b0, b};
  return s[SD_K-1:0] + {{(SD_K-1){1'b0}}, s[SD_K]};
endfunction

// (a + b) mod m, canonical.
function automatic sd_res_t sd_add(input logic [SD_K-1:0] a,
                                   input logic [SD_K-1:0] b);
  return sd_canon(sd_addr(a, b));
endfunction

// (-x) mod m: one's complement (x + ~x = m == 0), canonical.
function automatic sd_res_t sd_neg(input logic [SD_K-1:0] x);
  return sd_canon(~x);
endfunction

// (a - b) mod m, canonical.
function automatic sd_res_t sd_sub(input logic [SD_K-1:0] a,
                                   input logic [SD_K-1:0] b);
  return sd_add(a, ~b);
endfunction

// (a * b) mod m: K x K product folded once (2^K == 1), then a second EAC
// fold absorbs the carry of the first; canonical.
function automatic sd_res_t sd_mul(input logic [SD_K-1:0] a,
                                   input logic [SD_K-1:0] b);
  logic [2*SD_K-1:0] p;
  p = a * b;
  return sd_canon(sd_addr(p[SD_K-1:0], p[2*SD_K-1:SD_K]));
endfunction

// r * 2^rot mod m for a STATIC rotation amount (0 <= rot < K): rotate-left.
// Canonical in, canonical out (rotation cannot create all-ones from a
// canonical nonzero digit, and rotating all-zero stays zero).
function automatic sd_res_t sd_rols(input logic [SD_K-1:0] r,
                                    input int unsigned rot);
  logic [2*SD_K-1:0] d;
  d = {r, r} << (rot % SD_K);
  return d[2*SD_K-1:SD_K];
endfunction

// r * 2^e mod m for a DYNAMIC exponent e: rotate-left by e_modk, which the
// caller has already reduced to < K (e % K; synthesis folds the modulo to a
// bit-slice when K is a power of two).
function automatic sd_res_t sd_rold(input logic [SD_K-1:0] r,
                                    input logic [$clog2(SD_K > 1 ? SD_K : 2)-1:0] e_modk);
  logic [2*SD_K-1:0] d;
  d = {r, r} << e_modk;
  return d[2*SD_K-1:SD_K];
endfunction

// r * 2^(-e) mod m for a DYNAMIC exponent e (< K): rotate-right, used for
// the truncating-shift identity r(x >> s) = (r(x) - r(disc)) * 2^(-s).
function automatic sd_res_t sd_rord(input logic [SD_K-1:0] r,
                                    input logic [$clog2(SD_K > 1 ? SD_K : 2)-1:0] e_modk);
  logic [2*SD_K-1:0] d;
  d = {r, r} >> e_modk;
  return d[SD_K-1:0];
endfunction

// 2^e mod m as a canonical digit constant (elaboration-time helper).
function automatic sd_res_t sd_pow2(input int unsigned e);
  sd_res_t r;
  r = '0;
  r[e % SD_K] = 1'b1;
  return r;
endfunction
