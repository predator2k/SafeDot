module transdot_decomp_addend_datapath_piped #(
`ifdef SAFEDOT_FMA_CHECK
  // SafeDot: with +define+SAFEDOT_FMA_CHECK the residue shadow is enabled in
  // every instance without interface changes (mirrors the multiplier).
  parameter bit          SAFEDOT_CHECK  = 1'b1,
`else
  parameter bit          SAFEDOT_CHECK  = 1'b0,
`endif
  parameter int unsigned SUPER_MAN_BITS     = 23,  // mantissa width of superformat
  parameter int unsigned PRECISION_BITS     = SUPER_MAN_BITS + 1,  // mantissa precision bits (FP32: 24)
  parameter int unsigned SHIFT_AMOUNT_WIDTH = $clog2(3 * PRECISION_BITS + 5),   // shift amount bit width (for FP32: log2(77)=7)
   // ------------
   // SIMD lane 1 parameters
  parameter int unsigned SUPER_MAN_BITS_SIMD     = 10,  // mantissa width of superformat
  parameter int unsigned PRECISION_BITS_SIMD     = SUPER_MAN_BITS_SIMD + 1,  // mantissa precision bits (FP16: 11)
  parameter int unsigned SHIFT_AMOUNT_WIDTH_SIMD = $clog2(3 * PRECISION_BITS_SIMD + 5),   // shift amount bit width (for FP16: log2(38)=6)

     // SIMD lane 2/3 parameters
  parameter int unsigned SUPER_MAN_BITS_FP8     = 4,  // mantissa width of superformat
  parameter int unsigned PRECISION_BITS_FP8     = SUPER_MAN_BITS_FP8 + 1,  // mantissa precision bits (FP8: 11)
  parameter int unsigned SHIFT_AMOUNT_WIDTH_FP8 = $clog2(3 * PRECISION_BITS_FP8 + 5)   // shift amount bit width (for FP8: log2(38)=6)

)(
  // ---------------- Inputs ----------------
  input  logic                               clk_i,
  input  logic                               rst_ni,
  input  logic                               pipe_en,

  // ---------------- Main lane Inputs ----------------
  input  logic [PRECISION_BITS-1:0]          mantissa_c_i,         // raw mantissa of operand C
  input  logic [3*PRECISION_BITS+3:0]        product_shifted_i,    // shifted product mantissa //76-bit
  input  logic [SHIFT_AMOUNT_WIDTH-1:0]      addend_shamt_i,       // exponent diff shift
  input  logic                               effective_subtraction_i,
  input  logic                               tentative_sign_i,

  // ---------------- Outputs ----------------
  output logic                               sticky_before_add_o,
  output logic [3*PRECISION_BITS+3:0]        sum_o,
  output logic                               final_sign_o,

  //mode select for SIMD
  input  logic                               simd_enable_i,
  input  logic                               is_fp8,

    // ---------------- SIMD lane 1 Inputs ----------------
  input  logic [PRECISION_BITS_SIMD-1:0]          mantissa_c_simd_i,         // raw mantissa of operand C
  input  logic [3*PRECISION_BITS_SIMD+3:0]        product_shifted_simd_i,    // shifted product mantissa //37-bit
  input  logic [SHIFT_AMOUNT_WIDTH_SIMD-1:0]      addend_shamt_simd_i,       // exponent diff shift
  input  logic                               effective_subtraction_simd_i,
  input  logic                               tentative_sign_simd_i,
  // ---------------- SIMD lane 1 Outputs ----------------
  output logic                               sticky_before_add_simd_o,
  output logic [3*PRECISION_BITS_SIMD+3:0]        sum_simd_o,
  output logic                               final_sign_simd_o,

      // ---------------- SIMD lane 2 Inputs ----------------
  input  logic [PRECISION_BITS_FP8-1:0]          mantissa_c_fp8_1_i,         // raw mantissa of operand C
  input  logic [3*PRECISION_BITS_FP8+3:0]        product_shifted_fp8_1_i,    // shifted product mantissa //37-bit
  input  logic [SHIFT_AMOUNT_WIDTH_FP8-1:0]      addend_shamt_fp8_1_i,       // exponent diff shift
  input  logic                               effective_subtraction_fp8_1_i,
  input  logic                               tentative_sign_fp8_1_i,
  // ---------------- SIMD lane 2 Outputs ----------------
  output logic                               sticky_before_add_fp8_1_o,
  output logic [3*PRECISION_BITS_FP8+3:0]        sum_fp8_1_o,
  output logic                               final_sign_fp8_1_o,

      // ---------------- SIMD lane 3 Inputs ----------------
  input  logic [PRECISION_BITS_FP8-1:0]          mantissa_c_fp8_2_i,         // raw mantissa of operand C
  input  logic [3*PRECISION_BITS_FP8+3:0]        product_shifted_fp8_2_i,    // shifted product mantissa //37-bit
  input  logic [SHIFT_AMOUNT_WIDTH_FP8-1:0]      addend_shamt_fp8_2_i,       // exponent diff shift
  input  logic                               effective_subtraction_fp8_2_i,
  input  logic                               tentative_sign_fp8_2_i,
  // ---------------- SIMD lane 3 Outputs ----------------
  output logic                               sticky_before_add_fp8_2_o,
  output logic [3*PRECISION_BITS_FP8+3:0]        sum_fp8_2_o,
  output logic                               final_sign_fp8_2_o,
  // SafeDot residue-shadow alarm: [0] shifter-register segment checks,
  // [1] adder / selected-sum checks, [2] sticky-flag replicas,
  // [3] final-sign mirrors. Constant 0 when the shadow is disabled; flows
  // that do not compile src/safedot/* are unaffected.
  output logic [3:0]                         safedot_alarm_o
);

// --------------------------------------------------------------------------
  // Internal signals
  // --------------------------------------------------------------------------
  localparam int unsigned STICKY_WIDTH = PRECISION_BITS;

  logic [STICKY_WIDTH-1:0]     addend_sticky_bits;
  logic                        sticky_before_add;
  logic [3*PRECISION_BITS+4-1:0] addend_after_shift;
  logic [3*PRECISION_BITS+4-1:0] addend_shifted;
  logic                        inject_carry_in;

  logic [3*PRECISION_BITS+5-1:0] sum_pos, sum_neg;
  logic                        sum_carry;
  logic [3*PRECISION_BITS+4-1:0] sum;
  logic                        final_sign;

  // ------------
  // SIMD lane 1
  localparam int unsigned STICKY_WIDTH_SIMD = PRECISION_BITS_SIMD;

  logic [STICKY_WIDTH_SIMD-1:0]     addend_sticky_bits_simd;
  logic                        sticky_before_add_simd;
  logic [3*PRECISION_BITS_SIMD+4-1:0] addend_after_shift_simd;
  logic [3*PRECISION_BITS_SIMD+4-1:0] addend_shifted_simd;
  logic                        inject_carry_in_simd;
  logic [3*PRECISION_BITS_SIMD+5-1:0] sum_pos_simd, sum_neg_simd;
  logic                        sum_carry_simd;
  logic [3*PRECISION_BITS_SIMD+4-1:0] sum_simd;
  logic                        final_sign_simd;

  // ------------
  // SIMD lane 2
  localparam int unsigned STICKY_WIDTH_FP8 = PRECISION_BITS_FP8;

  logic [STICKY_WIDTH_FP8-1:0]     addend_sticky_bits_fp8_1;
  logic                        sticky_before_add_fp8_1;
  logic [3*PRECISION_BITS_FP8+4-1:0] addend_after_shift_fp8_1;
  logic [3*PRECISION_BITS_FP8+4-1:0] addend_shifted_fp8_1;
  logic                        inject_carry_in_fp8_1;
  logic [3*PRECISION_BITS_FP8+5-1:0] sum_pos_fp8_1, sum_neg_fp8_1;
  logic                        sum_carry_fp8_1;
  logic [3*PRECISION_BITS_FP8+4-1:0] sum_fp8_1;
  logic                        final_sign_fp8_1;


  // ------------
  // SIMD lane 3
  logic [STICKY_WIDTH_FP8-1:0]     addend_sticky_bits_fp8_2;
  logic                        sticky_before_add_fp8_2;
  logic [3*PRECISION_BITS_FP8+4-1:0] addend_after_shift_fp8_2;
  logic [3*PRECISION_BITS_FP8+4-1:0] addend_shifted_fp8_2;
  logic                        inject_carry_in_fp8_2;
  logic [3*PRECISION_BITS_FP8+5-1:0] sum_pos_fp8_2, sum_neg_fp8_2;
  logic                        sum_carry_fp8_2;
  logic [3*PRECISION_BITS_FP8+4-1:0] sum_fp8_2;
  logic                        final_sign_fp8_2;

  localparam int unsigned SHIFT_TOTAL_W = 4 * PRECISION_BITS + 4; //100
  localparam int unsigned SHIFT_HALF_W = SHIFT_TOTAL_W/2; //50
  localparam int unsigned SHIFT_QUARTER_W = SHIFT_TOTAL_W/4; //25
  localparam int unsigned SHIFT_SIMD_W = 4 * PRECISION_BITS_SIMD + 4; //48
  localparam int unsigned SHIFT_FP8_W = 4 * PRECISION_BITS_FP8 + 4; //20
  //assert(SHIFT_HALF_W >= SHIFT_SIMD_W);

  // --------------------------------------------------------------------------
  // Addend right-shift alignment with sticky-bit compression
  // --------------------------------------------------------------------------
  // BEFORE: mantissa_c | zeros(3p+4)
  // AFTER:  right-shifted with sticky bits captured in low p bits
  // 4*PRECISION_BITS+4   = 24*4+4 = 100 bit total bits shifter
  //assign {addend_after_shift, addend_sticky_bits} = (mantissa_c_i << (3 * PRECISION_BITS + 4)) >> addend_shamt_i;
  // 11*4+4 = 48 bit total bits shifter
  //assign {addend_after_shift_simd, addend_sticky_bits_simd} = (mantissa_c_simd_i << (3 * PRECISION_BITS_SIMD + 4)) >> addend_shamt_simd_i;
  logic [99:0] preshift_mantissa,addend_shift_full,addend_shift_full_d;
  assign preshift_mantissa = simd_enable_i ? 
                  is_fp8? {mantissa_c_i[23:20], {(21){1'b0}},mantissa_c_fp8_1_i, {(21){1'b0}}, mantissa_c_simd_i[10:7], {(21){1'b0}}, mantissa_c_fp8_2_i, {(21){1'b0}}}
                : {mantissa_c_i[23:13], {(39){1'b0}},mantissa_c_simd_i, {(39){1'b0}}} 
                : {mantissa_c_i, {(76){1'b0}}};
  //in normal mode, the simd lane1 is not used.
  
  //transdot_decomp_shifter #(
  //  .Half_N (SHIFT_HALF_W)
  //) i_addend_shifter (
  //  .dp_enable_i (simd_enable_i),
  //  .data_i      (preshift_mantissa),
  //  .shamt0_i    (addend_shamt_i),
  //  .shamt1_i    (addend_shamt_simd_i),
  //  .data_o      (addend_shift_full)
  //);

 transdot_decomp_shifter_w4 #(
  .QUARTER_N(25)
 ) i_decomp_addend_shifter (
  // 00: 1x N shifter
  // 01: 2x HALF_N shifters
  // 10: 4x QUARTER_N shifters
  // 11: invalid
  .mode_i(simd_enable_i? (is_fp8? 2'b10:2'b01) : 2'b00),

  .data_i (preshift_mantissa),

  .shamt3_i (addend_shamt_i),
  .shamt2_i (addend_shamt_fp8_1_i),
  .shamt1_i (addend_shamt_simd_i ),
  .shamt0_i (addend_shamt_fp8_2_i ),

  .data_o (addend_shift_full_d)
);

`ifdef COMBINATIONAL
  assign addend_shift_full = addend_shift_full_d;
`else
 always_ff @(posedge clk_i or negedge rst_ni) begin
   if (!rst_ni) begin
     addend_shift_full <= '0;
   end else if (pipe_en) begin
     addend_shift_full <= addend_shift_full_d;
   end
 end
`endif
  
  assign addend_after_shift = simd_enable_i? is_fp8?
        {'0,addend_shift_full[99:84]} //put to original 3P+4 position
        :{'0,addend_shift_full[99:63]} //put to original 4P+4 position
        : addend_shift_full[99:24]; //The higher 3*PRECISION_BITS+4 bits
  assign addend_sticky_bits = simd_enable_i? is_fp8?
                   {addend_shift_full[83:80]} 
                  :{addend_shift_full[62:52]} 
                  : addend_shift_full[23:0];

  //assign {addend_after_shift_simd, addend_sticky_bits_simd} = addend_shift_full[49:2];
  assign addend_after_shift_simd = is_fp8? {'0,addend_shift_full[49:34]} : addend_shift_full[49:13]; //put to original 3P+4 position
  assign addend_sticky_bits_simd = is_fp8? {addend_shift_full[33:30]} : {addend_shift_full[12:2]};

  assign sticky_before_add = (| addend_sticky_bits);
  // ------------
  // SIMD lane 1
  assign sticky_before_add_simd = (| addend_sticky_bits_simd);

   // ------------
  // SIMD lane 2
  assign {addend_after_shift_fp8_1, addend_sticky_bits_fp8_1} = addend_shift_full[74:55];
  assign sticky_before_add_fp8_1 = (| addend_sticky_bits_fp8_1);

   // ------------
  // SIMD lane 3
  assign {addend_after_shift_fp8_2, addend_sticky_bits_fp8_2} = addend_shift_full[24: 5];
  assign sticky_before_add_fp8_2 = (| addend_sticky_bits_fp8_2);

  // --------------------------------------------------------------------------
  // Handle subtraction (invert addend if subtraction)
  // --------------------------------------------------------------------------
  assign addend_shifted  = (effective_subtraction_i)
                           ? ~addend_after_shift
                           :  addend_after_shift;

  // Inject carry only when subtraction and no sticky (two’s complement negation)
  assign inject_carry_in = effective_subtraction_i & ~sticky_before_add;

  // ------------
  // SIMD lane 1
  assign addend_shifted_simd  = (effective_subtraction_simd_i)
                           ? ~addend_after_shift_simd
                           :  addend_after_shift_simd;

  assign inject_carry_in_simd = effective_subtraction_simd_i & ~sticky_before_add_simd;

  // ------------
  // SIMD lane 2
  assign addend_shifted_fp8_1  = (effective_subtraction_fp8_1_i)
                           ? ~addend_after_shift_fp8_1
                           :  addend_after_shift_fp8_1;

  assign inject_carry_in_fp8_1 = effective_subtraction_fp8_1_i & ~sticky_before_add_fp8_1;

  // ------------
  // SIMD lane 3
  assign addend_shifted_fp8_2  = (effective_subtraction_fp8_2_i)
                           ? ~addend_after_shift_fp8_2
                           :  addend_after_shift_fp8_2;
  assign inject_carry_in_fp8_2 = effective_subtraction_fp8_2_i & ~sticky_before_add_fp8_2;

  // --------------------------------------------------------------------------

  // --------------------------------------------------------------------------
  // Mantissa adder (unsigned)
  // --------------------------------------------------------------------------
  //adder_unsigned #(
  //  .IN_WIDTH  (3*PRECISION_BITS + 4),
  //  .OUT_WIDTH (3*PRECISION_BITS + 5)
  //) i_mantissa_adder (
  //  .a_i        (product_shifted_i),
  //  .b_i        (addend_shifted),
  //  .carry_in_i (inject_carry_in),
  //  .sum_o      (sum_pos)
  //);
  //adder_unsigned #(
  //  .IN_WIDTH  (3*PRECISION_BITS_SIMD + 4),
  //  .OUT_WIDTH (3*PRECISION_BITS_SIMD + 5)
  //) i_mantissa_adder_simd (
  //  .a_i        (product_shifted_simd_i),
  //  .b_i        (addend_shifted_simd),
  //  .carry_in_i (inject_carry_in_simd),
  //  .sum_o      (sum_pos_simd)
  //);
  logic [75:0] product_shifted_merge,product_shifted_merge_neg,addend_shifted_merge;
  localparam logic [75:0] FP8_NEG_CLEAR_MASK = {1'b1, 18'b0, 1'b1, 18'b0, 1'b1, 18'b0, 1'b1, 18'b0};

  assign product_shifted_merge = simd_enable_i ? is_fp8?
        {3'd0,product_shifted_fp8_2_i[15:0], 3'd0,product_shifted_simd_i[29:14],3'd0,product_shifted_fp8_1_i[15:0],3'd0,product_shifted_i[55:40]}
        :{1'b0,product_shifted_simd_i[36:0],1'b0,product_shifted_i[62:26]}
        : product_shifted_i ; //extend 1 bit for uniform width

  // Reuse the already-built mode-merged operand, then apply the FP8 lane-mask
  // fixup (clear lane-MSB complement bits) to match the original encoding.
  assign product_shifted_merge_neg = (simd_enable_i && is_fp8)
        ? ((~product_shifted_merge) & ~FP8_NEG_CLEAR_MASK)
        :  (~product_shifted_merge);

  assign addend_shifted_merge = simd_enable_i ? is_fp8?
        {3'd0,addend_shifted_fp8_2[15:0], 3'd0, addend_shifted_simd[15:0] ,3'd0,addend_shifted_fp8_1[15:0],3'd0,addend_shifted[15:0]}
        :{1'b0,addend_shifted_simd[36:0], 1'b0,addend_shifted[36:0]} 
        : addend_shifted ; //extend 1 bit for uniform width


  logic [77:0] sum_pos_merge;

  transdot_decomp_adder_w4 #(
    .QUARTER_N  (19) //3*24+4=76, in simd mode, it will be 38bit+38bit. {1'b0,37bit product_shifted_simd_i, 1'b0,37bit product_shifted_i}
  ) i_mantissa_adder_pos (
    .simd_enable_i (simd_enable_i),
    .is_fp8       (is_fp8),
    .a_i        (product_shifted_merge),
    .b_i        (addend_shifted_merge),
    .lane0_cin     (inject_carry_in),
    .lane1_cin     (inject_carry_in_fp8_1),
    .lane2_cin     (inject_carry_in_simd),
    .lane3_cin     (inject_carry_in_fp8_2),
    .sum_o      (sum_pos_merge)
  );

  assign sum_pos = simd_enable_i ? (is_fp8?
        {'0,sum_pos_merge[16:0],{40{1'b0}}} //put to original 4P+4 position
        :{'0,sum_pos_merge[37:0],{26{1'b0}}}) //put to original 3P+4 position
        : sum_pos_merge[76:0]; //The higher 3*PRECISION_BITS+5 bits

  assign sum_pos_simd = is_fp8?
          {'0,sum_pos_merge[54: 38],{14{1'b0}}}
          :sum_pos_merge[75:38];

  assign sum_pos_fp8_1 = sum_pos_merge[35:19];
  assign sum_pos_fp8_2 = sum_pos_merge[73:57];

  assign sum_carry = simd_enable_i ? (is_fp8?  sum_pos_merge[16]:sum_pos_merge[37]) :sum_pos_merge[76];

  assign sum_carry_simd = is_fp8?  sum_pos_merge[54]:sum_pos_merge[75];

  assign sum_carry_fp8_1 = sum_pos_merge[35];
  assign sum_carry_fp8_2 = sum_pos_merge[73];


  // --------------------------------------------------------------------------
  // Negative sum computation (for subtraction cases)
  // --------------------------------------------------------------------------
  logic [3*PRECISION_BITS+4-1:0] addend_after_shift_merge;
  logic [77:0] sum_neg_merge;
  assign addend_after_shift_merge = simd_enable_i ? is_fp8?
        {3'd0,addend_after_shift_fp8_2[15:0], 3'd0,addend_after_shift_simd[15:0] ,3'd0,addend_after_shift_fp8_1[15:0],3'd0,addend_after_shift[15:0]}
        :{1'b0,addend_after_shift_simd[36:0], 1'b0,addend_after_shift[36:0]}
        : addend_after_shift ; //extend 1 bit for uniform width
  transdot_decomp_adder_w4 #(
    .QUARTER_N  (19) //3*24+4=76, in simd mode, it will be 38bit+38bit. {1'b0,37bit product_shifted_simd_i, 1'b0,37bit product_shifted_i}
    ) i_mantissa_adder_neg (
    .simd_enable_i (simd_enable_i),
    .is_fp8       (is_fp8),
    .a_i        (addend_after_shift_merge),
    .b_i        (product_shifted_merge_neg),
    .lane0_cin     (1'd1),
    .lane1_cin     (1'd1),
    .lane2_cin     (1'd1),
    .lane3_cin     (1'd1),
    .sum_o      (sum_neg_merge)
  );

  assign sum_neg = simd_enable_i? is_fp8? 
                  $signed({sum_neg_merge[16:0],{40{1'b0}}})
                  :$signed({sum_neg_merge[37:0],{26{1'b0}}})
                  :sum_neg_merge;

  assign sum_neg_simd = is_fp8? 
           {sum_neg_merge[54: 38],{14{1'b0}}}
          :sum_neg_merge[75: 38];

  assign sum_neg_fp8_1 = sum_neg_merge[35:19];
  assign sum_neg_fp8_2 = sum_neg_merge[73:57];


  //assign sum_neg = simd_enable_i? ({'0,addend_after_shift,{(2*PRECISION_BITS-2*PRECISION_BITS_SIMD){1'b0}}} - product_shifted_i):(addend_after_shift - product_shifted_i);
  //assign sum_neg_simd = addend_after_shift_simd - product_shifted_simd_i;

  // Select proper sum result
  assign sum = (effective_subtraction_i && ~sum_carry)
               ? sum_neg[3*PRECISION_BITS+3:0]
               : sum_pos[3*PRECISION_BITS+3:0];

  // Select proper sum result
  assign sum_simd = (effective_subtraction_simd_i && ~sum_carry_simd)
               ? sum_neg_simd[3*PRECISION_BITS_SIMD+3:0]
               : sum_pos_simd[3*PRECISION_BITS_SIMD+3:0];

  assign sum_fp8_1 = (effective_subtraction_fp8_1_i && ~sum_carry_fp8_1)
               ? sum_neg_fp8_1[3*PRECISION_BITS_FP8+3:0]
               : sum_pos_fp8_1[3*PRECISION_BITS_FP8+3:0];
  
  assign sum_fp8_2 = (effective_subtraction_fp8_2_i && ~sum_carry_fp8_2)
               ? sum_neg_fp8_2[3*PRECISION_BITS_FP8+3:0]
               : sum_pos_fp8_2[3*PRECISION_BITS_FP8+3:0];
  // --------------------------------------------------------------------------
  // Final sign determination
  // --------------------------------------------------------------------------

  assign final_sign =
    (effective_subtraction_i && (sum_carry == tentative_sign_i))
      ? 1'b1
      : (effective_subtraction_i ? 1'b0 : tentative_sign_i);
    // ------------
    // SIMD lane 1
    assign final_sign_simd =
    (effective_subtraction_simd_i && (sum_carry_simd == tentative_sign_simd_i))
      ? 1'b1
      : (effective_subtraction_simd_i ? 1'b0 : tentative_sign_simd_i);


    // ------------
    // SIMD lane 2
    assign final_sign_fp8_1 =
    (effective_subtraction_fp8_1_i && (sum_carry_fp8_1 == tentative_sign_fp8_1_i))
      ? 1'b1
      : (effective_subtraction_fp8_1_i ? 1'b0 : tentative_sign_fp8_1_i);

    // ------------
    // SIMD lane 3
    assign final_sign_fp8_2 =
    (effective_subtraction_fp8_2_i && (sum_carry_fp8_2 == tentative_sign_fp8_2_i))
      ? 1'b1
      : (effective_subtraction_fp8_2_i ? 1'b0 : tentative_sign_fp8_2_i);

  // --------------------------------------------------------------------------
  // Output assignment
  // --------------------------------------------------------------------------
  assign sticky_before_add_o  = sticky_before_add;
  assign sum_o                = sum;
  assign final_sign_o         = final_sign;

  assign sticky_before_add_simd_o  = sticky_before_add_simd;
  assign sum_simd_o                = sum_simd;
  assign final_sign_simd_o         = final_sign_simd;

  assign sticky_before_add_fp8_1_o  = sticky_before_add_fp8_1;
  assign sum_fp8_1_o                = sum_fp8_1;
  assign final_sign_fp8_1_o         = final_sign_fp8_1;

  assign sticky_before_add_fp8_2_o  = sticky_before_add_fp8_2;
  assign sum_fp8_2_o                = sum_fp8_2;
  assign final_sign_fp8_2_o         = final_sign_fp8_2;

  // --------------------------------------------------------------------------
  // SafeDot residue shadow for the addend datapath (stage-1 core;
  // docs/SafeDot.md sec. 5.2/5.3). Modulus m = 2^SAFEDOT_K - 1 (mod 3
  // default). All packing/segment facts below were pinned empirically
  // against this RTL (one-hot probe + FMA-regression survey) — including
  // the fp8-mode 102->100 packing clip of mantissa_c[23:22] and the
  // foreign bits parked in neighbor quarters; the shadow mirrors the RTL
  // exactly (the bit-exact C++ golden defines these as the architecture).
  //
  // Timing contract (as instantiated by the FMA): mantissa/shamt arrive at
  // cycle T (consumed by the shifter, registered in addend_shift_full);
  // product_shifted / effective_subtraction / tentative_sign arrive at
  // T+1 and combine with the registered shift result; outputs are
  // combinational at T+1. Mode signals must be stable across T/T+1 (the
  // main datapath itself relies on this). The shadow registers T-side
  // payload residues and compares everything at T+1; the alarm is
  // registered (T+2).
  //
  // Checks: [0] shifter-register segments vs payload-residue rotations
  // (covers the 100-bit register including sticky bits — nothing is ever
  // discarded inside a segment for architectural shamt ranges);
  // [1] both w4 adders at merge-lane granularity (positive always, negative
  // when selected), with kept-slice residues derived from segment/sticky/
  // remainder extractions and complements handled as constant-minus-residue
  // over each lane's merge window; [2] sticky-flag OR replicas;
  // [3] final-sign mirrors.
  // --------------------------------------------------------------------------
`ifdef SAFEDOT_CHECK_EN
`ifndef SAFEDOT_K
`define SAFEDOT_K 2
`endif
  generate if (SAFEDOT_CHECK) begin : g_safedot
    localparam int unsigned SD_K = `SAFEDOT_K;
    `include "safedot_res.svh"
    localparam int unsigned MW = $clog2(SD_K > 1 ? SD_K : 2);
    // Complement windows per merge lane (value = 2^W - 1 - x):
    // (2^(W mod K) - 1) < m always, so no folding needed.
    localparam logic [SD_K-1:0] SD_C1     = SD_K'(1);
    localparam logic [SD_K-1:0] SD_C16M1  = SD_K'((1 << (16 % SD_K)) - 1);
    localparam logic [SD_K-1:0] SD_C18M1  = SD_K'((1 << (18 % SD_K)) - 1);
    localparam logic [SD_K-1:0] SD_C38M1  = SD_K'((1 << (38 % SD_K)) - 1);
    localparam logic [SD_K-1:0] SD_C76M1  = SD_K'((1 << (76 % SD_K)) - 1);
    localparam logic [SD_K-1:0] SD_C2P76  = SD_K'(1 << (76 % SD_K));

    // ---- launch cycle (T): payload residues + shamt mod K + modes.
    sd_res_t sd_r_mc24, sd_r_mch, sd_r_mcf8, sd_r_mcs, sd_r_mcsf8;
    sd_res_t sd_r_mc1, sd_r_mc2;
    safedot_mod3_reduce #(.W(24), .K(SD_K)) u_sd_mc24 (.x_i(mantissa_c_i),        .r_o(sd_r_mc24));
    safedot_mod3_reduce #(.W(11), .K(SD_K)) u_sd_mch  (.x_i(mantissa_c_i[23:13]), .r_o(sd_r_mch));
    safedot_mod3_reduce #(.W(4),  .K(SD_K)) u_sd_mcf8 (.x_i(mantissa_c_i[23:20]), .r_o(sd_r_mcf8));
    safedot_mod3_reduce #(.W(11), .K(SD_K)) u_sd_mcs  (.x_i(mantissa_c_simd_i),   .r_o(sd_r_mcs));
    safedot_mod3_reduce #(.W(4),  .K(SD_K)) u_sd_mcsf8(.x_i(mantissa_c_simd_i[10:7]), .r_o(sd_r_mcsf8));
    safedot_mod3_reduce #(.W(4),  .K(SD_K)) u_sd_mc1  (.x_i(mantissa_c_fp8_1_i[3:0]), .r_o(sd_r_mc1));
    safedot_mod3_reduce #(.W(4),  .K(SD_K)) u_sd_mc2  (.x_i(mantissa_c_fp8_2_i[3:0]), .r_o(sd_r_mc2));

    sd_res_t sd_r_mc24_q, sd_r_mch_q, sd_r_mcf8_q, sd_r_mcs_q, sd_r_mcsf8_q;
    sd_res_t sd_r_mc1_q, sd_r_mc2_q;
    // Alarm is qualified to the first cycle after a register load: that is
    // the cycle the FMA pairs this op's shifted addend with its product and
    // subtraction controls (holds and bubbles are not architecturally
    // consumed and would pair stale values).
    logic sd_vld_q;
    // Geometric in-range flags: a shamt beyond the payload's segment-bottom
    // offset (76/39/21) discards payload bits, which the rotation identity
    // does not model. Architectural shamts (<= 76/37/16) never exceed them;
    // out-of-contract shamts leave layer A ungated only where safe.
    logic sd_shok_sc_q, sd_shok_h_q, sd_shok_l_q;
    logic sd_shok3_q, sd_shok2_q, sd_shok1_q, sd_shok0_q;
    logic [MW-1:0] sd_sh_mk_q, sd_shs_mk_q, sd_sh1_mk_q, sd_sh2_mk_q;
    logic sd_simd_q, sd_fp8_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        sd_r_mc24_q  <= '0;
        sd_r_mch_q   <= '0;
        sd_r_mcf8_q  <= '0;
        sd_r_mcs_q   <= '0;
        sd_r_mcsf8_q <= '0;
        sd_r_mc1_q   <= '0;
        sd_r_mc2_q   <= '0;
        sd_vld_q     <= 1'b0;
        sd_shok_sc_q <= 1'b0;
        sd_shok_h_q  <= 1'b0;
        sd_shok_l_q  <= 1'b0;
        sd_shok3_q   <= 1'b0;
        sd_shok2_q   <= 1'b0;
        sd_shok1_q   <= 1'b0;
        sd_shok0_q   <= 1'b0;
        sd_sh_mk_q   <= '0;
        sd_shs_mk_q  <= '0;
        sd_sh1_mk_q  <= '0;
        sd_sh2_mk_q  <= '0;
        sd_simd_q    <= 1'b0;
        sd_fp8_q     <= 1'b0;
      end else begin
        // single top-level if per Presto async-reset FF policy (ELAB-302)
        sd_vld_q <= pipe_en;
        if (pipe_en) begin
          sd_r_mc24_q  <= sd_r_mc24;
        sd_r_mch_q   <= sd_r_mch;
        sd_r_mcf8_q  <= sd_r_mcf8;
        sd_r_mcs_q   <= sd_r_mcs;
        sd_r_mcsf8_q <= sd_r_mcsf8;
        sd_r_mc1_q   <= sd_r_mc1;
        sd_r_mc2_q   <= sd_r_mc2;
        sd_shok_sc_q <= (addend_shamt_i <= 76);
        sd_shok_h_q  <= (addend_shamt_i <= 39);
        sd_shok_l_q  <= (addend_shamt_simd_i <= 39);
        sd_shok3_q   <= (addend_shamt_i <= 21);
        sd_shok2_q   <= (addend_shamt_fp8_1_i <= 21);
        sd_shok1_q   <= (addend_shamt_simd_i <= 21);
        sd_shok0_q   <= (addend_shamt_fp8_2_i <= 21);
        sd_sh_mk_q   <= MW'(addend_shamt_i % SD_K);
        sd_shs_mk_q  <= MW'(addend_shamt_simd_i % SD_K);
        sd_sh1_mk_q  <= MW'(addend_shamt_fp8_1_i % SD_K);
        sd_sh2_mk_q  <= MW'(addend_shamt_fp8_2_i % SD_K);
        sd_simd_q    <= simd_enable_i;
        sd_fp8_q     <= is_fp8;
        end
      end
    end

    // ---- compare cycle (T+1), layer A: register segment checks.
    // Quarter residues of the registered shifter output (canonical: fp8
    // mode compares them directly).
    sd_res_t sd_rq0, sd_rq1, sd_rq2, sd_rq3;
    safedot_mod3_reduce #(.W(25), .K(SD_K)) u_sd_q0 (.x_i(addend_shift_full[24:0]),  .r_o(sd_rq0));
    safedot_mod3_reduce #(.W(25), .K(SD_K)) u_sd_q1 (.x_i(addend_shift_full[49:25]), .r_o(sd_rq1));
    safedot_mod3_reduce #(.W(25), .K(SD_K)) u_sd_q2 (.x_i(addend_shift_full[74:50]), .r_o(sd_rq2));
    safedot_mod3_reduce #(.W(25), .K(SD_K)) u_sd_q3 (.x_i(addend_shift_full[99:75]), .r_o(sd_rq3));
    sd_res_t sd_segl, sd_segh, sd_seg100;
    assign sd_segl   = sd_add(sd_rq0, sd_rols(sd_rq1, 25));
    assign sd_segh   = sd_add(sd_rq2, sd_rols(sd_rq3, 25));
    assign sd_seg100 = sd_add(sd_segl, sd_rols(sd_segh, 50));

    sd_res_t sd_pred100, sd_predh, sd_predl;
    sd_res_t sd_predq3, sd_predq2, sd_predq1, sd_predq0;
    assign sd_pred100 = sd_rord(sd_rols(sd_r_mc24_q, 76), sd_sh_mk_q);
    assign sd_predh   = sd_rord(sd_rols(sd_r_mch_q, 39), sd_sh_mk_q);
    assign sd_predl   = sd_rord(sd_rols(sd_r_mcs_q, 39), sd_shs_mk_q);
    // FP8 quarters (FMA config, PRECISION_BITS_FP8 == 4): each quarter
    // holds one 4-bit payload at segment-relative [24:21]; architectural
    // shamt <= 16 keeps every payload bit inside its quarter.
    assign sd_predq3  = sd_rord(sd_rols(sd_r_mcf8_q, 21), sd_sh_mk_q);
    assign sd_predq2  = sd_rord(sd_rols(sd_r_mc1_q, 21), sd_sh1_mk_q);
    assign sd_predq1  = sd_rord(sd_rols(sd_r_mcsf8_q, 21), sd_shs_mk_q);
    assign sd_predq0  = sd_rord(sd_rols(sd_r_mc2_q, 21), sd_sh2_mk_q);

    logic sd_mis_a;
    assign sd_mis_a = sd_simd_q
        ? (sd_fp8_q ? ((sd_shok3_q && (sd_rq3 != sd_predq3)) ||
                       (sd_shok2_q && (sd_rq2 != sd_predq2)) ||
                       (sd_shok1_q && (sd_rq1 != sd_predq1)) ||
                       (sd_shok0_q && (sd_rq0 != sd_predq0)))
                    : ((sd_shok_h_q && (sd_segh != sd_predh)) ||
                       (sd_shok_l_q && (sd_segl != sd_predl))))
        : (sd_shok_sc_q && (sd_seg100 != sd_pred100));

    // ---- layer B prerequisites: sticky/remainder extractions and derived
    // kept-slice residues (mode-fixed register slices; digits are fine,
    // sd_sub canonicalizes).
    sd_res_t sd_stk24, sd_stkh, sd_stkl, sd_remh, sd_reml;
    sd_res_t sd_stk8 [0:3];
    sd_res_t sd_rem8 [0:3];
    safedot_mod3_reduce #(.W(24), .K(SD_K), .CANON(1'b0)) u_sd_stk24 (.x_i(addend_shift_full[23:0]),  .r_o(sd_stk24));
    safedot_mod3_reduce #(.W(11), .K(SD_K), .CANON(1'b0)) u_sd_stkh  (.x_i(addend_shift_full[62:52]), .r_o(sd_stkh));
    safedot_mod3_reduce #(.W(11), .K(SD_K), .CANON(1'b0)) u_sd_stkl  (.x_i(addend_shift_full[12:2]),  .r_o(sd_stkl));
    safedot_mod3_reduce #(.W(2),  .K(SD_K), .CANON(1'b0)) u_sd_remh  (.x_i(addend_shift_full[51:50]), .r_o(sd_remh));
    safedot_mod3_reduce #(.W(2),  .K(SD_K), .CANON(1'b0)) u_sd_reml  (.x_i(addend_shift_full[1:0]),   .r_o(sd_reml));
    safedot_mod3_reduce #(.W(4),  .K(SD_K), .CANON(1'b0)) u_sd_stk8_0 (.x_i(addend_shift_full[83:80]), .r_o(sd_stk8[0]));
    safedot_mod3_reduce #(.W(4),  .K(SD_K), .CANON(1'b0)) u_sd_stk8_1 (.x_i(addend_shift_full[58:55]), .r_o(sd_stk8[1]));
    safedot_mod3_reduce #(.W(4),  .K(SD_K), .CANON(1'b0)) u_sd_stk8_2 (.x_i(addend_shift_full[33:30]), .r_o(sd_stk8[2]));
    safedot_mod3_reduce #(.W(4),  .K(SD_K), .CANON(1'b0)) u_sd_stk8_3 (.x_i(addend_shift_full[8:5]),   .r_o(sd_stk8[3]));
    safedot_mod3_reduce #(.W(5),  .K(SD_K), .CANON(1'b0)) u_sd_rem8_0 (.x_i(addend_shift_full[79:75]), .r_o(sd_rem8[0]));
    safedot_mod3_reduce #(.W(5),  .K(SD_K), .CANON(1'b0)) u_sd_rem8_1 (.x_i(addend_shift_full[54:50]), .r_o(sd_rem8[1]));
    safedot_mod3_reduce #(.W(5),  .K(SD_K), .CANON(1'b0)) u_sd_rem8_2 (.x_i(addend_shift_full[29:25]), .r_o(sd_rem8[2]));
    safedot_mod3_reduce #(.W(5),  .K(SD_K), .CANON(1'b0)) u_sd_rem8_3 (.x_i(addend_shift_full[4:0]),   .r_o(sd_rem8[3]));

    // Kept-slice residues (the values the adders actually consume):
    //   scalar: seg100 = kept76 * 2^24 + stk24
    //   fp16:   seg50  = kept37 * 2^13 + stk11 * 2^2 + rem2
    //   fp8:    seg25  = kept   * 2^(9|10) + stk * 2^5 + rem5
    sd_res_t sd_kept_sc, sd_kept16h, sd_kept16l;
    sd_res_t sd_kept8 [0:3];
    assign sd_kept_sc = sd_rols(sd_sub(sd_seg100, sd_stk24), SD_K - (24 % SD_K));
    assign sd_kept16h = sd_rols(sd_sub(sd_sub(sd_segh, sd_rols(sd_stkh, 2)), sd_remh),
                                SD_K - (13 % SD_K));
    assign sd_kept16l = sd_rols(sd_sub(sd_sub(sd_segl, sd_rols(sd_stkl, 2)), sd_reml),
                                SD_K - (13 % SD_K));
    assign sd_kept8[0] = sd_rols(sd_sub(sd_sub(sd_rq3, sd_rols(sd_stk8[0], 5)), sd_rem8[0]),
                                 SD_K - (9 % SD_K));
    assign sd_kept8[1] = sd_rols(sd_sub(sd_sub(sd_rq2, sd_rols(sd_stk8[1], 5)), sd_rem8[1]),
                                 SD_K - (9 % SD_K));
    assign sd_kept8[2] = sd_rols(sd_sub(sd_sub(sd_rq1, sd_rols(sd_stk8[2], 5)), sd_rem8[2]),
                                 SD_K - (9 % SD_K));
    assign sd_kept8[3] = sd_rols(sd_sub(sd_sub(sd_rq0, sd_rols(sd_stk8[3], 5)), sd_rem8[3]),
                                 SD_K - (9 % SD_K));

    // ---- layer C: sticky-flag OR replicas and final-sign mirrors (live
    // modes, mirroring the main path's own mode usage).
    logic sd_st_sc, sd_st_simd, sd_st_1, sd_st_2;
    assign sd_st_sc   = simd_enable_i ? (is_fp8 ? (|addend_shift_full[83:80])
                                                : (|addend_shift_full[62:52]))
                                      : (|addend_shift_full[23:0]);
    assign sd_st_simd = is_fp8 ? (|addend_shift_full[33:30]) : (|addend_shift_full[12:2]);
    assign sd_st_1    = |addend_shift_full[58:55];
    assign sd_st_2    = |addend_shift_full[8:5];
    logic sd_mis_c;
    assign sd_mis_c = (sd_st_sc   != sticky_before_add_o)
                   || (sd_st_simd != sticky_before_add_simd_o)
                   || (sd_st_1    != sticky_before_add_fp8_1_o)
                   || (sd_st_2    != sticky_before_add_fp8_2_o);

    logic sd_fs_sc, sd_fs_simd, sd_fs_1, sd_fs_2;
    assign sd_fs_sc   = effective_subtraction_i ? ((sum_carry == tentative_sign_i) ? 1'b1 : 1'b0)
                                                : tentative_sign_i;
    assign sd_fs_simd = effective_subtraction_simd_i ? ((sum_carry_simd == tentative_sign_simd_i) ? 1'b1 : 1'b0)
                                                     : tentative_sign_simd_i;
    assign sd_fs_1    = effective_subtraction_fp8_1_i ? ((sum_carry_fp8_1 == tentative_sign_fp8_1_i) ? 1'b1 : 1'b0)
                                                      : tentative_sign_fp8_1_i;
    assign sd_fs_2    = effective_subtraction_fp8_2_i ? ((sum_carry_fp8_2 == tentative_sign_fp8_2_i) ? 1'b1 : 1'b0)
                                                      : tentative_sign_fp8_2_i;
    logic sd_mis_d;
    assign sd_mis_d = (sd_fs_sc != final_sign_o) || (sd_fs_simd != final_sign_simd_o)
                   || (sd_fs_1 != final_sign_fp8_1_o) || (sd_fs_2 != final_sign_fp8_2_o);

    // ---- layer B: both w4 adders at merge-lane granularity.
    // Quarter residues of the operand and result merge vectors (19-bit
    // lanes; each lane's carry bit sits inside its own quarter slice, and
    // the pad bits above it are structurally zero).
    sd_res_t sd_rp [0:3];
    sd_res_t sd_rsp [0:3];
    sd_res_t sd_rsn [0:3];
    safedot_mod3_reduce #(.W(19), .K(SD_K), .CANON(1'b0)) u_sd_rp0 (.x_i(product_shifted_merge[18:0]),  .r_o(sd_rp[0]));
    safedot_mod3_reduce #(.W(19), .K(SD_K), .CANON(1'b0)) u_sd_rp1 (.x_i(product_shifted_merge[37:19]), .r_o(sd_rp[1]));
    safedot_mod3_reduce #(.W(19), .K(SD_K), .CANON(1'b0)) u_sd_rp2 (.x_i(product_shifted_merge[56:38]), .r_o(sd_rp[2]));
    safedot_mod3_reduce #(.W(19), .K(SD_K), .CANON(1'b0)) u_sd_rp3 (.x_i(product_shifted_merge[75:57]), .r_o(sd_rp[3]));
    safedot_mod3_reduce #(.W(19), .K(SD_K)) u_sd_sp0 (.x_i(sum_pos_merge[18:0]),  .r_o(sd_rsp[0]));
    safedot_mod3_reduce #(.W(19), .K(SD_K)) u_sd_sp1 (.x_i(sum_pos_merge[37:19]), .r_o(sd_rsp[1]));
    safedot_mod3_reduce #(.W(19), .K(SD_K)) u_sd_sp2 (.x_i(sum_pos_merge[56:38]), .r_o(sd_rsp[2]));
    safedot_mod3_reduce #(.W(19), .K(SD_K)) u_sd_sp3 (.x_i(sum_pos_merge[75:57]), .r_o(sd_rsp[3]));
    safedot_mod3_reduce #(.W(19), .K(SD_K)) u_sd_sn0 (.x_i(sum_neg_merge[18:0]),  .r_o(sd_rsn[0]));
    safedot_mod3_reduce #(.W(19), .K(SD_K)) u_sd_sn1 (.x_i(sum_neg_merge[37:19]), .r_o(sd_rsn[1]));
    safedot_mod3_reduce #(.W(19), .K(SD_K)) u_sd_sn2 (.x_i(sum_neg_merge[56:38]), .r_o(sd_rsn[2]));
    safedot_mod3_reduce #(.W(19), .K(SD_K)) u_sd_sn3 (.x_i(sum_neg_merge[75:57]), .r_o(sd_rsn[3]));

    // Replica carry-inject terms use the replica sticky flags, so a fault
    // in the main path's sticky OR shows up here as well.
    logic sd_cin_sc, sd_cin_simd, sd_cin_1, sd_cin_2;
    assign sd_cin_sc   = effective_subtraction_i        & ~sd_st_sc;
    assign sd_cin_simd = effective_subtraction_simd_i   & ~sd_st_simd;
    assign sd_cin_1    = effective_subtraction_fp8_1_i  & ~sd_st_1;
    assign sd_cin_2    = effective_subtraction_fp8_2_i  & ~sd_st_2;

    // Scalar mode: one 77-bit lane (carry at bit 76).
    sd_res_t sd_pos_lhs_sc, sd_pos_rhs_sc, sd_neg_lhs_sc, sd_neg_rhs_sc;
    sd_res_t sd_rp_all;
    assign sd_rp_all = sd_add(sd_add(sd_rp[0], sd_rols(sd_rp[1], 19)),
                              sd_add(sd_rols(sd_rp[2], 38), sd_rols(sd_rp[3], 57)));
    assign sd_pos_lhs_sc = sd_add(sd_add(sd_add(sd_rsp[0], sd_rols(sd_rsp[1], 19)),
                                         sd_add(sd_rols(sd_rsp[2], 38), sd_rols(sd_rsp[3], 57))),
                                  sum_pos_merge[76] ? SD_C2P76 : {SD_K{1'b0}});
    assign sd_pos_rhs_sc = sd_add(sd_add(sd_rp_all,
                                         effective_subtraction_i ? sd_sub(SD_C76M1, sd_kept_sc)
                                                                 : sd_canon(sd_kept_sc)),
                                  sd_cin_sc ? SD_C1 : {SD_K{1'b0}});
    assign sd_neg_lhs_sc = sd_add(sd_add(sd_add(sd_rsn[0], sd_rols(sd_rsn[1], 19)),
                                         sd_add(sd_rols(sd_rsn[2], 38), sd_rols(sd_rsn[3], 57))),
                                  sum_neg_merge[76] ? SD_C2P76 : {SD_K{1'b0}});
    assign sd_neg_rhs_sc = sd_add(sd_add(sd_canon(sd_kept_sc), sd_sub(SD_C76M1, sd_rp_all)), SD_C1);

    // FP16 mode: two 38-bit lanes (low = scalar-lane variables, high =
    // simd-lane variables; carries at merge bits 37 / 75 sit inside the
    // quarter slices).
    sd_res_t sd_pos_lhs_l, sd_pos_rhs_l, sd_neg_lhs_l, sd_neg_rhs_l;
    sd_res_t sd_pos_lhs_h, sd_pos_rhs_h, sd_neg_lhs_h, sd_neg_rhs_h;
    sd_res_t sd_rp_l, sd_rp_h;
    assign sd_rp_l = sd_add(sd_rp[0], sd_rols(sd_rp[1], 19));
    assign sd_rp_h = sd_add(sd_rp[2], sd_rols(sd_rp[3], 19));
    assign sd_pos_lhs_l = sd_add(sd_rsp[0], sd_rols(sd_rsp[1], 19));
    assign sd_pos_rhs_l = sd_add(sd_add(sd_rp_l,
                                        effective_subtraction_i ? sd_sub(SD_C38M1, sd_kept16h)
                                                                : sd_canon(sd_kept16h)),
                                 sd_cin_sc ? SD_C1 : {SD_K{1'b0}});
    assign sd_neg_lhs_l = sd_add(sd_rsn[0], sd_rols(sd_rsn[1], 19));
    assign sd_neg_rhs_l = sd_add(sd_add(sd_canon(sd_kept16h), sd_sub(SD_C38M1, sd_rp_l)), SD_C1);
    assign sd_pos_lhs_h = sd_add(sd_rsp[2], sd_rols(sd_rsp[3], 19));
    assign sd_pos_rhs_h = sd_add(sd_add(sd_rp_h,
                                        effective_subtraction_simd_i ? sd_sub(SD_C38M1, sd_kept16l)
                                                                     : sd_canon(sd_kept16l)),
                                 sd_cin_simd ? SD_C1 : {SD_K{1'b0}});
    assign sd_neg_lhs_h = sd_add(sd_rsn[2], sd_rols(sd_rsn[3], 19));
    assign sd_neg_rhs_h = sd_add(sd_add(sd_canon(sd_kept16l), sd_sub(SD_C38M1, sd_rp_h)), SD_C1);

    // FP8 mode: four 17-bit lanes, one per quarter (adder lane order is
    // q0 = scalar, q1 = fp8_1, q2 = simd, q3 = fp8_2 — NOT the register
    // quarter order). Complement windows: 16 bits (positive adder operand),
    // 18 bits (negative adder, after the FP8_NEG_CLEAR_MASK fixup).
    sd_res_t sd_pos_lhs8 [0:3];
    sd_res_t sd_pos_rhs8 [0:3];
    sd_res_t sd_neg_lhs8 [0:3];
    sd_res_t sd_neg_rhs8 [0:3];
    // register-side kept indices per adder lane: scalar->kept8[0](q3),
    // fp8_1->kept8[1](q2), simd->kept8[2](q1), fp8_2->kept8[3](q0).
    logic sd_es8 [0:3];
    logic sd_cin8 [0:3];
    assign sd_es8[0]  = effective_subtraction_i;
    assign sd_es8[1]  = effective_subtraction_fp8_1_i;
    assign sd_es8[2]  = effective_subtraction_simd_i;
    assign sd_es8[3]  = effective_subtraction_fp8_2_i;
    assign sd_cin8[0] = sd_cin_sc;
    assign sd_cin8[1] = sd_cin_1;
    assign sd_cin8[2] = sd_cin_simd;
    assign sd_cin8[3] = sd_cin_2;
    for (genvar gl = 0; gl < 4; gl++) begin : g_sd_fp8lane
      assign sd_pos_lhs8[gl] = sd_canon(sd_rsp[gl]);
      assign sd_pos_rhs8[gl] = sd_add(sd_add(sd_canon(sd_rp[gl]),
                                             sd_es8[gl] ? sd_sub(SD_C16M1, sd_kept8[gl])
                                                        : sd_canon(sd_kept8[gl])),
                                      sd_cin8[gl] ? SD_C1 : {SD_K{1'b0}});
      assign sd_neg_lhs8[gl] = sd_canon(sd_rsn[gl]);
      assign sd_neg_rhs8[gl] = sd_add(sd_add(sd_canon(sd_kept8[gl]),
                                             sd_sub(SD_C18M1, sd_rp[gl])), SD_C1);
    end

    // Negative-path checks apply only when that lane's negative sum is the
    // selected result (mirrors the main path's select condition).
    logic sd_mis_b;
    logic sd_negsel_sc, sd_negsel_simd, sd_negsel_1, sd_negsel_2;
    assign sd_negsel_sc   = effective_subtraction_i       && ~sum_carry;
    assign sd_negsel_simd = effective_subtraction_simd_i  && ~sum_carry_simd;
    assign sd_negsel_1    = effective_subtraction_fp8_1_i && ~sum_carry_fp8_1;
    assign sd_negsel_2    = effective_subtraction_fp8_2_i && ~sum_carry_fp8_2;
    always_comb begin
      if (simd_enable_i && is_fp8) begin
        sd_mis_b = (sd_pos_lhs8[0] != sd_pos_rhs8[0]) || (sd_pos_lhs8[1] != sd_pos_rhs8[1])
                || (sd_pos_lhs8[2] != sd_pos_rhs8[2]) || (sd_pos_lhs8[3] != sd_pos_rhs8[3])
                || (sd_negsel_sc   && (sd_neg_lhs8[0] != sd_neg_rhs8[0]))
                || (sd_negsel_1    && (sd_neg_lhs8[1] != sd_neg_rhs8[1]))
                || (sd_negsel_simd && (sd_neg_lhs8[2] != sd_neg_rhs8[2]))
                || (sd_negsel_2    && (sd_neg_lhs8[3] != sd_neg_rhs8[3]));
      end else if (simd_enable_i) begin
        sd_mis_b = (sd_pos_lhs_l != sd_pos_rhs_l) || (sd_pos_lhs_h != sd_pos_rhs_h)
                || (sd_negsel_sc   && (sd_neg_lhs_l != sd_neg_rhs_l))
                || (sd_negsel_simd && (sd_neg_lhs_h != sd_neg_rhs_h));
      end else begin
        sd_mis_b = (sd_pos_lhs_sc != sd_pos_rhs_sc)
                || (sd_negsel_sc && (sd_neg_lhs_sc != sd_neg_rhs_sc));
      end
    end

    // ---- alarm register (T+2).
    // The compare is architecturally settled in cycles where the pipeline
    // advances (pipe_en): that is when the downstream stage captures this
    // module's outputs, and the FMA guarantees the cycle-2 inputs (product,
    // subtraction controls) belong to the op whose shifted addend is in the
    // register. sd_vld_q additionally requires that the register holds a
    // launched op (kills reset/bubble garbage).
    logic [3:0] sd_alarm_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) sd_alarm_q <= '0;
      else         sd_alarm_q <= {4{sd_vld_q && pipe_en}}
                              & {sd_mis_d, sd_mis_c, sd_mis_b, sd_mis_a};
    end
    assign safedot_alarm_o = sd_alarm_q;
  end else begin : g_safedot_off
    assign safedot_alarm_o = '0;
  end endgenerate
`else
  assign safedot_alarm_o = '0;
`endif
endmodule


module transdot_decomp_addend_datapath_piped_packed_product #(
  parameter int unsigned SUPER_MAN_BITS     = 23,
  parameter int unsigned PRECISION_BITS     = SUPER_MAN_BITS + 1,
  parameter int unsigned SHIFT_AMOUNT_WIDTH = $clog2(3 * PRECISION_BITS + 5),
  parameter int unsigned SUPER_MAN_BITS_SIMD     = 10,
  parameter int unsigned PRECISION_BITS_SIMD     = SUPER_MAN_BITS_SIMD + 1,
  parameter int unsigned SHIFT_AMOUNT_WIDTH_SIMD = $clog2(3 * PRECISION_BITS_SIMD + 5),
  parameter int unsigned SUPER_MAN_BITS_FP8     = 4,
  parameter int unsigned PRECISION_BITS_FP8     = SUPER_MAN_BITS_FP8 + 1,
  parameter int unsigned SHIFT_AMOUNT_WIDTH_FP8 = $clog2(3 * PRECISION_BITS_FP8 + 5)
)(
  input  logic                               clk_i,
  input  logic                               pipe_en,

  input  logic [PRECISION_BITS-1:0]          mantissa_c_i,
  input  logic [3*PRECISION_BITS+3:0]        product_shifted_i,
  input  logic [2*PRECISION_BITS-1:0]        product_comb_i,
  input  logic [SHIFT_AMOUNT_WIDTH-1:0]      addend_shamt_i,
  input  logic                               effective_subtraction_i,
  input  logic                               tentative_sign_i,

  output logic                               sticky_before_add_o,
  output logic [3*PRECISION_BITS+3:0]        sum_o,
  output logic                               final_sign_o,

  input  logic                               simd_enable_i,
  input  logic                               is_fp8,

  input  logic [PRECISION_BITS_SIMD-1:0]     mantissa_c_simd_i,
  input  logic [SHIFT_AMOUNT_WIDTH_SIMD-1:0] addend_shamt_simd_i,
  input  logic                               effective_subtraction_simd_i,
  input  logic                               tentative_sign_simd_i,
  output logic                               sticky_before_add_simd_o,
  output logic [3*PRECISION_BITS_SIMD+3:0]   sum_simd_o,
  output logic                               final_sign_simd_o,

  input  logic [PRECISION_BITS_FP8-1:0]      mantissa_c_fp8_1_i,
  input  logic [SHIFT_AMOUNT_WIDTH_FP8-1:0]  addend_shamt_fp8_1_i,
  input  logic                               effective_subtraction_fp8_1_i,
  input  logic                               tentative_sign_fp8_1_i,
  output logic                               sticky_before_add_fp8_1_o,
  output logic [3*PRECISION_BITS_FP8+3:0]    sum_fp8_1_o,
  output logic                               final_sign_fp8_1_o,

  input  logic [PRECISION_BITS_FP8-1:0]      mantissa_c_fp8_2_i,
  input  logic [SHIFT_AMOUNT_WIDTH_FP8-1:0]  addend_shamt_fp8_2_i,
  input  logic                               effective_subtraction_fp8_2_i,
  input  logic                               tentative_sign_fp8_2_i,
  output logic                               sticky_before_add_fp8_2_o,
  output logic [3*PRECISION_BITS_FP8+3:0]    sum_fp8_2_o,
  output logic                               final_sign_fp8_2_o
);

  logic [3*PRECISION_BITS_SIMD+3:0] product_shifted_simd_from_packed;
  logic [3*PRECISION_BITS_FP8+3:0]  product_shifted_fp8_1_from_packed;
  logic [3*PRECISION_BITS_FP8+3:0]  product_shifted_fp8_2_from_packed;

  // Reconstruct the non-DP SIMD/FP8 lane products locally from the packed
  // multiplier output so the top level no longer has to fan out three extra
  // shifted-product buses into this addend merge stage.
  assign product_shifted_simd_from_packed = is_fp8
      ? {{(3*PRECISION_BITS_SIMD+4-25){1'b0}}, 1'b0, product_comb_i[19:12], 16'd0}
      : {{(3*PRECISION_BITS_SIMD+4-25){1'b0}}, 1'b0,
         product_comb_i[2*PRECISION_BITS-3-:2*PRECISION_BITS_SIMD], 2'b00};
  assign product_shifted_fp8_1_from_packed =
      {{(3*PRECISION_BITS_FP8+4-11){1'b0}}, 1'b0, product_comb_i[31:24], 2'b00};
  assign product_shifted_fp8_2_from_packed =
      {{(3*PRECISION_BITS_FP8+4-11){1'b0}}, 1'b0, product_comb_i[43:36], 2'b00};

  transdot_decomp_addend_datapath_piped #(
    .SUPER_MAN_BITS         ( SUPER_MAN_BITS ),
    .PRECISION_BITS         ( PRECISION_BITS ),
    .SHIFT_AMOUNT_WIDTH     ( SHIFT_AMOUNT_WIDTH ),
    .SUPER_MAN_BITS_SIMD    ( SUPER_MAN_BITS_SIMD ),
    .PRECISION_BITS_SIMD    ( PRECISION_BITS_SIMD ),
    .SHIFT_AMOUNT_WIDTH_SIMD( SHIFT_AMOUNT_WIDTH_SIMD ),
    .SUPER_MAN_BITS_FP8     ( SUPER_MAN_BITS_FP8 ),
    .PRECISION_BITS_FP8     ( PRECISION_BITS_FP8 ),
    .SHIFT_AMOUNT_WIDTH_FP8 ( SHIFT_AMOUNT_WIDTH_FP8 )
  ) i_decomp_addend_datapath_packed (
    .clk_i                   ( clk_i ),
    .pipe_en                 ( pipe_en ),
    .mantissa_c_i            ( mantissa_c_i ),
    .product_shifted_i       ( product_shifted_i ),
    .addend_shamt_i          ( addend_shamt_i ),
    .effective_subtraction_i ( effective_subtraction_i ),
    .tentative_sign_i        ( tentative_sign_i ),
    .sticky_before_add_o     ( sticky_before_add_o ),
    .sum_o                   ( sum_o ),
    .final_sign_o            ( final_sign_o ),
    .simd_enable_i           ( simd_enable_i ),
    .is_fp8                  ( is_fp8 ),
    .mantissa_c_simd_i            ( mantissa_c_simd_i ),
    .product_shifted_simd_i       ( product_shifted_simd_from_packed ),
    .addend_shamt_simd_i          ( addend_shamt_simd_i ),
    .effective_subtraction_simd_i ( effective_subtraction_simd_i ),
    .tentative_sign_simd_i        ( tentative_sign_simd_i ),
    .sticky_before_add_simd_o     ( sticky_before_add_simd_o ),
    .sum_simd_o                   ( sum_simd_o ),
    .final_sign_simd_o            ( final_sign_simd_o ),
    .mantissa_c_fp8_1_i            ( mantissa_c_fp8_1_i ),
    .product_shifted_fp8_1_i       ( product_shifted_fp8_1_from_packed ),
    .addend_shamt_fp8_1_i          ( addend_shamt_fp8_1_i ),
    .effective_subtraction_fp8_1_i ( effective_subtraction_fp8_1_i ),
    .tentative_sign_fp8_1_i        ( tentative_sign_fp8_1_i ),
    .sticky_before_add_fp8_1_o     ( sticky_before_add_fp8_1_o ),
    .sum_fp8_1_o                   ( sum_fp8_1_o ),
    .final_sign_fp8_1_o            ( final_sign_fp8_1_o ),
    .mantissa_c_fp8_2_i            ( mantissa_c_fp8_2_i ),
    .product_shifted_fp8_2_i       ( product_shifted_fp8_2_from_packed ),
    .addend_shamt_fp8_2_i          ( addend_shamt_fp8_2_i ),
    .effective_subtraction_fp8_2_i ( effective_subtraction_fp8_2_i ),
    .tentative_sign_fp8_2_i        ( tentative_sign_fp8_2_i ),
    .sticky_before_add_fp8_2_o     ( sticky_before_add_fp8_2_o ),
    .sum_fp8_2_o                   ( sum_fp8_2_o ),
    .final_sign_fp8_2_o            ( final_sign_fp8_2_o ),
    .safedot_alarm_o               ( /* unused in this variant */ )
  );
endmodule

module transdot_decomp_addend_datapath_piped_packed_product_dp #(
  parameter int unsigned SUPER_MAN_BITS     = 23,
  parameter int unsigned PRECISION_BITS     = SUPER_MAN_BITS + 1,
  parameter int unsigned SHIFT_AMOUNT_WIDTH = $clog2(3 * PRECISION_BITS + 5),
  parameter int unsigned SUPER_MAN_BITS_SIMD     = 10,
  parameter int unsigned PRECISION_BITS_SIMD     = SUPER_MAN_BITS_SIMD + 1,
  parameter int unsigned SHIFT_AMOUNT_WIDTH_SIMD = $clog2(3 * PRECISION_BITS_SIMD + 5),
  parameter int unsigned SUPER_MAN_BITS_FP8     = 4,
  parameter int unsigned PRECISION_BITS_FP8     = SUPER_MAN_BITS_FP8 + 1,
  parameter int unsigned SHIFT_AMOUNT_WIDTH_FP8 = $clog2(3 * PRECISION_BITS_FP8 + 5)
)(
  input  logic                               clk_i,
  input  logic                               pipe_en,
  input  logic                               dp_enable_i,
  input  logic                               fp4_enable_i,

  input  logic [PRECISION_BITS-1:0]          mantissa_c_i,
  input  logic [2*PRECISION_BITS-1:0]        product_comb_i,
  input  logic [2*PRECISION_BITS_SIMD+4:0]   product_shifted_dp_post_i,
  input  logic [47:0]                        product_shifted_dp_fp4_i,
  input  logic [SHIFT_AMOUNT_WIDTH-1:0]      addend_shamt_i,
  input  logic                               effective_subtraction_i,
  input  logic                               tentative_sign_i,

  output logic                               sticky_before_add_o,
  output logic [3*PRECISION_BITS+3:0]        sum_o,
  output logic                               final_sign_o,

  input  logic                               simd_enable_i,
  input  logic                               is_fp8,

  input  logic [PRECISION_BITS_SIMD-1:0]     mantissa_c_simd_i,
  input  logic [SHIFT_AMOUNT_WIDTH_SIMD-1:0] addend_shamt_simd_i,
  input  logic                               effective_subtraction_simd_i,
  input  logic                               tentative_sign_simd_i,
  output logic                               sticky_before_add_simd_o,
  output logic [3*PRECISION_BITS_SIMD+3:0]   sum_simd_o,
  output logic                               final_sign_simd_o,

  input  logic [PRECISION_BITS_FP8-1:0]      mantissa_c_fp8_1_i,
  input  logic [SHIFT_AMOUNT_WIDTH_FP8-1:0]  addend_shamt_fp8_1_i,
  input  logic                               effective_subtraction_fp8_1_i,
  input  logic                               tentative_sign_fp8_1_i,
  output logic                               sticky_before_add_fp8_1_o,
  output logic [3*PRECISION_BITS_FP8+3:0]    sum_fp8_1_o,
  output logic                               final_sign_fp8_1_o,

  input  logic [PRECISION_BITS_FP8-1:0]      mantissa_c_fp8_2_i,
  input  logic [SHIFT_AMOUNT_WIDTH_FP8-1:0]  addend_shamt_fp8_2_i,
  input  logic                               effective_subtraction_fp8_2_i,
  input  logic                               tentative_sign_fp8_2_i,
  output logic                               sticky_before_add_fp8_2_o,
  output logic [3*PRECISION_BITS_FP8+3:0]    sum_fp8_2_o,
  output logic                               final_sign_fp8_2_o
);

  logic [3*PRECISION_BITS+3:0]     product_shifted_selected;
  logic [3*PRECISION_BITS_SIMD+3:0] product_shifted_simd_from_packed;
  logic [3*PRECISION_BITS_FP8+3:0]  product_shifted_fp8_1_from_packed;
  logic [3*PRECISION_BITS_FP8+3:0]  product_shifted_fp8_2_from_packed;

  always_comb begin
    if (dp_enable_i) begin
      product_shifted_selected = fp4_enable_i
          ? {'0, product_shifted_dp_fp4_i, 4'd0}
          : {'0, product_shifted_dp_post_i, {(2*PRECISION_BITS-2*PRECISION_BITS_SIMD){1'b0}}};
    end else if (!simd_enable_i) begin
      product_shifted_selected = {'0, product_comb_i, 2'b00};
    end else if (is_fp8) begin
      product_shifted_selected = {
        {(3*PRECISION_BITS+4-(2*PRECISION_BITS+3)){1'b0}},
        1'b0,
        product_comb_i[2*PRECISION_BITS_FP8-1:0],
        {(2*PRECISION_BITS-2*PRECISION_BITS_FP8){1'b0}},
        2'b00
      };
    end else begin
      product_shifted_selected = {
        {(3*PRECISION_BITS+4-(2*PRECISION_BITS+3)){1'b0}},
        1'b0,
        product_comb_i[PRECISION_BITS-3-:2*PRECISION_BITS_SIMD],
        {(2*PRECISION_BITS-2*PRECISION_BITS_SIMD){1'b0}},
        2'b00
      };
    end
  end

  assign product_shifted_simd_from_packed = is_fp8
      ? {{(3*PRECISION_BITS_SIMD+4-25){1'b0}}, 1'b0, product_comb_i[19:12], 16'd0}
      : {{(3*PRECISION_BITS_SIMD+4-25){1'b0}}, 1'b0,
         product_comb_i[2*PRECISION_BITS-3-:2*PRECISION_BITS_SIMD], 2'b00};
  assign product_shifted_fp8_1_from_packed =
      {{(3*PRECISION_BITS_FP8+4-11){1'b0}}, 1'b0, product_comb_i[31:24], 2'b00};
  assign product_shifted_fp8_2_from_packed =
      {{(3*PRECISION_BITS_FP8+4-11){1'b0}}, 1'b0, product_comb_i[43:36], 2'b00};

  transdot_decomp_addend_datapath_piped #(
    .SUPER_MAN_BITS         ( SUPER_MAN_BITS ),
    .PRECISION_BITS         ( PRECISION_BITS ),
    .SHIFT_AMOUNT_WIDTH     ( SHIFT_AMOUNT_WIDTH ),
    .SUPER_MAN_BITS_SIMD    ( SUPER_MAN_BITS_SIMD ),
    .PRECISION_BITS_SIMD    ( PRECISION_BITS_SIMD ),
    .SHIFT_AMOUNT_WIDTH_SIMD( SHIFT_AMOUNT_WIDTH_SIMD ),
    .SUPER_MAN_BITS_FP8     ( SUPER_MAN_BITS_FP8 ),
    .PRECISION_BITS_FP8     ( PRECISION_BITS_FP8 ),
    .SHIFT_AMOUNT_WIDTH_FP8 ( SHIFT_AMOUNT_WIDTH_FP8 )
  ) i_decomp_addend_datapath_packed_dp (
    .clk_i                   ( clk_i ),
    .pipe_en                 ( pipe_en ),
    .mantissa_c_i            ( mantissa_c_i ),
    .product_shifted_i       ( product_shifted_selected ),
    .addend_shamt_i          ( addend_shamt_i ),
    .effective_subtraction_i ( effective_subtraction_i ),
    .tentative_sign_i        ( tentative_sign_i ),
    .sticky_before_add_o     ( sticky_before_add_o ),
    .sum_o                   ( sum_o ),
    .final_sign_o            ( final_sign_o ),
    .simd_enable_i           ( simd_enable_i ),
    .is_fp8                  ( is_fp8 ),
    .mantissa_c_simd_i            ( mantissa_c_simd_i ),
    .product_shifted_simd_i       ( product_shifted_simd_from_packed ),
    .addend_shamt_simd_i          ( addend_shamt_simd_i ),
    .effective_subtraction_simd_i ( effective_subtraction_simd_i ),
    .tentative_sign_simd_i        ( tentative_sign_simd_i ),
    .sticky_before_add_simd_o     ( sticky_before_add_simd_o ),
    .sum_simd_o                   ( sum_simd_o ),
    .final_sign_simd_o            ( final_sign_simd_o ),
    .mantissa_c_fp8_1_i            ( mantissa_c_fp8_1_i ),
    .product_shifted_fp8_1_i       ( product_shifted_fp8_1_from_packed ),
    .addend_shamt_fp8_1_i          ( addend_shamt_fp8_1_i ),
    .effective_subtraction_fp8_1_i ( effective_subtraction_fp8_1_i ),
    .tentative_sign_fp8_1_i        ( tentative_sign_fp8_1_i ),
    .sticky_before_add_fp8_1_o     ( sticky_before_add_fp8_1_o ),
    .sum_fp8_1_o                   ( sum_fp8_1_o ),
    .final_sign_fp8_1_o            ( final_sign_fp8_1_o ),
    .mantissa_c_fp8_2_i            ( mantissa_c_fp8_2_i ),
    .product_shifted_fp8_2_i       ( product_shifted_fp8_2_from_packed ),
    .addend_shamt_fp8_2_i          ( addend_shamt_fp8_2_i ),
    .effective_subtraction_fp8_2_i ( effective_subtraction_fp8_2_i ),
    .tentative_sign_fp8_2_i        ( tentative_sign_fp8_2_i ),
    .sticky_before_add_fp8_2_o     ( sticky_before_add_fp8_2_o ),
    .sum_fp8_2_o                   ( sum_fp8_2_o ),
    .final_sign_fp8_2_o            ( final_sign_fp8_2_o ),
    .safedot_alarm_o               ( /* unused in this variant */ )
  );
endmodule

module transdot_decomp_addend_datapath_piped_combined_product_dp #(
  parameter int unsigned SUPER_MAN_BITS     = 23,
  parameter int unsigned PRECISION_BITS     = SUPER_MAN_BITS + 1,
  parameter int unsigned SHIFT_AMOUNT_WIDTH = $clog2(3 * PRECISION_BITS + 5),
  parameter int unsigned SUPER_MAN_BITS_SIMD     = 10,
  parameter int unsigned PRECISION_BITS_SIMD     = SUPER_MAN_BITS_SIMD + 1,
  parameter int unsigned SHIFT_AMOUNT_WIDTH_SIMD = $clog2(3 * PRECISION_BITS_SIMD + 5),
  parameter int unsigned SUPER_MAN_BITS_FP8     = 4,
  parameter int unsigned PRECISION_BITS_FP8     = SUPER_MAN_BITS_FP8 + 1,
  parameter int unsigned SHIFT_AMOUNT_WIDTH_FP8 = $clog2(3 * PRECISION_BITS_FP8 + 5)
)(
  input  logic                               clk_i,
  input  logic                               rst_ni,
  input  logic                               pipe_en,
  input  logic                               dp_enable_i,
  // fp4_enable_i selects the FP4 8-lane DP path. FP4 uses a fixed
  // anchor (10'sd132) in transdot_decomp_exponent_datapath_fp8 that
  // already accounts for log2(8)=3 bits of lane-sum growth, so its
  // product padding stays at the original 4'd0. Only the FP8/FP16
  // 2/4-lane DP path needs the +2 anchor + 3'd0 compensation.
  input  logic                               fp4_enable_i,

  input  logic [PRECISION_BITS-1:0]          mantissa_c_i,
  input  logic [2*PRECISION_BITS-1:0]        product_comb_i,
  input  logic [2*PRECISION_BITS-1:0]        product_dp_i,
  input  logic [SHIFT_AMOUNT_WIDTH-1:0]      addend_shamt_i,
  input  logic                               effective_subtraction_i,
  input  logic                               tentative_sign_i,

  output logic                               sticky_before_add_o,
  output logic [3*PRECISION_BITS+3:0]        sum_o,
  output logic                               final_sign_o,

  input  logic                               simd_enable_i,
  input  logic                               is_fp8,

  input  logic [PRECISION_BITS_SIMD-1:0]     mantissa_c_simd_i,
  input  logic [SHIFT_AMOUNT_WIDTH_SIMD-1:0] addend_shamt_simd_i,
  input  logic                               effective_subtraction_simd_i,
  input  logic                               tentative_sign_simd_i,
  output logic                               sticky_before_add_simd_o,
  output logic [3*PRECISION_BITS_SIMD+3:0]   sum_simd_o,
  output logic                               final_sign_simd_o,

  input  logic [PRECISION_BITS_FP8-1:0]      mantissa_c_fp8_1_i,
  input  logic [SHIFT_AMOUNT_WIDTH_FP8-1:0]  addend_shamt_fp8_1_i,
  input  logic                               effective_subtraction_fp8_1_i,
  input  logic                               tentative_sign_fp8_1_i,
  output logic                               sticky_before_add_fp8_1_o,
  output logic [3*PRECISION_BITS_FP8+3:0]    sum_fp8_1_o,
  output logic                               final_sign_fp8_1_o,

  input  logic [PRECISION_BITS_FP8-1:0]      mantissa_c_fp8_2_i,
  input  logic [SHIFT_AMOUNT_WIDTH_FP8-1:0]  addend_shamt_fp8_2_i,
  input  logic                               effective_subtraction_fp8_2_i,
  input  logic                               tentative_sign_fp8_2_i,
  output logic                               sticky_before_add_fp8_2_o,
  output logic [3*PRECISION_BITS_FP8+3:0]    sum_fp8_2_o,
  output logic                               final_sign_fp8_2_o,
  // SafeDot shadow alarm pass-through (constant 0 when disabled).
  output logic [3:0]                         safedot_alarm_o
);

  logic [3*PRECISION_BITS+3:0]      product_shifted_selected;
  logic [3*PRECISION_BITS_SIMD+3:0] product_shifted_simd_from_packed;
  logic [3*PRECISION_BITS_FP8+3:0]  product_shifted_fp8_1_from_packed;
  logic [3*PRECISION_BITS_FP8+3:0]  product_shifted_fp8_2_from_packed;

  always_comb begin
    if (dp_enable_i) begin
      // FP4 path uses fixed anchor +132 sized for 8-lane growth → keep
      // 4'd0. FP8/FP16 2/4-lane DP path uses anchor +2 in the exp
      // datapath → use 3'd0 to keep the represented value invariant.
      product_shifted_selected = fp4_enable_i ? {'0, product_dp_i, 4'd0}
                                              : {'0, product_dp_i, 3'd0};
    end else if (!simd_enable_i) begin
      product_shifted_selected = {'0, product_comb_i, 2'b00};
    end else if (is_fp8) begin
      product_shifted_selected = {
        {(3*PRECISION_BITS+4-(2*PRECISION_BITS+3)){1'b0}},
        1'b0,
        product_comb_i[2*PRECISION_BITS_FP8-1:0],
        {(2*PRECISION_BITS-2*PRECISION_BITS_FP8){1'b0}},
        2'b00
      };
    end else begin
      product_shifted_selected = {
        {(3*PRECISION_BITS+4-(2*PRECISION_BITS+3)){1'b0}},
        1'b0,
        product_comb_i[PRECISION_BITS-3-:2*PRECISION_BITS_SIMD],
        {(2*PRECISION_BITS-2*PRECISION_BITS_SIMD){1'b0}},
        2'b00
      };
    end
  end

  assign product_shifted_simd_from_packed = is_fp8
      ? {{(3*PRECISION_BITS_SIMD+4-25){1'b0}}, 1'b0, product_comb_i[19:12], 16'd0}
      : {{(3*PRECISION_BITS_SIMD+4-25){1'b0}}, 1'b0,
         product_comb_i[2*PRECISION_BITS-3-:2*PRECISION_BITS_SIMD], 2'b00};
  assign product_shifted_fp8_1_from_packed =
      {{(3*PRECISION_BITS_FP8+4-11){1'b0}}, 1'b0, product_comb_i[31:24], 2'b00};
  assign product_shifted_fp8_2_from_packed =
      {{(3*PRECISION_BITS_FP8+4-11){1'b0}}, 1'b0, product_comb_i[43:36], 2'b00};

  transdot_decomp_addend_datapath_piped #(
    .SUPER_MAN_BITS         ( SUPER_MAN_BITS ),
    .PRECISION_BITS         ( PRECISION_BITS ),
    .SHIFT_AMOUNT_WIDTH     ( SHIFT_AMOUNT_WIDTH ),
    .SUPER_MAN_BITS_SIMD    ( SUPER_MAN_BITS_SIMD ),
    .PRECISION_BITS_SIMD    ( PRECISION_BITS_SIMD ),
    .SHIFT_AMOUNT_WIDTH_SIMD( SHIFT_AMOUNT_WIDTH_SIMD ),
    .SUPER_MAN_BITS_FP8     ( SUPER_MAN_BITS_FP8 ),
    .PRECISION_BITS_FP8     ( PRECISION_BITS_FP8 ),
    .SHIFT_AMOUNT_WIDTH_FP8 ( SHIFT_AMOUNT_WIDTH_FP8 )
  ) i_decomp_addend_datapath_combined_dp (
    .clk_i                   ( clk_i ),
    .rst_ni                  ( rst_ni ),
    .pipe_en                 ( pipe_en ),
    .mantissa_c_i            ( mantissa_c_i ),
    .product_shifted_i       ( product_shifted_selected ),
    .addend_shamt_i          ( addend_shamt_i ),
    .effective_subtraction_i ( effective_subtraction_i ),
    .tentative_sign_i        ( tentative_sign_i ),
    .sticky_before_add_o     ( sticky_before_add_o ),
    .sum_o                   ( sum_o ),
    .final_sign_o            ( final_sign_o ),
    .simd_enable_i           ( simd_enable_i ),
    .is_fp8                  ( is_fp8 ),
    .mantissa_c_simd_i            ( mantissa_c_simd_i ),
    .product_shifted_simd_i       ( product_shifted_simd_from_packed ),
    .addend_shamt_simd_i          ( addend_shamt_simd_i ),
    .effective_subtraction_simd_i ( effective_subtraction_simd_i ),
    .tentative_sign_simd_i        ( tentative_sign_simd_i ),
    .sticky_before_add_simd_o     ( sticky_before_add_simd_o ),
    .sum_simd_o                   ( sum_simd_o ),
    .final_sign_simd_o            ( final_sign_simd_o ),
    .mantissa_c_fp8_1_i            ( mantissa_c_fp8_1_i ),
    .product_shifted_fp8_1_i       ( product_shifted_fp8_1_from_packed ),
    .addend_shamt_fp8_1_i          ( addend_shamt_fp8_1_i ),
    .effective_subtraction_fp8_1_i ( effective_subtraction_fp8_1_i ),
    .tentative_sign_fp8_1_i        ( tentative_sign_fp8_1_i ),
    .sticky_before_add_fp8_1_o     ( sticky_before_add_fp8_1_o ),
    .sum_fp8_1_o                   ( sum_fp8_1_o ),
    .final_sign_fp8_1_o            ( final_sign_fp8_1_o ),
    .mantissa_c_fp8_2_i            ( mantissa_c_fp8_2_i ),
    .product_shifted_fp8_2_i       ( product_shifted_fp8_2_from_packed ),
    .addend_shamt_fp8_2_i          ( addend_shamt_fp8_2_i ),
    .effective_subtraction_fp8_2_i ( effective_subtraction_fp8_2_i ),
    .tentative_sign_fp8_2_i        ( tentative_sign_fp8_2_i ),
    .sticky_before_add_fp8_2_o     ( sticky_before_add_fp8_2_o ),
    .sum_fp8_2_o                   ( sum_fp8_2_o ),
    .final_sign_fp8_2_o            ( final_sign_fp8_2_o ),
    .safedot_alarm_o               ( safedot_alarm_o )
  );
endmodule
