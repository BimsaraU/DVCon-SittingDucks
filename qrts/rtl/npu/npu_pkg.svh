// =============================================================================
// npu_pkg.svh - shared constants for the DVCon NPU
//
// These mirror compiler/npu_isa.py, which is the specification. If a number
// changes here it changes there, and the golden model decides who is right.
// =============================================================================
`ifndef NPU_PKG_SVH
`define NPU_PKG_SVH

localparam integer NPU_LANES     = 16;     // array edge, channels per block
localparam integer NPU_IBUF_ROWS = 4096;   // input band buffer, 16-byte rows
localparam integer NPU_WBUF_ROWS = 4096;   // weight buffer, 16-byte rows
localparam integer NPU_ACC_DEPTH = 1024;   // pixels per band
localparam integer NPU_P_MIN     = 48;     // minimum slots per array pass
localparam integer NPU_BANK_WORDS = 2048;  // elementwise scratch bank
localparam integer NPU_MAX_BOXES = 300;

// opcodes (descriptor word 0 [7:0])
localparam [7:0] OP_END      = 8'd0,
                 OP_CONV     = 8'd1,
                 OP_ADD      = 8'd2,
                 OP_UPSAMPLE = 8'd3,
                 OP_MAXPOOL  = 8'd4,
                 OP_SOFTMAX  = 8'd5,
                 OP_PACK     = 8'd6,
                 OP_DETECT   = 8'd7;

// conv flags (descriptor word 0 [15:8])
localparam integer FB_DW  = 0;   // depthwise
localparam integer FB_RT  = 1;   // runtime weights from src1
localparam integer FB_TR  = 2;   // runtime tiles stored [oc][red]: column load
localparam integer FB_RES = 3;   // whole layer's weights resident

// IDENT register: magic, array edge, version
localparam [31:0] NPU_IDENT = 32'hDC10_0200;

`endif
