
`ifndef STRONG_NOT_TAKEN
`define STRONG_NOT_TAKEN 2'b00
`define WEAK_NOT_TAKEN   2'b01
`define WEAK_TAKEN       2'b10
`define STRONG_TAKEN     2'b11
`endif
/*
// -----------------------------------------------------------------------------
// pc - simple PC register (async reset)
// -----------------------------------------------------------------------------
module pc (
    input  wire        clk,
    input  wire        rst,      // active-high asynchronous reset
    input  wire [31:0] next_pc,
    input  wire        pc_en,    // enable update (for stalls)
    output reg  [31:0] pc
);
    always @(posedge clk or posedge rst) begin
        if (rst) pc <= 32'h00000000;
        else if (pc_en) pc <= next_pc;
        // else hold (stall)
    end
endmodule

// -----------------------------------------------------------------------------
// pc_update - choose next PC from EX override, BTB prediction, or pc+4
// -----------------------------------------------------------------------------
module pc_update(
    input wire [31:0] pc,
    input wire [31:0] pc_jump_addr, // jump target computed in EX (override)
    input wire [31:0] btb_target_pc, // BTB predicted target
    input wire btb_pc_valid, // BTB has a valid entry for this PC
    input wire btb_pc_predictTaken, // BTB prediction: taken?
    input wire jump_en, // EX indicates an immediate override (e.g., resolved taken)
    output reg [31:0] next_pc
);
    // Selection priority:
    // 1) EX override (jump_en) -> pc_jump_addr
    // 2) BTB predicted taken -> btb_target_pc
    // 3) Default sequential PC+4
    wire [31:0] pc_plus_4 = pc + 32'h4;
    always @(*) begin
        if (jump_en) next_pc = pc_jump_addr;
        else if (btb_pc_valid && btb_pc_predictTaken) next_pc = btb_target_pc;
        else next_pc = pc_plus_4;
    end
endmodule
// -----------------------------------------------------------------------------
// if_stage_complex_btb - IF stage with BTB lookup + next PC selection
// -----------------------------------------------------------------------------
module if_stage_complex_btb (
    input wire clk,
    input wire rst,
    // Hazard unit control
    input wire pc_en, // allow updating PC
    input wire if_id_en, // allow IF/ID latch (unused inside IF stage, forwarded by top)
    input wire if_id_flush, // flush IF/ID pipeline reg (unused inside IF stage, forwarded by top)
    // From EX stage for immediate resolution / updates
    input wire modify_pc_ex, // EX indicates an immediate override (e.g., branch taken/mispredict)
    input wire [31:0] update_pc_ex, // corrected PC from EX (override next PC)
    input wire [31:0] pc_ex, // branch PC from EX (for BTB update index/tag)
    input wire [31:0] jump_addr_ex, // computed target from EX (for BTB update_target)
    input wire update_btb_ex, // update BTB on resolved control-flow
    // IF outputs
    output [31:0] pc_if,
    output wire [31:0] instr_if,
    output wire predictedTaken_if,
    output wire [31:0] predictedTarget_if
);
    // -------------------------------------------------------------
    // 1) PC REGISTER
    // -------------------------------------------------------------
    wire [31:0] pc_next;
    reg [31:0] pc_reg;
    always @(posedge clk or posedge rst) begin
        if (rst)
            pc_reg <= 32'h0000_0000;
        else if (pc_en)
            pc_reg <= pc_next;
        // else hold
    end
    assign pc_if = pc_reg;

    // -------------------------------------------------------------
    // 2) COMPLEX BTB LOOKUP (read-only from IF)
    // -------------------------------------------------------------
    wire [31:0] btb_target;
    wire btb_valid;
    wire btb_predictedTaken;

    // The btb module performs read using pc_reg and may accept update inputs from EX stage.
    btb u_btb (
        .clk(clk),
        .rst(rst),
        .pc(pc_reg),
        .update_pc(pc_ex),         // branch PC (for update index/tag)
        .update(update_btb_ex),    // update request from EX
        .update_target(jump_addr_ex),
        .mispredicted(1'b0),       // pass-through placeholder; BTB internal FSM uses provided signals during update path
        .target_pc(btb_target),
        .valid(btb_valid),
        .predictedTaken(btb_predictedTaken)
    );

    assign predictedTaken_if = btb_predictedTaken & btb_valid;
    assign predictedTarget_if = (predictedTaken_if ? btb_target : pc_reg + 32'd4);

    // -------------------------------------------------------------
    // 3) Next-PC selection using pc_update module
    //    - If EX requests immediate override (modify_pc_ex) -> use update_pc_ex
    //    - Else if BTB predicts taken -> use btb_target
    //    - Else -> pc+4
    // -------------------------------------------------------------
    wire [31:0] pc_jump_addr = update_pc_ex;
    wire jump_en = modify_pc_ex;
    pc_update u_pcupdate (
        .pc(pc_reg),
        .pc_jump_addr(pc_jump_addr),
        .btb_target_pc(btb_target),
        .btb_pc_valid(btb_valid),
        .btb_pc_predictTaken(btb_predictedTaken),
        .jump_en(jump_en),
        .next_pc(pc_next)
    );

    // -------------------------------------------------------------
    // 4) Instruction Memory
    // -------------------------------------------------------------
    inst_mem u_imem (
        .pc(pc_reg),
        .read_en(1'b1),
        .instruction(instr_if)
    );
endmodule

*/

// -----------------------------------------------------------------------------
// inst_mem// -----------------------------------------------------------------------------
module inst_mem (
    input  wire [31:0] pc,       // byte address
    input  wire        read_en,  // ignored (always read in IF-stage), kept for API consistency
    output wire [31:0] instruction
);
    // Simple small instruction memory (256 x 32-bit words)
    reg [31:0] mem [0:255];

    initial begin
        
        $readmemh("instructions.hex", mem);
    end

    
    assign instruction = mem[pc[11:2]];
endmodule


module pc (
    input  wire        clk,
    input  wire        rst,      // active-high async reset
    input  wire [31:0] next_pc,
    input  wire        pc_en,    // enable update (for stall)
    output reg  [31:0] pc
);
    always @(posedge clk or posedge rst) begin
        if (rst)
            pc <= 32'h0000_0000;
        else if (pc_en)
            pc <= next_pc;
        // else hold (stall)
    end
endmodule

module pc_update(
    input wire [31:0] pc,
    input wire [31:0] pc_jump_addr,      // EX override target
    input wire [31:0] btb_target_pc,     // BTB predicted target
    input wire        btb_pc_valid,      // BTB hit
    input wire        btb_pc_predictTaken, // BTB predicted taken
    input wire        jump_en,           // EX override enable (actual_taken)
    output reg [31:0] next_pc
);

    wire [31:0] pc_plus_4 = pc + 32'h4;

    always @(*) begin
        // Priority:
        // 1) EX override (resolved taken or mispredict correction)
        if (jump_en)
            next_pc = pc_jump_addr;

        // 2) BTB predicted taken
        else if (btb_pc_valid && btb_pc_predictTaken)
            next_pc = btb_target_pc;

        // 3) default sequential
        else
            next_pc = pc_plus_4;
    end
endmodule

// -----------------------------------------------------------------------------
// IF stage with integrated BTB prediction + EX override support
// -----------------------------------------------------------------------------
module if_stage_complex_btb (
    input wire clk,
    input wire rst,

    // Hazard controls
    input wire pc_en,
    input wire if_id_en,       // (unused here, forwarded to pipeline)
    input wire if_id_flush,    // (unused here, forwarded to pipeline)

    // EX stage update / override signals
    input wire        modify_pc_ex,    // actual_taken (branch/jump resolved taken)
    input wire [31:0] update_pc_ex,    // corrected PC from EX
    input wire [31:0] pc_ex,           // original PC of branch (for BTB update)
    input wire [31:0] jump_addr_ex,    // resolved branch/jump target
    input wire        update_btb_ex,   // BTB update enable (branch resolved)

    // IF outputs
    output      [31:0] pc_if,
    output wire [31:0] instr_if,
    output wire        predictedTaken_if,
    output wire [31:0] predictedTarget_if
);

    // ============================================================
    // 1) PC REGISTER
    // ============================================================
    reg  [31:0] pc_reg;
    wire [31:0] pc_next;

    always @(posedge clk or posedge rst) begin
        if (rst)
            pc_reg <= 32'h00000000;
        else if (pc_en)
            pc_reg <= pc_next;
        // else hold for stall
    end

    assign pc_if = pc_reg;

    // ============================================================
    // 2) BTB READ + UPDATE
    // ============================================================
    wire [31:0] btb_target;
    wire        btb_valid;
    wire        btb_predictedTaken;

    btb u_btb (
        .clk(clk),
        .rst(rst),

        // ----- READ -----
        .pc(pc_reg),

        // ----- UPDATE -----
        .update_pc(pc_ex),           // PC of resolved branch
        .update(update_btb_ex),      // update enable
        .update_taken(modify_pc_ex), // ACTUAL branch outcome (correct)
        .update_target(jump_addr_ex),

        // ----- OUTPUT -----
        .target_pc(btb_target),
        .valid(btb_valid),
        .predictedTaken(btb_predictedTaken)
    );

    assign predictedTaken_if  = btb_valid && btb_predictedTaken;
    assign predictedTarget_if = predictedTaken_if ? btb_target : pc_reg + 32'd4;

    // ============================================================
    // 3) NEXT PC SELECTION (PC UPDATE LOGIC)
    // ============================================================
    pc_update u_pcupdate (
        .pc(pc_reg),
        .pc_jump_addr(update_pc_ex),     // EX override target
        .btb_target_pc(btb_target),
        .btb_pc_valid(btb_valid),
        .btb_pc_predictTaken(btb_predictedTaken),
        .jump_en(modify_pc_ex),          // EX override enable
        .next_pc(pc_next)
    );

    // ============================================================
    // 4) INSTRUCTION MEMORY
    // ============================================================
    inst_mem u_imem (
        .pc(pc_reg),
        .read_en(1'b1),
        .instruction(instr_if)
    );

endmodule


// -----------------------------------------------------------------------------
// dynamic_branch_predictor - 2-bit saturating counter update
// -----------------------------------------------------------------------------
module dynamic_branch_predictor(
    input  [1:0] current_state,
    input        actual_taken,     // corrected input name
    output reg [1:0] next_state
);

    always @(*) begin
        if (actual_taken) begin
            case (current_state)
                2'b00: next_state = 2'b01; // SN -> WN
                2'b01: next_state = 2'b10; // WN -> WT
                2'b10: next_state = 2'b11; // WT -> ST
                2'b11: next_state = 2'b11; // ST -> ST
                default: next_state = 2'b01;
            endcase
        end else begin
            case (current_state)
                2'b00: next_state = 2'b00; // SN -> SN
                2'b01: next_state = 2'b00; // WN -> SN
                2'b10: next_state = 2'b01; // WT -> WN
                2'b11: next_state = 2'b10; // ST -> WT
                default: next_state = 2'b01;
            endcase
        end
    end

endmodule
module btb_write(
    input  [127:0] update_set,
    input  [7:0]   LRU,
    input  [26:0]  update_tag,
    input  [2:0]   update_index,
    input  [31:0]  update_target,
    input          update_taken,        // corrected signal name
    output [127:0] write_set,
    output         next_LRU_write
);

    // Extract ways
    wire [63:0] way0 = update_set[127:64];
    wire [63:0] way1 = update_set[63:0];

    wire valid0 = way0[63];
    wire valid1 = way1[63];

    wire [26:0] tag0 = way0[62:36];
    wire [26:0] tag1 = way1[62:36];

    wire [31:0] tgt0 = way0[35:4];
    wire [31:0] tgt1 = way1[35:4];

    wire [1:0] state0 = way0[3:2];
    wire [1:0] state1 = way1[3:2];

    wire hit0 = valid0 && (tag0 == update_tag);
    wire hit1 = valid1 && (tag1 == update_tag);
    wire entry_exists = hit0 || hit1;

    // LRU selection
    wire lru_bit = LRU[update_index];     // 0 = way0 LRU, 1 = way1 LRU

    wire insert0 = entry_exists ? hit0 : (lru_bit == 1'b0);
    wire insert1 = entry_exists ? hit1 : (lru_bit == 1'b1);

    // Predictor FSMs
    wire [1:0] next_state0, next_state1;

    dynamic_branch_predictor p0(
        .current_state(entry_exists ? state0 : 2'b01),   // init = weakly not taken
        .actual_taken(update_taken),
        .next_state(next_state0)
    );

    dynamic_branch_predictor p1(
        .current_state(entry_exists ? state1 : 2'b01),
        .actual_taken(update_taken),
        .next_state(next_state1)
    );

    // Select write state
    wire [1:0] write_state0 = insert0 ? next_state0 : state0;
    wire [1:0] write_state1 = insert1 ? next_state1 : state1;

    // Correct target update rule (match Code-B)
    wire [31:0] write_tgt0 = insert0 ?
                                (update_taken ? update_target : (update_index * 4)) :
                                tgt0;

    wire [31:0] write_tgt1 = insert1 ?
                                (update_taken ? update_target : (update_index * 4)) :
                                tgt1;

    // Form 64-bit entries (removed padding bits!)
    wire [63:0] new_way0 =
        { (valid0 | insert0),
          update_tag,
          write_tgt0,
          write_state0 };

    wire [63:0] new_way1 =
        { (valid1 | insert1),
          update_tag,
          write_tgt1,
          write_state1 };

    assign write_set = { new_way0, new_way1 };

    // Replacement policy
    assign next_LRU_write = entry_exists ? lru_bit : insert1;

endmodule
module btb_file (
    input  wire        clk,
    input  wire        rst,
    input  wire [2:0]  read_index,
    input  wire [2:0]  update_index,
    input  wire [2:0]  write_index,
    input  wire [127:0] write_set,
    input  wire        write_en,

    output wire [127:0] read_set,
    output wire [127:0] update_set
);

    // 8 sets × 128-bit entries
    reg [127:0] file [0:7];
    integer i;

    // Synchronous write + synchronous reset
    always @(posedge clk) begin
        if (rst) begin
            // NOTE: pure Verilog increment: i = i + 1
            for (i = 0; i < 8; i = i + 1)
                file[i] <= 128'h0;
        end 
        else if (write_en) begin
            file[write_index] <= write_set;
        end
    end

    // Read operations (combinational)
    assign update_set = file[update_index];

    // Read-forwarding to prevent hazards
    assign read_set = (write_en && (write_index == read_index)) ?
                        write_set :
                        file[read_index];

endmodule
module btb_read(
    input  [127:0] read_set,
    input  [7:0]   LRU,
    input  [26:0]  read_tag,
    input  [2:0]   read_index,

    output         next_LRU_read,
    output         valid,
    output         predictedTaken,
    output [31:0]  target
);

    wire [63:0] way0 = read_set[127:64];
    wire [63:0] way1 = read_set[63:0];

    wire valid0 = way0[63];
    wire valid1 = way1[63];

    wire [26:0] tag0 = way0[62:36];
    wire [26:0] tag1 = way1[62:36];

    wire [31:0] tgt0 = way0[35:4];
    wire [31:0] tgt1 = way1[35:4];

    wire [1:0] state0 = way0[3:2];
    wire [1:0] state1 = way1[3:2];

    wire hit0 = valid0 && (tag0 == read_tag);
    wire hit1 = valid1 && (tag1 == read_tag);

    assign valid = hit0 || hit1;
    assign target = hit0 ? tgt0 : tgt1;
    assign predictedTaken = hit0 ? state0[1] :
                            hit1 ? state1[1] : 1'b0;

    // LRU update on read hit
    wire curr_lru = LRU[read_index];
    assign next_LRU_read = valid ? hit1 : curr_lru;

endmodule
module btb(
    input clk,
    input rst,
    input [31:0] pc,
    input [31:0] update_pc,
    input        update,
    input        update_taken,
    input [31:0] update_target,

    output [31:0] target_pc,
    output        valid,
    output        predictedTaken
);

    wire [2:0] read_index = pc[4:2];
    wire [26:0] read_tag = pc[31:5];

    wire [2:0] update_index = update_pc[4:2];
    wire [26:0] update_tag  = update_pc[31:5];

    wire [127:0] read_set, update_set, write_set;

    // LRU
    wire [7:0] LRU, next_LRU;
    wire next_lru_read, next_lru_write;

    // LRU register
    lru_reg lru_reg_inst(
        .clk(clk),
        .rst(rst),
        .LRU_updated(next_LRU),
        .LRU(LRU)
    );

    // BTB file
    btb_file btb_file_inst(
        .clk(clk),
        .rst(rst),
        .read_index(read_index),
        .update_index(update_index),
        .write_index(update_index),
        .write_set(write_set),
        .write_en(update),
        .read_set(read_set),
        .update_set(update_set)
    );

    // READ
    btb_read btb_read_inst(
        .read_set(read_set),
        .LRU(LRU),
        .read_tag(read_tag),
        .read_index(read_index),
        .next_LRU_read(next_lru_read),
        .valid(valid),
        .predictedTaken(predictedTaken),
        .target(target_pc)
    );

    // WRITE
    btb_write btb_write_inst(
        .update_set(update_set),
        .LRU(LRU),
        .update_tag(update_tag),
        .update_index(update_index),
        .update_target(update_target),
        .update_taken(update_taken),
        .write_set(write_set),
        .next_LRU_write(next_lru_write)
    );

    // LRU update
    lru_next lru_next_inst(
        .index(read_index),
        .update_index(update_index),
        .update_lru_read(next_lru_read),
        .update_lru_write(next_lru_write),
        .valid(valid),
        .update(update),
        .LRU(LRU),
        .next_LRU(next_LRU)
    );

endmodule

module lru_next(
    input [2:0] index,
    input [2:0] update_index,
    input update_lru_read,
    input update_lru_write,
    input valid,
    input update,
    input [7:0] LRU,
    output reg [7:0] next_LRU
);
    reg [7:0] read_mask;
    reg [7:0] write_mask;
    reg [7:0] update_mask;
    reg [7:0] update_bits;

    always @* begin
        read_mask   = (8'b00000001 << index);
        write_mask  = (8'b00000001 << update_index);

        update_mask = read_mask | write_mask;

        update_bits = (update_lru_read  && valid  ? read_mask  : 8'b00000000)
                    | (update_lru_write && update ? write_mask : 8'b00000000);

        next_LRU = (LRU & ~update_mask) | update_bits;
    end

endmodule 



