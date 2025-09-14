// ------------------------- Program Counter -----------------------
module Program_Counter(
    input clk,
    input rst,                
    input [31:0] pc_in,
    output reg [31:0] pc_out
);
    always @(posedge clk) 
	begin
        if (rst) 
		pc_out <= 32'b0;
        else 
		pc_out <= pc_in;
    	end
endmodule

// ------------------------- PC Adder (+4) -------------------------
module PC_Adder(
    input  [31:0] pc_in,
    output [31:0] pc_next
);
    assign pc_next = pc_in + 4;
endmodule

// ------------------------- PC MUX (next PC selection) -------------
module PC_MUX(
    input  [31:0] pc_seq,
    input  [31:0] pc_branch,
    input         pc_select,   
    output [31:0] pc_out
);
    assign pc_out = pc_select ? pc_branch : pc_seq;
endmodule

// --------------------- Program Memory (4KB ROM) -------------------
module Program_Memory(
    input  [31:0] read_address,   // byte address from PC
    output [31:0] instruction_out
);
    // 1024 words (4 KB memory)
    reg [31:0] 	Mem_Array[0:1023];
    wire [9:0] word_index;

    // word aligned (PC[11:2] for 1024 words)
    assign word_index = read_address[11:2];
    assign instruction_out = Mem_Array[word_index];
    
    initial begin
        $display("Loading program.hex...");
        $readmemh("program.hex", Mem_Array);
        $display("Program loaded successfully");
        // Display first few instructions for testing
        $display("Program Memory: index[0] = 0x%08h", Mem_Array[0]);
        $display("Program Memory: index[1] = 0x%08h", Mem_Array[1]);
        $display("Program Memory: index[2] = 0x%08h", Mem_Array[2]);
	$display("Program Memory: index[3] = 0x%08h", Mem_Array[3]);
    end
endmodule


// ------------------------- Register File -------------------------
module REG_FILE(
    input         clk,
    input         reset,       
    input         RegWrite,
    input  [4:0]  Rs1,
    input  [4:0]  Rs2,
    input  [4:0]  Rd,
    input  [31:0] write_data,
    output [31:0] read_data1,
    output [31:0] read_data2
);
    reg [31:0] Registers [31:0];
    integer i;

    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) Registers[i] <= 32'b0;
        end else begin
            if (RegWrite && (Rd != 5'b0)) Registers[Rd] <= write_data;
            Registers[0] <= 32'b0; 
        end
    end

    assign read_data1 = (Rs1 == 5'b0) ? 32'b0 : Registers[Rs1];
    assign read_data2 = (Rs2 == 5'b0) ? 32'b0 : Registers[Rs2];
endmodule

// ------------------------- Control Unit -------------------------
module CONTROL_UNIT(
    input  [6:0] opcode,          // Instruction opcode field
    output reg   RegWrite,        // Register write enable
    output reg   MemRead,         // Memory read enable
    output reg   MemWrite,        // Memory write enable
    output reg   MemToReg,        // Select between ALU result or memory data
    output reg   ALUSrc,          // Select between register file or immediate
    output reg   Branch,          // Branch control
    output reg [1:0] ALUOp        // ALU operation selector
);

    always @(*) begin
        case (opcode)
            7'b0110011: // R-type (add, sub, and, or, slt, etc.)
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b0,   1'b0,     1'b1,     1'b0,    1'b0,    1'b0,   2'b10};

            7'b0010011: // I-type ALU (addi, andi, ori, xori, shifts, slti,)
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b1,   1'b0,     1'b1,     1'b0,    1'b0,    1'b0,   2'b10};

            7'b0000011: // Load (LB, LH, LW, LBU, LHU)
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b1,   1'b1,     1'b1,     1'b1,    1'b0,    1'b0,   2'b00};

            7'b0100011: // Store (SB, SH, SW)
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b1,   1'b0,     1'b0,     1'b0,    1'b1,    1'b0,   2'b00};

            7'b1100011: // Branch (BEQ, BNE, BLT, BGE, BLTU, BGEU)
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b0,   1'b0,     1'b0,     1'b0,    1'b0,    1'b1,   2'b01};

            7'b1101111: // JAL 
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b0,   1'b0,     1'b1,     1'b0,    1'b0,    1'b0,   2'b00};

            7'b1100111: // JALR
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b1,   1'b0,     1'b1,     1'b0,    1'b0,    1'b0,   2'b00};

            7'b0110111: // LUI
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b1,   1'b0,     1'b1,     1'b0,    1'b0,    1'b0,   2'b11};

            7'b0010111: // AUIPC
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b1,   1'b0,     1'b1,     1'b0,    1'b0,    1'b0,   2'b00};

            default: // Safe defaults (NOP-like)
                {ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} =
                {1'b0,   1'b0,     1'b0,     1'b0,    1'b0,    1'b0,   2'b00};
        endcase
    end

endmodule


// ------------------------- Immediate Generator -------------------------
module Imm_Gen(
    input  [31:0] instruction,
    output reg [31:0] imm_out
);
    wire [6:0] opcode;
    assign opcode = instruction[6:0];

    always @(*) begin
        case (opcode)
            7'b0010011: // I-type ALU
                imm_out = {{20{instruction[31]}}, instruction[31:20]};
            7'b0000011: // Load I-type
                imm_out = {{20{instruction[31]}}, instruction[31:20]};
            7'b1100111: // JALR I-type
                imm_out = {{20{instruction[31]}}, instruction[31:20]};
            7'b0100011: // S-type (store)
                imm_out = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            7'b1100011: // B-type (branch)
                imm_out = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            7'b0110111: // LUI U-type
                imm_out = {instruction[31:12], 12'b0};
            7'b0010111: // AUIPC U-type
                imm_out = {instruction[31:12], 12'b0};
            7'b1101111: // J-type (JAL)
                imm_out = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            default:
                imm_out = 32'b0;
        endcase
    end
endmodule

// ------------------------- ALU -------------------------
module ALU(
    input  [31:0] A,
    input  [31:0] B,
    input  [3:0]  ALUcontrol_In,
    output reg [31:0] Result,
    output reg        Zero
);
    always @(*) begin
        case (ALUcontrol_In)
            4'b0000: Result = A + B;                        // ADD / ADDI / AUIPC
            4'b0001: Result = A - B;                        // SUB / compare for branch
            4'b0010: Result = A & B;                        // AND
            4'b0011: Result = A | B;                        // OR
            4'b0100: Result = A ^ B;                        // XOR
            4'b0101: Result = A << B[4:0];                  // SLL
            4'b0110: Result = A >> B[4:0];                  // SRL (logical)
            4'b0111: Result = $signed(A) >>> B[4:0];        // SRA (arithmetic)
            4'b1000: Result = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0; // SLT
            4'b1001: Result = (A < B) ? 32'b1 : 32'b0;      // SLTU
            4'b1100: Result = B;                            // LUI -> pass immediate (already shifted)
            default: Result = 32'b0;
        endcase
        Zero = (Result == 32'b0);
    end
endmodule

// ------------------------- ALU Control -------------------------
module ALU_Control(
    input  [2:0] funct3,
    input  [6:0] funct7,
    input  [1:0] ALUOp,
    output reg [3:0] ALUcontrol_Out
);
    reg [9:0] fn;
    always @(*) begin
        fn = {funct7[6:0], funct3}; // 7 + 3 -> 10 bits (we will inspect)
        case (ALUOp)
            2'b00: ALUcontrol_Out = 4'b0000; // ADD for loads/stores/AUIPC/JAL/JALR default
            2'b01: begin // Branches (funct3 decides)
                case (funct3)
                    3'b000: ALUcontrol_Out = 4'b0001; // BEQ/BNE -> subtract
                    3'b001: ALUcontrol_Out = 4'b0001; // BNE -> subtract too
                    3'b100: ALUcontrol_Out = 4'b1000; // BLT -> SLT
                    3'b101: ALUcontrol_Out = 4'b1000; // BGE -> SLT (ALU checks result)
                    3'b110: ALUcontrol_Out = 4'b1001; // BLTU -> SLTU
                    3'b111: ALUcontrol_Out = 4'b1001; // BGEU -> SLTU
                    default: ALUcontrol_Out = 4'b0001;
                endcase
            end
            2'b10: begin
                case ({funct7[5], funct3})                 
                    {1'b0, 3'b000}: ALUcontrol_Out = 4'b0000; // ADD/ADDI
                    {1'b1, 3'b000}: ALUcontrol_Out = 4'b0001; // SUB
                    {1'b0, 3'b100}: ALUcontrol_Out = 4'b0100; // XOR
                    {1'b0, 3'b110}: ALUcontrol_Out = 4'b0011; // OR
                    {1'b0, 3'b111}: ALUcontrol_Out = 4'b0010; // AND
                    {1'b0, 3'b001}: ALUcontrol_Out = 4'b0101; // SLL
                    {1'b0, 3'b101}: ALUcontrol_Out = 4'b0110; // SRL (funct7[5]=0)
                    {1'b1, 3'b101}: ALUcontrol_Out = 4'b0111; // SRA (funct7[5]=1)
                    {1'b0, 3'b010}: ALUcontrol_Out = 4'b1000; // SLT
                    {1'b0, 3'b011}: ALUcontrol_Out = 4'b1001; // SLTU
                    default: ALUcontrol_Out = 4'b0000;
                endcase
            end
            2'b11: ALUcontrol_Out = 4'b1100; // LUI
            default: ALUcontrol_Out = 4'b0000;
        endcase
    end
endmodule

// ------------------------- 2:1 MUX -------------------------
module MUX2to1 (
    input [31:0] input0,
    input [31:0] input1,
    input        select,
    output [31:0] out
);
    assign out = select ? input1 : input0;
endmodule

// ------------------------- Data Memory (4KB byte-addressable) -------------------------
// Supports LB(000), LH(001), LW(010), LBU(100), LHU(101)
// Supports SB(000), SH(001), SW(010)
// mem_op = funct3 from instruction to determine width & sign

module Data_Memory(
    input            clk,
    input            rst,
    input            MemRead,
    input            MemWrite,
    input  [2:0]     mem_op,      // funct3
    input  [31:0]    address,     // byte address
    input  [31:0]    write_data,  // data from register to write
    output reg [31:0] read_data
);
    // 4096 bytes
    reg [7:0] mem [0:4095];
    integer i;
    // synchronous reset 
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 4096; i = i + 1) mem[i] <= 8'b0;
        end
    end

    // Write logic (synchronous write)
    always @(posedge clk) begin
        if (MemWrite) begin
            case (mem_op)
                3'b000: begin // SB
                    mem[address] <= write_data[7:0];
                end
                3'b001: begin // SH (halfword, little endian)
                    mem[address]     <= write_data[7:0];
                    mem[address + 1] <= write_data[15:8];
                end
                3'b010: begin // SW (word)
                    mem[address]     <= write_data[7:0];
                    mem[address + 1] <= write_data[15:8];
                    mem[address + 2] <= write_data[23:16];
                    mem[address + 3] <= write_data[31:24];
                end
            endcase
        end
    end

    // Read logic (combinational read)
    always @(*) begin
        if (!MemRead) begin
            read_data = 32'b0;
        end else begin
            case (mem_op)
                3'b000: begin // LB (sign-extend byte)
                    read_data = {{24{mem[address][7]}}, mem[address]};
                end
                3'b100: begin // LBU (zero-extend byte)
                    read_data = {24'b0, mem[address]};
                end
                3'b001: begin // LH (sign-extend half)
                    read_data = {{16{mem[address+1][7]}}, mem[address+1], mem[address]};
                end
                3'b101: begin // LHU (zero-extend half)
                    read_data = {16'b0, mem[address+1], mem[address]};
                end
                3'b010: begin // LW
                    read_data = {mem[address+3], mem[address+2], mem[address+1], mem[address]};
                end
                default: read_data = 32'b0;
            endcase
        end
    end
endmodule

// ------------------------- Branch Adder -------------------------
module Branch_Adder(
    input  [31:0] PC,
    input  [31:0] offset,
    output [31:0] branch_target
);
    assign branch_target = PC + offset;
endmodule

// ------------------------- Top module (single-cycle) -------------------------
module RISCV_Top(
    input clk,
    input rst
);
    // Wires
    wire [31:0] pc_out, pc_seq, pc_branch, pc_next;
    wire [31:0] instr;
    wire [31:0] reg_read1, reg_read2;
    wire [31:0] imm;
    wire [31:0] alu_inB, alu_result;
    wire [31:0] mem_read_data;
    wire [31:0] wb_data;
    wire zero_flag;
    wire pc_src; // branch&zero or jump

    // Control signals
    wire RegWrite, MemRead, MemWrite, MemToReg, ALUSrc, Branch;
    wire [1:0] ALUOp;
    wire [3:0] ALUControl;

    // Instruction fields
    wire [6:0] opcode = instr[6:0];
    wire [4:0] rd     = instr[11:7];
    wire [2:0] funct3 = instr[14:12];
    wire [4:0] rs1    = instr[19:15];
    wire [4:0] rs2    = instr[24:20];
    wire [6:0] funct7 = instr[31:25];

    // Program counter & fetch
    Program_Counter PC(.clk(clk), .rst(rst), .pc_in(pc_out), .pc_out(pc_seq)); 
    PC_Adder PCplus4(.pc_in(pc_seq), .pc_next(pc_next));
    Branch_Adder Branch_Adder(.PC(pc_seq), .offset(imm), .branch_target(pc_branch));

    // Program_Memory (use pc_seq to read)
     Program_Memory Program_Memory(.read_address(pc_seq), .instruction_out(instr));

    // Control, immediate, regfile
    CONTROL_UNIT Control_Unit(.opcode(opcode), .RegWrite(RegWrite), .MemRead(MemRead),
                      .MemWrite(MemWrite), .MemToReg(MemToReg), .ALUSrc(ALUSrc),
                      .Branch(Branch), .ALUOp(ALUOp));
    Imm_Gen Imm_Gen(.instruction(instr), .imm_out(imm));
    REG_FILE Reg_File(.clk(clk), .reset(rst), .RegWrite(RegWrite),
                     .Rs1(rs1), .Rs2(rs2), .Rd(rd), .write_data(wb_data),
                     .read_data1(reg_read1), .read_data2(reg_read2));

    // ALU control & ALU
    ALU_Control ALU_Control(.funct3(funct3), .funct7(funct7), .ALUOp(ALUOp), .ALUcontrol_Out(ALUControl));
    MUX2to1 mux_aluB(.input0(reg_read2), .input1(imm), .select(ALUSrc), .out(alu_inB));
    ALU ALU(.A(reg_read1), .B(alu_inB), .ALUcontrol_In(ALUControl), .Result(alu_result), .Zero(zero_flag));

    // Data memory (byte-addressable). Address from ALU result
    Data_Memory Data_Memory(.clk(clk), .rst(rst), .MemRead(MemRead), .MemWrite(MemWrite),
                     .mem_op(funct3), .address(alu_result), .write_data(reg_read2),
                     .read_data(mem_read_data));

    // WB mux
    MUX2to1 wb_mux(.input0(alu_result), .input1(mem_read_data), .select(MemToReg), .out(wb_data));

    // Next PC selection:
     
    wire is_jal   = (opcode == 7'b1101111);
    wire is_jalr  = (opcode == 7'b1100111);

    // For JAL: target = pc_seq + imm  
    wire [31:0] jal_target  = pc_seq + imm;
    wire [31:0] jalr_target = { (reg_read1 + imm) & 32'hfffffffe };

    // Choose branch/jump target:
    wire [31:0] chosen_target = is_jal ? jal_target : (is_jalr ? jalr_target : pc_branch);

    // PC update selection: if (Branch & ALU zero) or is_jal or is_jalr -> take chosen target else pc_next.
    assign pc_src = (Branch & zero_flag) | is_jal | is_jalr;
    PC_MUX pc_mux(.pc_seq(pc_next), .pc_branch(chosen_target), .pc_select(pc_src), .pc_out(pc_out));

    wire [31:0] pc_plus4 = pc_next;

    wire [31:0] final_wb = (is_jal | is_jalr) ? pc_plus4 : wb_data;

    endmodule



module RISCV_Tb;

    // Clock and reset
    reg clk;
    reg rst;

    // Wires
    wire [31:0] PC;
    wire [31:0] Instruction;
    wire [31:0] Imm_Gen;

   
    RISCV_Top UUT(
        .clk(clk),
        .rst(rst)
       
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

    // Stimulus
    initial begin
        // Apply reset
        rst = 1;
        #20;
        rst = 0;

        // Run for some time then finish
        #800 $finish;
    end

    

endmodule

