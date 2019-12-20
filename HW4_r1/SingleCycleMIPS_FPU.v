module SingleCycleMIPS( 
    clk,
    rst_n,
    IR_addr,
    IR,
    ReadDataMem,
    CEN,
    WEN,
    A,
    Data2Mem,
    OEN
);
//==== in/out declaration =================================
    //-------- processor ----------------------------------
    input         clk, rst_n;
    input  [31:0] IR;
    output [31:0] IR_addr;
    //-------- data memory --------------------------------
    input  [31:0] ReadDataMem;  
    output        CEN;  
    output        WEN;  
    output  [6:0] A;  
    output [31:0] Data2Mem;  
    output        OEN;  

//==== reg/wire declaration ===============================
// Instruction Decoding Wire (INT)
wire [5:0] OP;
wire [4:0] RS;
wire [4:0] RT;
wire [4:0] RD;
wire [4:0] SHAMT;
wire [31:0] SHAMT_EXT;
wire [5:0] FUNCT;
wire signed [15:0] IMME;
wire signed [31:0] IMME_EXT;
wire [25:0] ADDR;
// Instruction Decoding Wire (FP)
// Important!!!
wire [4:0] FMT;
wire [4:0] FT;
wire [4:0] FS;
wire [4:0] FD;
// Control Signals
wire Jump;
wire JumpReg;
wire StoreRA;
wire Branch;
wire RegDst;
wire RegWrite;
wire MemRead;
wire Mem2Reg;
wire MemWrite;
wire [1:0] ALUSrc;
wire [2:0] ALUOp;
// ALU Control Signal
wire [3:0] ALUCtrl;
// ALU Result
wire signed [31:0] ALUResult;
wire Zero;
// Important!!!
// ALUResultForDoubleLST is for the last 32 bits of ALUResult of Double FP ALU
wire [31:0] ALUResultForDoubleLST;
// Register (INT)
reg signed [31:0] REG [0:31];
reg signed [31:0] n_REG [0:31];
reg  [31:0] IR_addr;
wire [31:0] n_IR_addr;
wire [31:0] IR_addr_plus_4;
// Important!!!
// Register (FP)
reg signed [63:0] FPREG [0:31];
reg signed [63:0] n_FPREG [0:31];
reg  FPCond;
wire n_FPCond;
// data of read register
wire signed [31:0] ReadData1, ReadData2;
// data to be stored in memory
wire [31:0] Data2Mem;
// address of memory that data should be stored in
wire [6:0] A;
// control signals for memory
wire CEN;
wire WEN;
wire OEN;
// write register number
wire [4:0] WriteRegister;
// data that should be written into write register
wire [31:0] WriteData;
// Important!!!
// Floating Point Calculation Result
wire [31:0] ReadDataFPS1;
wire [31:0] ReadDataFPS2;
wire [31:0] ADDResultFPS;
wire [31:0] SUBResultFPS;
wire [31:0] MULResultFPS;
wire [31:0] DIVResultFPS;
wire [63:0] ReadDataFPD1;
wire [63:0] ReadDataFPD2;
wire [63:0] ADDResultFPD;
wire [63:0] SUBResultFPD;
wire [2:0]  RND; // for FP rounding
// STATUS are dummys
wire [7:0]  ADDSTATUS_S;
wire [7:0]  ADDSTATUS_D;
wire [7:0]  SUBSTATUS_S;
wire [7:0]  SUBSTATUS_D;
wire [7:0]  MULSTATUS;
wire [7:0]  DIVSTATUS;
// Important!!!
// FP Control Signals
wire FP; // FP = 1 iff FR-Type instruction or lwcl or ldcl or swcl or sdcl or bclt is presented
wire FPCondWrite; // FPCondWrite = 1 iff c.eq.s is presented
wire SingleOrDouble; // SingleOrDouble = 0 if single; otherwise, SingleOrDouble = 1 if double
reg  DoubleStage2; // For lwdl swdl, there should be two stage of memory access since there are 64 bits of data to transfer
wire n_DoubleStage2; 

integer num;
integer i;
//==== wire connection to submodule ======================
//assigning instruction decoding wire (INT)
assign OP = IR[31:26];
assign RS = IR[25:21];
assign RT = IR[20:16];
assign RD = IR[15:11];
assign SHAMT = IR[10:6];
assign FUNCT = IR[5:0];
assign IMME = IR[15:0];
assign ADDR = IR[25:0];
//Important!!!
//assigning instruction decoding wire (FP)
assign FMT = IR[25:21];
assign FT  = IR[20:16];
assign FS  = IR[15:11];
assign FD  = IR[10:6];
//Rounding type should be 0 (I asked TA for this part)
assign RND = 3'b000;
//
//Example:
//	ctrl control(
//	.clk(clk),
//	.rst_n(rst_n), ......

//	);

Ctrl control(
    .OP(OP),
    .FUNCT(FUNCT), 
    .DoubleStage2(DoubleStage2),
    .Jump(Jump), 
    .JumpReg(JumpReg),
    .StoreRA(StoreRA),
    .Branch(Branch), 
    .RegDst(RegDst), 
    .RegWrite(RegWrite),
    .MemRead(MemRead), 
    .Mem2Reg(Mem2Reg), 
    .MemWrite(MemWrite), 
    .FP(FP),
    .SingleOrDouble(SingleOrDouble),
    .n_DoubleStage2(n_DoubleStage2),
    .ALUSrc(ALUSrc), 
    .ALUOp(ALUOp),
    .CEN(CEN),
    .OEN(OEN), 
    .WEN(WEN)
    );

ALU alu(
    .ReadData1(ReadData1), 
    .ReadData2(ReadData2), 
    .SHAMT_EXT(SHAMT_EXT),
    .IMME_EXT(IMME_EXT),
    .FPCond(FPCond), 
    .ALUSrc(ALUSrc), 
    .ALUCtrl(ALUCtrl), 
    .ADDResultFPS(ADDResultFPS),
    .SUBResultFPS(SUBResultFPS),
    .MULResultFPS(MULResultFPS),
    .DIVResultFPS(DIVResultFPS),
    .ADDResultFPD(ADDResultFPD),
    .SUBResultFPD(SUBResultFPD),
    .ALUResult(ALUResult), 
    .ALUResultForDoubleLST(ALUResultForDoubleLST),
    .Zero(Zero)
);

ALUCtrl alucontrol(
    .ALUOp(ALUOp), 
    .FUNCT(FUNCT), 
    .ALUCtrl(ALUCtrl),
    .FPCondWrite(FPCondWrite)
);

IRCal PC(
    .IR_addr(IR_addr),
    .ADDR(ADDR), 
    .Jump(Jump), 
    .JumpReg(JumpReg), 
    .ReadData1(ReadData1), 
    .IMME_EXT(IMME_EXT), 
    .Branch(Branch), 
    .Zero(Zero), 
    .n_DoubleStage2(n_DoubleStage2),
    .IR_addr_plus_4(IR_addr_plus_4), 
    .n_IR_addr(n_IR_addr)
    );

SignExt sign_extension(
    .IMME(IMME), 
    .IMME_EXT(IMME_EXT)
    );

UnsignExt unsign_extension(
    .SHAMT(SHAMT), 
    .SHAMT_EXT(SHAMT_EXT)
    );
// FPU module
// Important!!!
DW_fp_add fp_adder_single(
    .a(ReadDataFPS1),
    .b(ReadDataFPS2),
    .rnd(RND),
    .z(ADDResultFPS),
    .status(ADDSTATUS_S)
);

DW_fp_sub fp_subtracter_single(
    .a(ReadDataFPS1),
    .b(ReadDataFPS2),
    .rnd(RND),
    .z(SUBResultFPS),
    .status(SUBSTATUS_S)
);

DW_fp_mult fp_multiplier_single(
    .a(ReadDataFPS1),
    .b(ReadDataFPS2),
    .rnd(RND),
    .z(MULResultFPS),
    .status(MULSTATUS)
);

DW_fp_div fp_divider_single(
    .a(ReadDataFPS1),
    .b(ReadDataFPS2),
    .rnd(RND),
    .z(DIVResultFPS),
    .status(DIVSTATUS)
);

DW_fp_add #(52,11,0)
fp_adder_double(
    .a(ReadDataFPD1),
    .b(ReadDataFPD2),
    .rnd(RND),
    .z(ADDResultFPD),
    .status(ADDSTATUS_D)
);

DW_fp_sub #(52,11,0)
fp_subtracter_double(
    .a(ReadDataFPS1),
    .b(ReadDataFPS2),
    .rnd(RND),
    .z(SUBResultFPD),
    .status(SUBSTATUS_D)
);
//==== combinational part =================================

assign ReadData1 = (OP == 0 && (FUNCT == 0 || FUNCT == 2))? REG[RT] : REG[RS];
assign ReadData2 = REG[RT];
// Important!!!
// Floating Point ReadData1 only depends on FS (lwcl lwdl swcl swdl are dependent of RS)
// Floating Point ReadData2 only depends on FT
assign ReadDataFPS1 = FPREG[FS];
assign ReadDataFPS2 = FPREG[FT];
assign ReadDataFPD1 = {FPREG[FS], FPREG[FS+1]};
assign ReadDataFPD2 = {FPREG[FT], FPREG[FT+1]};
// Important!!!
// We use RT for FPREG since swdl and swcl are I-Type (Not FI-Type)
assign Data2Mem  = (FP)? (DoubleStage2)? FPREG[RT+1] : FPREG[RT] : REG[RT];
// Important!!!
// If FP is 1, then we should write FPREG instead, so we should use FD instead 
// (We don't need FT since RT is used for lwdl and lwcl)
assign WriteRegister = (RegDst)? (FP)? FD : RD : (StoreRA)? 5'b11111 : RT;
assign WriteData = (Mem2Reg)? ReadDataMem : (StoreRA)? IR_addr_plus_4 : ALUResult;
assign A = (DoubleStage2)? ALUResult[8:2] + 1 : ALUResult[8:2];

always@(*)begin
    for(num = 0; num <= 31; num = num + 1)begin
    // FP has to be 0 for REG to be written
        if(num == WriteRegister && RegWrite == 1'b1 && FP == 1'b0)begin
            n_REG[num] = WriteData;
        end else begin
            n_REG[num] = REG[num];
        end
    end
    for(num = 0; num <= 31; num = num + 1)begin
    // FP has to be 1 and FPCondWrite has to be 0 for FPREG to be written
        if(num == WriteRegister && RegWrite == 1'b1 && FP == 1'b1 && FPCondWrite = 1'b0)begin
            n_FPREG[num] = WriteData;
        end else begin
            n_FPREG[num] = REG[num];
        end
    end
    if(FPCondWrite = 1'b1)begin
    // FPCondWrite has to be 1 for FPREG to be written
        n_FPCond = ALUResult[0];
    end else begin
        n_FPCond = FPCond;
    end
end

//==== sequential part ====================================

//Important!!!
//FPREG and FPCond should be blocked by flip-flop
always@(posedge clk)begin
    if(rst_n)begin
        for(i = 0; i <= 31; i = i + 1)begin
            REG[i] <= n_REG[i];
            FPREG[i] <= n_FPREG[i];
        end
        IR_addr <= n_IR_addr;
        FPCond <= n_FPCond;
        DoubleStage2 <= n_DoubleStage2;
    end else begin
        for(i = 0; i <= 31; i = i + 1)begin
            REG[i] <= 32'd0;
            FPREG[i] <= 32'd0;
        end
        IR_addr <= 32'd0;
        FPCond <= 1'b0;
        DoubleStage2 <= 1'b0;
    end
end

endmodule

// recommend you to use submodule for easier debugging 
//=========================================================
//Example:
//	module ctrl(
//		clk,
//		rst_n, ....


//	);





//	endmodule
// FP is 1 FRtype, lwcl, lwdl, swcl, swdl, bclt is presented (For INT instruction, FP == 0)
// SingleOrDouble = 1 if double; otherwise , SingleOrDouble = 0
// Use DoubleStage2 to calculate n_DoubleStage2
// Important!!!
// new ALUOp table
// Instruction type    |  ALUOp
// lw/sw/addi          |  000
// lwcl/lwdl/swdl/swcl |  000
// beq                 |  001
// R-Type              |  010
// bne                 |  011
// FR-type single      |  100
// FR-type double      |  101
// bclt                |  110
module Ctrl(OP, FUNCT, DoubleStage2, Jump, JumpReg, StoreRA, Branch, RegDst, RegWrite, MemRead, Mem2Reg, MemWrite, FP, SingleOrDouble, n_DoubleStage2, ALUSrc, ALUOp,CEN ,OEN, WEN);
    input  wire [5:0] OP;
    input  wire [5:0] FUNCT;
    input  wire DoubleStage2;
    output reg  Jump;
    output reg  JumpReg;
    output reg  StoreRA;
    output reg  Branch;
    output reg  RegDst;
    output reg  RegWrite;
    output reg  MemRead;
    output reg  Mem2Reg;
    output reg  MemWrite;
    output reg  FP;
    output reg  SingleOrDouble;
    output reg  n_DoubleStage2;
    output reg  [1:0] ALUSrc;
    output reg  [2:0] ALUOp;
    output reg  CEN;
    output reg  OEN;
    output reg  WEN;
    //default for op code 6bit zero
    parameter R_FORMAT = 6'h00;
    parameter FR_FORMAT = 6'h11;
    parameter ADDI = 6'h08;
    parameter BEQ = 6'h04;
    parameter BNE = 6'h05;
    parameter JMP = 6'h02;
    parameter JAL = 6'h03;
    parameter LW = 6'h23;
    parameter SW = 6'h2B;
    // FP Insturction
    parameter LW_S = 6'h31;
    parameter SW_S = 6'h39;
    parameter LW_D = 6'h35;
    parameter SW_D = 6'h3D;
    parameter SINGLE = 5'h10;
    parameter DOUBLE = 5'h11;
    parameter BCLT   = 5'h08;

    always@(*)begin
        case(OP)
        //r format
        R_FORMAT: begin
            RegDst = 1'b1;
            Jump = 1'b0;
            Branch = 1'b0;
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            //Jump register
            if(FUNCT == 6'h08) begin
                JumpReg = 1'b1;
                RegWrite = 1'b0;
            end else begin
                JumpReg = 1'b0;
                RegWrite = 1'b1;
            end
            FP = 1'b0;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            if(FUNCT == 6'h0 || FUNCT == 6'h2)
                //shammt ext
                ALUSrc = 2'b11;
            else
                //read data 2
                ALUSrc = 2'b00;
            ALUOp = 3'b010;
            CEN = 1'b1;
            //OEN WEN should be reverse
            //but since CEN is disabled
            //OEN WEN could be either 1 0 or 0 1
            OEN = 1'b0;
            WEN = 1'b1;
        end
        //fr format
        //Important!!!
        //including bclt
        FR_FORMAT: begin
            //RegDst = 1 for FD to be WriteRegister
            RegDst = 1'b1;
            Jump = 1'b0;
            Branch = 1'b0;
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            FP = 1'b1;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            // We don't care ALUSrc at this point
            // (use ADDRESULT_S, ADDRESULT_D, SUBRESULT_S... instead)
            ALUSrc = 2'b10; 
            if(FMT == SINGLE)begin
                RegWrite = 1'b1;
                SingleOrDouble = 1'b0;
                n_DoubleStage2 = 1'b0;
                ALUOp = 3'b100;
            end else if(FMT == DOUBLE)begin
                RegWrite = 1'b1;
                SingleOrDouble = 1'b1;
                // we can do FR-Type in one stage -> skip the seconf stage in order to prevent unnecessary stall.
                n_DoubleStage2 = 1'b0;
                ALUOp = 3'b101;
            end else if(FMT == BCLT)begin
                RegWrite = 1'b1;
                SingleOrDouble = 1'b0;
                n_DoubleStage2 = 1'b0;
                ALUOp = 3'b110;
            end else begin
                RegWrite = 1'bx;
                SingleOrDouble = 1'bx;
                n_DoubleStage2 = 1'bx;
                ALUOp = 3'bxxx;
            end
            CEN = 1'b1;
            //OEN WEN should be reverse
            //but since CEN is disabled
            //OEN WEN could be either 1 0 or 0 1
            OEN = 1'b0;
            WEN = 1'b1;
        end
        ADDI: begin
            //no rd
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b1;
            FP = 1'b0;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b01;
            ALUOp = 3'b000;
            CEN = 1'b1;
            //OEN WEN should be reverse
            //but since CEN is disabled
            //OEN WEN could be either 1 0 or 0 1
            OEN = 1'b0;
            WEN = 1'b1;     
        end
        BEQ: begin
            //RegDst is don't care
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b1;
            RegWrite = 1'b0;
            FP = 1'b0;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            MemRead = 1'b0;
            //Dont care
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b00;
            ALUOp = 3'b001;
            CEN = 1'b1;
            //OEN WEN should be reverse
            //but since CEN is disabled
            //OEN WEN could be either 1 0 or 0 1
            OEN = 1'b0;
            WEN = 1'b1; 
        end
        BNE: begin
            //RegDst is don't care
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b1;
            RegWrite = 1'b0;
            FP = 1'b0;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            MemRead = 1'b0;
            //Dont care
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b00;
            //ALUOP == 11 if BNE
            ALUOp = 3'b011;
            CEN = 1'b1;
            //OEN WEN should be reverse
            //but since CEN is disabled
            //OEN WEN could be either 1 0 or 0 1
            OEN = 1'b0;
            WEN = 1'b1; 
        end
        JMP: begin
            RegDst = 1'b0;
            Jump = 1'b1;
            Branch = 1'b0;
            RegWrite = 1'b0;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            FP = 1'b0;
            MemRead = 1'b0;
            //Dont care
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            //Dont care
            ALUSrc = 2'b00;
            ALUOp = 3'b000;
            CEN = 1'b1;
            //OEN WEN should be reverse
            //but since CEN is disabled
            //OEN WEN could be either 1 0 or 0 1
            OEN = 1'b0;
            WEN = 1'b1;
        end
        JAL: begin
            RegDst = 1'b0;
            Jump = 1'b1;
            Branch = 1'b0;
            //true to store $ra
            RegWrite = 1'b1;
            FP = 1'b0;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            StoreRA = 1'b1;
            MemRead = 1'b0;
            //Dont care
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            MemWrite = 1'b0;
            //Dont care
            ALUSrc = 2'b00;
            ALUOp = 3'b000;
            CEN = 1'b1;
            //OEN WEN should be reverse
            //but since CEN is disabled
            //OEN WEN could be either 1 0 or 0 1
            OEN = 1'b0;
            WEN = 1'b1;
        end
        LW: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b1;
            FP = 1'b0;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            MemRead = 1'b1;
            Mem2Reg = 1'b1;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b01;
            ALUOp = 3'b000;
            CEN = 1'b0;
            OEN = 1'b0;
            WEN = 1'b1; 
        end
        LW_S: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b1;
            FP = 1'b1;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            MemRead = 1'b1;
            Mem2Reg = 1'b1;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b01;
            ALUOp = 3'b000;
            CEN = 1'b0;
            OEN = 1'b0;
            WEN = 1'b1; 
        end
        LW_D: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b1;
            FP = 1'b1;
            SingleOrDouble = 1'b1;
            n_DoubleStage2 = ~DoubleStage2;
            MemRead = 1'b1;
            Mem2Reg = 1'b1;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b01;
            ALUOp = 3'b000;
            CEN = 1'b0;
            OEN = 1'b0;
            WEN = 1'b1; 
        end
        SW: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b0;
            FP = 1'b0;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b1;
            ALUSrc = 2'b01;
            ALUOp = 3'b000;
            CEN = 1'b0;
            OEN = 1'b1;
            WEN = 1'b0;
        end
        SW_S: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b0;
            FP = 1'b1;
            SingleOrDouble = 1'b0;
            n_DoubleStage2 = 1'b0;
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b1;
            ALUSrc = 2'b01;
            ALUOp = 3'b000;
            CEN = 1'b0;
            OEN = 1'b1;
            WEN = 1'b0;
        end
        SW_D: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b0;
            FP = 1'b1;
            SingleOrDouble = 1'b1;
            n_DoubleStage2 = ~DoubleStage2;
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b1;
            ALUSrc = 2'b01;
            ALUOp = 3'b000;
            CEN = 1'b0;
            OEN = 1'b1;
            WEN = 1'b0;
        end
        default: begin
            //default case
            RegDst = 1'bx;
            Jump = 1'bx;
            Branch = 1'bx;
            RegWrite = 1'bx;
            FP = 1'bx;
            SingleOrDouble = 1'bx;
            n_DoubleStage2 = 1'bx;
            MemRead = 1'bx;
            Mem2Reg = 1'bx;
            JumpReg = 1'bx;
            StoreRA = 1'bx;
            MemWrite = 1'bx;
            ALUSrc = 2'bxx;
            ALUOp = 3'bxxx;
            CEN = 1'bx;
            OEN = 1'bx;
            WEN = 1'bx;
        end
        endcase
    end
endmodule
//ALU
//Use ALUSrc to choose between ReadData2, IMME_EXT, SHAMT_EXT
//Use ALUCtrl to choose between different calculation(beq bne are 0110(subtract) and 0111, respectively.)
//Output ALUResult for the result of calculation.
//Output Zero = 1 for beq if ALUResult == 0 and for bne if ALUResult != 0
// Important!!!
// Needs ADDResultFPS, ADDResultFPD, SUBResultFPS, SUBResultFPD, MULResultFPS, DIVResultFPS for FP ALU
// Needs ALUResultForDoubleLST for the LST 32 bits of Double FP ALU
// If FPREG[FS] == FPREG[FT] and the instruction is c.eq.s, ALUResult = 1;
// Zero has different criterion for bclt
// remember to define ALUResultForDoubleLST for INT instruction
// TODO
module ALU(ReadData1, ReadData2, SHAMT_EXT, IMME_EXT, FPCond, ALUSrc, ALUCtrl, ADDResultFPS, SUBResultFPS, MULResultFPS, DIVResultFPS, ADDResultFPD, SUBResultFPD, ALUResult, ALUResultForDoubleLST, Zero);
    input wire signed [31:0] ReadData1;
    input wire signed [31:0] ReadData2;
    input        wire [31:0] SHAMT_EXT;
    input wire signed [31:0] IMME_EXT;
    input        wire FPCond;
    input        wire [1:0]  ALUSrc;
    input wire signed [3:0]  ALUCtrl;
    input        wire [31:0] ADDResultFPS;
    input        wire [31:0] SUBResultFPS;
    input        wire [31:0] MULResultFPS;
    input        wire [31:0] DIVResultFPS;
    input        wire [63:0] ADDResultFPD;
    input        wire [63:0] SUBResultFPD;
    output reg signed [31:0] ALUResult;
    output        reg [31:0] ALUResultForDoubleLST; // remember to define it for INT instruction
    output       wire Zero;
    reg        signed [31:0] SecondData;
    
    assign Zero = (ReadData1 == SecondData)? ((ALUCtrl[3]) ? 1'b0 : 1'b1) : ((ALUCtrl[3]) ? 1'b1 :1'b0);
    always@(*) begin
        case(ALUSrc)
            2'b00:
                SecondData = ReadData2;
            2'b01:
                SecondData = IMME_EXT;
            2'b11:
                SecondData = SHAMT_EXT;
            default:
                SecondData = 32'bX;
        endcase
        case(ALUCtrl)
            4'b0000: // and
                ALUResult = ReadData1 & SecondData;
            4'b0001: // or
                ALUResult = ReadData1 | SecondData;
            4'b0010: // add
                ALUResult = ReadData1 + SecondData;
            4'b0110: // sub
                ALUResult = ReadData1 - SecondData;
            4'b1110: // bne
                ALUResult = ReadData1 - SecondData;
            4'b0111: // slt
                if(ReadData1 - SecondData < 0)begin
                    ALUResult = 32'd1;
                end else begin
                    ALUResult = 32'd0;
                end
            4'b1100: // nor
                ALUResult = ~(ReadData1 | SecondData);
            4'b1111: // sll
                ALUResult = ReadData1 << SecondData;
            4'b1101: // srl
                ALUResult = ReadData1 >> SecondData;
            default:
                ALUResult = 32'bX;
        endcase
    end
endmodule

//ALU Control
//ALU control table
//Important!!!
//Instruction  |  ALUCtrl Value
//lw/sw/addi   |  0010 (add)
//lwcl/lwdl    |  0010 (add)
//swcl/swdl    |  0010 (add)
//beq          |  0110 (sub)
//bne          |  1110 (sub'n)

//and          |  0000 
//or           |  0001
//add          |  0010
//fp_add_single|  0011
//fp_sub_single|  0100
//bclt         |  0101
//sub          |  0110
//slt          |  0111
//fp_mul_single|  1000
//fp_div_single|  1001
//fp_add_double|  1010
//fp_sub_double|  1011
//fp_eq        |  1100
//srl          |  1101
//sub'n        |  1110
//sll          |  1111

// FPCondWrite = 1 if the instruction is c.eq.s
// TODO
module ALUCtrl(ALUOp, FUNCT, ALUCtrl, FPCondWrite);
    input wire [2:0] ALUOp;
    input wire [5:0] FUNCT;
    output reg [3:0] ALUCtrl;
    reg        [7:0] Concat;
    reg        FPCondWrite;
    always@(*) begin
        Concat = {ALUOp,FUNCT};
        casex(Concat)
            8'b00xxxxxx:
                //lw sw addi
                ALUCtrl = 4'b0010;
            8'b01xxxxxx:
                //beq
                ALUCtrl = 4'b0110;
            8'b10100000:
                //add
                ALUCtrl = 4'b0010;
            8'b10100010:
                //sub
                ALUCtrl = 4'b0110;
            8'b10100100:
                //and
                ALUCtrl = 4'b0000;
            8'b10100101:
                //or
                ALUCtrl = 4'b0001;
            8'b10101010:
                //slt
                ALUCtrl = 4'b0111;
            8'b10000000:
                //sll
                ALUCtrl = 4'b1111;
            8'b10000010:
                //srl
                ALUCtrl = 4'b1101;         
            8'b11xxxxxx: begin
                //bne
                ALUCtrl = 4'b1110;
            end
            default: begin
                //default case all zero
                ALUCtrl = 4'bXXXX;
            end
        endcase
    end

endmodule

//PC calculator
//New PC = PC + 4
//Use ADDR for jump address calculation. Use Jump in MUX.
//Use IMME_EXT and IR_addr + 4 for branch address calculation. Use Branch & Zero in MUX.
//Use ReadData1 for jump register address. Use JumpReg in MUX.
//Output n_IR_addr.
//Important!!!
//Remember to stall at the first stage of lwdl and swdl
//TODO
module IRCal(IR_addr, ADDR, Jump, JumpReg, ReadData1, IMME_EXT, Branch, Zero, n_DoubleStage2, IR_addr_plus_4, n_IR_addr);
    input wire [31:0] IR_addr;  
    input wire [25:0] ADDR;
    input wire [31:0] ReadData1;
    input wire Jump, JumpReg;
    input wire [31:0] IMME_EXT;
    input wire Branch, Zero;
    input wire n_DoubleStage2;
    output wire [31:0] IR_addr_plus_4; // for PC + 4
    output wire [31:0] n_IR_addr;
    
    wire [31:0] IMME_EXT_Shift_2; //for byte address of branch address
    wire [31:0] Jump_addr;  // jump address
    wire [31:0] Branch_addr; // Branch address = PC + 4 + IMME_EXT_Shift_2

    assign IR_addr_plus_4 = IR_addr + 4;
    assign IMME_EXT_Shift_2 = {IMME_EXT[29:0], 2'b00};
    assign Jump_addr = {IR_addr[31:28], ADDR, 2'b00};
    assign Branch_addr = IMME_EXT_Shift_2 + IR_addr_plus_4;
    assign n_IR_addr = (Jump | JumpReg)? ((Jump)? Jump_addr : ReadData1) : ((Branch & Zero)? Branch_addr : IR_addr_plus_4);
    

endmodule

//sign extension
module SignExt(IMME, IMME_EXT);
    input wire signed [15:0] IMME;
    output reg signed [31:0] IMME_EXT;
    integer i;

    always@(*)begin
        for(i = 0; i <= 31; i = i + 1)begin
            if(i >= 16)begin
                IMME_EXT[i] = IMME[15];
            end else begin
                IMME_EXT[i] = IMME[i];
            end
        end
    end

endmodule

//unsign extension
module UnsignExt(SHAMT, SHAMT_EXT);
    input wire [4:0] SHAMT;
    output reg [31:0] SHAMT_EXT;
    integer i;
    always@(*)begin
        for(i = 5; i <= 31; i = i + 1) begin
            SHAMT_EXT[i] = 1'b0;
        end
        SHAMT_EXT[4:0] = SHAMT;
    end
endmodule