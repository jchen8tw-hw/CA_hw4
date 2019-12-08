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
//Instruction Decoding Wire
wire [5:0] OP;
wire [4:0] RS;
wire [4:0] RT;
wire [4:0] RD;
wire [4:0] SHAMT;
wire [5:0] FUNCT;
wire signed [15:0] IMME;
wire signed [31:0] IMME_EXT;
wire [25:0] ADDR;
//Control Signals
wire Jump;
wire JumpReg;
wire StoreRA;
wire Branch;
wire RegDst;
wire RegWrite;
wire MemRead;
wire Mem2Reg;
wire MemWrite;
wire ALUSrc;
wire [1:0] ALUOp;
// ALU Control Signal
wire [3:0] ALUCtrl;
// ALU Result
wire signed [31:0] ALUResult;
wire Zero;
// Register 
reg signed [31:0] REG [0:31];
reg signed [31:0] n_REG [0:31];
reg signed [31:0] ReadData1, ReadData2;
reg [31:0] IR_addr, n_IR_addr;
wire [31:0] Data2Mem;
wire [6:0] A;
wire CEN;
wire WEN;
wire OEN;

//==== wire connection to submodule ======================
//assigning instruction decoding wire
assign OP = IR[31:26];
assign RS = IR[25:21];
assign RT = IR[20:16];
assign RD = IR[15:11];
assign SHAMT = IR[10:6];
assign FUNCT = IR[5:0];
assign IMME = IR[15:0];
assign ADDR = IR[25:0];
//Example:
//	ctrl control(
//	.clk(clk),
//	.rst_n(rst_n), ......

//	);

//==== combinational part =================================

always@(*)begin





end

//==== sequential part ====================================
always@(posedge clk)begin
	

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
module Ctrl(OP, Jump, JumpReg, StoreRA, Branch, RegDst, RegWrite, MemRead, Mem2Reg, MemWrite, ALUSrc, ALUOp);
    input  wire OP;
    output reg  Jump;
    output reg  JumpReg;
    output reg  StoreRA;
    output reg  Branch;
    output reg  RegDst;
    output reg  RegWrite;
    output reg  MemRead;
    output reg  Mem2Reg;
    output reg  MemWrite;
    output reg  ALUSrc;
    output reg  [1:0] ALUOp;

endmodule

module ALU(ReadData1, ReadData2, SHAMT, ALUCtrl, ALUResult, Zero);
    input signed wire [31:0] ReadData1;
    input signed wire [31:0] ReadData2;
    input        wire [4:0] SHAMT;
    input signed wire [3:0]  ALUCtrl;
    output signed reg [31:0] ALUResult;
    output wire Zero;

endmodule

module ALUCtrl(ALUOp, FUNCT, ALUCtrl);
    input wire [1:0] ALUOp;
    input wire [5:0] FUNCT;
    output reg [3:0] ALUCtrl;

endmodule

//PC calculator
//Use ADDR for jump address
module IRCal(IR_addr, ADDR, Jump, JumpReg, IMME_EXT, Branch, Zero, n_IR_addr);
    input wire [31:0] IR_addr;
    input wire [25:0] ADDR;
    input wire Jump, JumpReg;
    input wire [31:0] IMME_EXT;
    input wire Branch, Zero;
    output reg [31:0] n_IR_addr;

endmodule

//sign extension
module SignExt(IMME, IMME_EXT);
    input wire [15:0] IMME;
    output wire [31:0] IMME_EXT;

endmodule

//Read Register 1, Read Register 2, Write Register Identification 
    //Use MUX to choose between RD, RT, and $ra (Remember to use RegDst and StoreRA in MUX!!!!)
//Write Data(n_REG) for Register. Use MUX to choose between ReadDataMem and ALUResult 
    //(Remember to use Mem2Reg in MUX!!!)
//Ouput Read Data 1, Read Data 2, n_REG (update for REG will be written in top module!!!)		
module NextRegister(REG, RS, RT, RD, RegDst, StoreRA, RegWrite, ReadDataMem, ALUResult, Mem2Reg, ReadData1, ReadData2, n_REG);
    input signed wire [31:0] REG [0:31];
    input        wire [4:0] RS;
    input        wire [4:0] RT;
    input        wire [4:0] RD;
    input        wire RegDst;
    input        wire StoreRA;
    input        wire RegWrite;
    input signed wire [31:0] ReadDataMem;
    input signed wire [31:0] ALUResult;
    input        wire Mem2Reg;
    output signed reg [31:0] ReadData1;
    output signed reg [31:0] ReadData2;
    output signed reg [31:0] n_REG [0:31];

endmodule