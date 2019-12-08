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
wire [15:0] IMME;
wire [25:0] ADDR;
//Control Signals
wire Jump;
wire JumpReg;
wire JumpLink;
wire Branch;
wire RegDst;
wire RegWrite;
wire MemRead;
wire Mem2Reg;
wire MemWrite;
wire ALUSrc;
wire [1:0] ALUOp;
//
reg [31:0] REG [0:31];
reg [31:0] IR_addr, n_IR_addr;
reg [31:0] Data2Mem, n_Data2Mem;
reg [6:0] A, n_A;
reg CEN, n_CEN;
reg WEN, n_WEN;
reg OEN, n_OEN;

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
module ctrl(OP, Jump, JumpReg, JumpLink, Branch, RegDst, RegWrite, MemRead, Mem2Reg, MemWrite, ALUSrc, ALUOp);
    input  wire OP;
    output reg  Jump;
    output reg  JumpReg;
    output reg  JumpLink;
    output reg  Branch;
    output reg  RegDst;
    output reg  RegWrite;
    output reg  MemRead;
    output reg  Mem2Reg;
    output reg  MemWrite;
    output reg  ALUSrc;
    output reg  [1:0] ALUOp;

endmodule

module ALU(ReadData1, ReadData2, ALUOp, FUNCT)
		