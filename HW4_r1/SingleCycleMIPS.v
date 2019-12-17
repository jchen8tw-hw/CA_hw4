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
wire [31:0] SHAMT_EXT;
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
wire [1:0] ALUSrc;
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
module Ctrl(OP, FUNCT, Jump, JumpReg, StoreRA, Branch, RegDst, RegWrite, MemRead, Mem2Reg, MemWrite, ALUSrc, ALUOp,CEN ,OEN, WEN);
    input  wire OP;
    input  wire [5:0] FUNCT;
    output reg  Jump;
    output reg  JumpReg;
    output reg  StoreRA;
    output reg  Branch;
    output reg  RegDst;
    output reg  RegWrite;
    output reg  MemRead;
    output reg  Mem2Reg;
    output reg  MemWrite;
    output reg  [1:0] ALUSrc;
    output reg  [1:0] ALUOp;
    output reg CEN;
    output reg OEN;
    output reg WEN;
    //default for op code 6bit zero
    parameter R_FORMAT = 6'h0;
    parameter ADDI = 6'h8;
    parameter BEQ = 6'h4;
    parameter BNE = 6'h5;
    parameter JMP = 6'h2;
    parameter JAL = 6'h3;
    parameter LW = 6'h23;
    parameter SW = 6'h2B;
    //FP not implement yet
    always@(*)begin
        case(OP)
        //r format
        R_FORMAT: begin
            RegDst = 1'b1;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b1;
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            //Jump register
            if(FUNCT == 6'h8)
                JumpReg = 1'b1;
            else 
                JumpReg = 1'b0;
            
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            if(FUNCT == 6'h0 || FUNCT == 6'h2)
                //shammt ext
                ALUSrc = 2'b11;
            else
                //read data 2
                ALUSrc = 2'b00;
            ALUOp = 2'b10;
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
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b01;
            ALUOp = 2'b10;
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
            MemRead = 1'b0;
            //Dont care
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b00;
            ALUOp = 2'b01;
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
            MemRead = 1'b0;
            //Dont care
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b00;
            //ALUOP == 11 if BNE
            ALUOp = 2'b11;
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
            MemRead = 1'b0;
            //Dont care
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            //Dont care
            ALUSrc = 2'b00;
            ALUOp = 2'b00;
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
            StoreRA = 1'b1;
            MemRead = 1'b0;
            //Dont care
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            MemWrite = 1'b0;
            //Dont care
            ALUSrc = 2'b00;
            ALUOp = 2'b00;
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
            MemRead = 1'b0;
            Mem2Reg = 1'b1;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b01;
            ALUOp = 2'b00;
            CEN = 1'b0;
            OEN = 1'b1;
            WEN = 1'b0; 
        end
        SW: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b0;
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b1;
            ALUSrc = 2'b01;
            ALUOp = 2'b00;
            CEN = 1'b0;
            OEN = 1'b1;
            WEN = 1'b0;
        end
        default: begin
            //default case
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            RegWrite = 1'b0;
            MemRead = 1'b0;
            Mem2Reg = 1'b0;
            JumpReg = 1'b0;
            StoreRA = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 2'b00;
            ALUOp = 2'b00;
            CEN = 1'b1;
            OEN = 1'b1;
            WEN = 1'b0;
        end
        endcase
    end
    

endmodule

//ALU
//                             00          01         11
//Use ALUSrc to choose between ReadData2, IMME_EXT, SHAMT_EXT
//Use ALUCtrl to choose between different calculation(beq bne are 0110(subtract) and 0111, respectively.)
//Output ALUResult for the result of calculation.
//Output Zero = 1 for beq if ALUResult == 0 and for bne if ALUResult != 0
module ALU(ReadData1, ReadData2, SHAMT_EXT, IMME_EXT, ALUSrc, ALUCtrl, ALUResult, Zero);
    input signed wire [31:0] ReadData1;
    input signed wire [31:0] ReadData2;
    input        wire [31:0] SHAMT_EXT;
    input signed wire [31:0] IMME_EXT;
    input        wire [1:0] ALUSrc;
    input signed wire [3:0]  ALUCtrl;
    output signed reg [31:0] ALUResult;
    output wire Zero;
    always@(*) begin
        wire [31:0]SecondData;
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
            4'b0000:
                ALUResult = ReadData1 & SecondData;
            4'b0001:
                ALUResult = ReadData1 | SecondData;
            4'b0010:
                ALUResult = ReadData1 + SecondData;
            4'b0110:
                ALUResult = ReadData1 - SecondData;
            4'b0111:
                ALUResult = ReadData1 - SecondData;
            4'b1100:
                ALUResult = ~(ReadData1 | SecondData);
            4'b1111:
                ALUResult = ReadData1 << SecondData;
            4'b1110:
                ALUResult = ReadData1 >> SecondData;
            default:
                ALUResult = 32'bX;
        endcase
        if(ReadData1 - SecondData == 0)
            Zero = (ALUCtrl[4]) ? 1'b0 :1'b1;
        else
            Zero = (ALUCtrl[4]) ? 1'b1 :1'b0;
    end
endmodule

//ALU Control
//ALUOP == 11 if BNE == 01 if BEQ
//ALUCtrl  =  0110(subtract) and 0111,for beq bne respectively.
//ALUCtrl = 1111(sll) 1110(srl)
module ALUCtrl(ALUOp, FUNCT, ALUCtrl);
    input wire [1:0] ALUOp;
    input wire [5:0] FUNCT;
    output reg [3:0] ALUCtrl;
    always@(*) begin
        case(ALUOP)
            2'b00:
                ALUCtrl = 4'b0010;
            2'b01:
                ALUCtrl = 4'b0110;
            2'b10: begin
                case(FUNCT)
                    6'b100000:
                        ALUCtrl = 4'b0010;
                    6'b100010:
                        ALUCtrl = 4'b0110;
                    6'b100100:
                        ALUCtrl = 4'b0000;
                    6'b100101:
                        ALUCtrl = 4'b0001;
                    6'b101010:
                        ALUCtrl = 4'b0111;
                    6'h0: begin
                    //shift left logical
                        ALUCtrl = 4'b1111;
                    end
                    6'h2:begin
                    ////shift right logical
                        ALUCtrl = 4'b1110;
                    end
                    default: begin
                        //default case
                        ALUCtrl = 4'bXXXX;
                    end
                endcase
            end
            2'b11: begin
                //bne
                ALUCtrl = 4'b0111;
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
module IRCal(IR_addr, ADDR, Jump, JumpReg, ReadData1, IMME_EXT, Branch, Zero, n_IR_addr);
    input wire [31:0] IR_addr;
    input wire [25:0] ADDR;
    input wire [31:0] ReadData1;
    input wire Jump, JumpReg;
    input wire [31:0] IMME_EXT;
    input wire Branch, Zero;
    output reg [31:0] n_IR_addr;

endmodule

//sign extension
module SignExt(IMME, IMME_EXT);
    input signed wire [15:0] IMME;
    output signed wire [31:0] IMME_EXT;

endmodule

//unsign extension
module UnsignExt(SHAMT, SHAMT_EXT);
    input wire [4:0] SHAMT;
    output wire [31:0] SHAMT_EXT;
    for(i=5;i<=31;i=i+1)
        SHAMT_EXT[i] = 1'b0;
    SHAMT_EXT[4:0] = SHAMT;
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