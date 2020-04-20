`timescale 1ns/1ns
module MUX32Bit (input [31:0]ZEROsel, ONEsel, input selector, output reg [31:0] out);
    always @(ZEROsel, ONEsel, selector) begin
        out = 32'b0;
        if(selector == 1'b0)
            out = ZEROsel;
        else if (selector == 1'b1)
            out = ONEsel;
    end
endmodule

module  MUX5Bit(input [4:0]ZEROsel, ONEsel, input selector, output reg [4:0] out);
    always @(ZEROsel, ONEsel, selector) begin
        out = 5'b0;
        if(selector == 1'b0)
            out = ZEROsel;
        else if (selector == 1'b1)
            out = ONEsel;
    end
endmodule

module  SignExtender(input [15:0]in, output reg [31:0] out);
    always @(in) begin
        out = 32'b0;
        out = 32'(signed'(in));
    end
endmodule

module ShiftLeft2bit(input [31:0]in, output reg [31:0]out);
    always @(in) begin
        out = 32'b0;
        out = {in[29:0], 2'b00};
    end
endmodule

module Adder32bit(input [31:0]A, B, output reg [31:0]out, output reg cout);
    always @(A, B) begin
        out = 32'b0;
        cout = 1'b0;
        {cout, out} = A + B;
    end
endmodule

module ALU32Bit (input [31:0]A, B, input [2:0]ALUop, output reg[31:0] ALUout, output reg ZeroFlag);
    always @(A, B, ALUop) begin
        ALUout = 32'b0;
        ZeroFlag = 1'b0;
        if (ALUop == 3'b000) // case AND
            ALUout = A & B;
        else if (ALUop == 3'b001) // case OR
            ALUout = A | B;
        else if (ALUop == 3'b010) // case ADD
            ALUout = A + B;
        else if (ALUop == 3'b110) // case SUB
            ALUout = A - B;
        else if (ALUop == 3'b111) // case SLT
            ALUout = A - B;
        if(ALUout == 32'b0)
            ZeroFlag = 1'b1;
    end
endmodule

module PC(input [31:0]in, input rst, clk, ldin, initPC, output reg[31:0]out);
    always @(posedge clk, posedge rst) begin
        if (rst)
            out <= 32'b0;
        else if (initPC)
            out <= 32'b0;
        else if (ldin)
            out <= in;
    end
endmodule

module ZEROExtender(input [25:0]in, output reg [27:0]out);
    always @(in) begin
        out = 28'b0000000000000000000000000000; // output rule for combinationals
        out = {in, 2'b00};
    end
endmodule

module RegFile (input [31:0]ReadReg1, ReadReg2, input [4:0]WriteReg, input [31:0]Writedata, input clk, rst, regWriteSignal, output reg [31:0]ReadData1, ReadData2);
    reg [31:0] REGFILE [0:31];

    // May a readmemh command needed
    always @(posedge clk, posedge rst)begin
        {ReadData1, ReadData2} = 64'b0;
        if (rst) begin
            for(integer i = 0; i < 32; i++) begin
                REGFILE[i] = 32'b0;
            end
        end
        if (regWriteSignal)
            REGFILE[WriteReg] = Writedata;
        else
            ReadData1 = REGFILE[ReadReg1];
            ReadData2 = REGFILE[ReadReg2];
    end
endmodule

module DataMemory (input [31:0]address, writedata, input MemRead, MemWrite, output reg [31:0]ReadData);
    reg [31:0] DMemory [0:16];
    initial begin
        $sreadmemh("DataMemory.data", DMemory);
    end
    always@(address, writedata, MemRead, MemWrite) begin
        ReadData = 32'b00000000000000000000000000000000;
        if (writedata)
            DMemory[address] = writedata;
        else if (MemRead)
            ReadData = DMemory[address];
    end
endmodule

module MIPSDatapath (input clk, rst, ldinpc, initpc, JumpSrc, PCsignal, RegDst, WriteSrc, RegWSrc, RegWrite, ALUSrc, ALUoperation, MemRead, MemWrite, PCSrc, MemtoReg, output reg [31:0]instruction, output reg zeroflag);
    wire [31:0] wire1, container4, addresswire, pcadderout, wire2, shl2outsext, Readdata1, Readdata2, sextout, mainALUout, DataMemReaddataout;
    wire [31:0] ALUMUXin, MemtoRegfile, jumpmuxin, jumpmuxout, pcin, instructionwire, wire3, writedatain;
    wire [27:0] shl2instruction;
    wire [4:0] writeregin, muxin31;
    wire zeroflagout;
    wire cout1, cout2;

    MUX32Bit MUX1(Readdata1, jumpmuxin, JumpSrc, jumpmuxout);
    MUX32Bit MUX2(wire1, jumpmuxout, PCsignal, pcin);
    PC ProgramCounter(pcin, rst, clk, ldinpc, initpc, addresswire);
    assign container4 = 32'b00000000000000000000000000000100; // 32bit 4 value
    Adder32bit AddressAdder1(container4, addresswire, pcadderout, cout1);
    Adder32bit AddressAdder2(pcadderout, shl2outsext, wire2, cout2);
    MUX32Bit MUX3(pcadderout, wire2, PCSrc, wire1);
    ShiftLeft2bit Shl2Sext(sextout, shl2outsext);
    ZEROExtender ZeroExt(instructionwire[25:0],shl2instruction);
    assign jumpmuxin = {addresswire[31:28],shl2instruction};
    SignExtender SEXT(instructionwire[15:0], sextout);
    MUX5Bit MUX4(instructionwire[20:16], instructionwire[15:11], RegDst, wire3);
    assign muxin31 = 5'b11111; // 31 value
    MUX5Bit MUX5(wire3, muxin31, RegWSrc, writeRegIn);
    MUX32Bit MUX6(MemtoRegfile, pcadderout, WriteSrc, writedatain);
    MUX32Bit MUX7(Readdata1, sextout, ALUSrc, ALUMUXin);
    ALU32Bit MainALU(Readdata1, ALUMUXin, ALUoperation, mainALUout, zeroflagout);
    MUX32Bit MUX8(mainALUout, DataMemReaddataout, MemtoReg, MemtoRegfile);
    RegFile MainRegFile(instructionwire[25:21], instructionwire[20:16], writeRegIn, writedatain, clk, rst, RegWrite, Readdata1, Readdata2);
    DataMemory MainDataMemory(mainALUout, Readdata2, MemRead, MemWrite, DataMemReaddataout);
    reg [31:0] InstructionMemory [0:15];
    initial begin
        $sreadmemh("instruction.data", InstructionMemory);
    end
endmodule