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
        else if (ALUop == 3'b111)begin // case SLT
            if ($signed(A) < $signed(B))  // supposed that A and B are signed check whether comparison happens right?
                ALUout = 32'b00000000000000000000000000000001;
            else
                ALUout = 32'b00000000000000000000000000000000;
        end
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

// CHECK REGISTER FILE AND DATA MEMORY
module RegFile (input [4:0]ReadReg1, ReadReg2, input [4:0]WriteReg, input [31:0]Writedata, input clk, rst, regWriteSignal, output reg [31:0]ReadData1, ReadData2);
    reg [31:0] REGFILE [0:31];

    always @(ReadReg1, ReadReg2) begin
        {ReadData1, ReadData2} = 64'b0;
        ReadData1 = REGFILE[ReadReg1];
        ReadData2 = REGFILE[ReadReg2];
    end
    always @(posedge clk, posedge rst)begin
        {ReadData1, ReadData2} = 64'b0;
        if (rst) begin
            for(integer i = 0; i < 32; i++) begin
                REGFILE[i] = 32'b0;
            end
        end
        if (regWriteSignal)
            REGFILE[WriteReg] = Writedata;
    end
endmodule

module DataMemory (input [31:0]address, writedata, input MemRead, MemWrite, clk, rst, output reg [31:0]ReadData);
    reg [31:0] DMemory [0:512];
    always @(negedge rst) begin
        $readmemb("DataMemory.mem", DMemory);
    end
    always @(address, MemRead) begin
        ReadData = 32'b00000000000000000000000000000000;
        if (MemRead)
            ReadData = DMemory[address[31:2]];
    end
    always@( posedge clk, posedge rst) begin
        if (rst)
            for(integer i = 0; i < 512; i++)begin
                DMemory[i] = 32'b00000000000000000000000000000000;
            end
        else if (MemWrite)
            DMemory[address[31:2]] = writedata;
    end
endmodule

module InstructionMemory(input rst, input [31:0]addressin, output reg [31:0]instruction);
    reg [31:0] instructionMemory [0:512];
    always @(negedge rst) begin
        $readmemb("instructionb.mem", instructionMemory);
    end
    always @(posedge rst, addressin)begin
        instruction = 32'b00000000000000000000000000000000;
        if(rst)
            for(integer i = 0; i < 512; i++)begin
                instructionMemory[i] = 32'b00000000000000000000000000000000;
            end
        else
            instruction = instructionMemory[addressin[31:2]];
    end
endmodule


module MIPSDatapath (input [2:0]ALUoperation, input clk, rst, ldinpc, initpc, JumpSrc, PCsignal, RegDst, WriteSrc, RegWSrc, RegWrite, ALUSrc, MemRead, MemWrite, PCSrc, MemtoReg, output [31:0]instructionwire, output zeroflag);
    wire [31:0] wire1, container4, addresswire, pcadderout, wire2, shl2outsext, Readdata1, Readdata2, sextout, mainALUout, DataMemReaddataout;
    wire [31:0] ALUMUXin, MemtoRegfile, jumpmuxin, jumpmuxout, pcin, writedatain;
    wire [27:0] shl2instruction;
    wire [4:0] writeRegIn, muxin31, wire3;
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
    MUX32Bit MUX7(Readdata2, sextout, ALUSrc, ALUMUXin);
    ALU32Bit MainALU(Readdata1, ALUMUXin, ALUoperation, mainALUout, zeroflag);
    MUX32Bit MUX8(mainALUout, DataMemReaddataout, MemtoReg, MemtoRegfile);
    RegFile MainRegFile(instructionwire[25:21], instructionwire[20:16], writeRegIn, writedatain, clk, rst, RegWrite, Readdata1, Readdata2);
    DataMemory MainDataMemory(mainALUout, Readdata2, MemRead, MemWrite, clk, rst, DataMemReaddataout);
    InstructionMemory MainInstrMemory(rst, addresswire, instructionwire);
endmodule