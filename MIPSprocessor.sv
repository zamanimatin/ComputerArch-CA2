`timescale 1ns/1ns

module MIPSprocessor (input clk, rst, ldinpc, initpc);
    wire PCSrc, MemtoReg, MemRead, MemWrite, ALUSrc, RegWrite, RegWSrc, WriteSrc, RegDst, PCsignal, jumpSrc;
    wire [2:0] ALUop;
    wire [31:0] instruction;
    wire zeroflag;
    MIPSController CUunit(instruction[31:26], instruction[5:0], zeroflag, PCSrc, MemtoReg, MemRead, MemWrite, ALUSrc, RegWrite, RegWSrc, WriteSrc, RegDst, PCsignal, jumpSrc, ALUop);
    MIPSDatapath DPunit(clk, rst, ldinpc, initpc, jumpSrc, PCsignal, RegDst, WriteSrc, RegWSrc, RegWrite, ALUSrc, ALUop, MemRead, MemWrite, PCSrc, MemtoReg, instruction, zeroflag);
endmodule
