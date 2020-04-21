module ALUCU (input [5:0] functionCode, input [1:0]ALUop, output reg [2:0] ALUOperation);
    always @(functionCode, ALUop) begin
        ALUOperation = 3'b000;
        case(ALUop)
            2'b00: begin
                ALUOperation = 3'b010;
            end
            2'b01: begin
                ALUOperation = 3'b011;
            end
            2'b10: begin
                case(functionCode)
                    6'b100000: ALUOperation = 3'b010;
                    6'b100010: ALUOperation = 3'b110;
                    6'b100100: ALUOperation = 3'b000;
                    6'b100101: ALUOperation = 3'b001;
                    6'b101010: ALUOperation = 3'b111;
                    default: ALUOperation = 3'b000;
                endcase
            end
            default: begin
                ALUOperation = 3'b000;
            end
        endcase
    end
endmodule



module MIPSControllerMoshkeldar (input [5:0]opCode, functionCode, input zeroFlag, output reg PCSrc, MemtoReg, MemRead, MemWrite, ALUSrc, RegWrite, RegWSrc, WriteSrc, RegDst, PCsignal, jumpSrc, output reg [2:0]ALUOperation);
    reg [1:0] ALUop;

    ALUCU alu(functionCode, ALUop, ALUOperation);

    always @(opCode, functionCode, zeroFlag) begin
        {ALUop, PCSrc, MemtoReg, MemRead, MemWrite, ALUSrc, RegWrite, RegWSrc, WriteSrc, RegDst, PCsignal, jumpSrc} = 13'b0;
        case(opCode)
            6'b000000: {RegWrite, RegDst, ALUop} = 2'b1110;
            6'b000010: {PCsignal, jumpSrc} = 2'b11;
            6'b000011: {RegWrite, RegWSrc, WriteSrc, PCsignal, jumpSrc} = 4'b1111;
            6'b000100: {PCSrc, ALUop} = {zeroFlag, 2'b01};
            6'b000101: {PCSrc, ALUop} = {~zeroFlag, 2'b01};
            6'b001000: {ALUSrc, RegWrite} = 2'b11;
            6'b001100: {ALUSrc, RegWrite} = 2'b11;
            6'b100011: {MemtoReg, MemRead, ALUSrc, RegWrite} = 4'b1111;
            6'b101011: {MemWrite, ALUSrc} = 2'b11;
            6'b100000: PCsignal = 1'b1;
            default: {ALUop, PCSrc, MemtoReg, MemRead, MemWrite, ALUSrc, RegWrite, RegWSrc, WriteSrc, RegDst, PCsignal, jumpSrc} = 13'b0;
        endcase
    end
endmodule



module MIPSController (input [5:0]opCode, functionCode, input zeroFlag, output reg PCSrc, MemtoReg, MemRead, MemWrite, ALUSrc, RegWrite, RegWSrc, WriteSrc, RegDst, PCsignal, jumpSrc, output reg [2:0]ALUOperation);

    always @(opCode, functionCode, zeroFlag) begin
        {ALUOperation, PCSrc, MemtoReg, MemRead, MemWrite, ALUSrc, RegWrite, RegWSrc, WriteSrc, RegDst, PCsignal, jumpSrc} = 14'b0;
        case(opCode)
            6'b000000: begin
                {RegWrite, RegDst} = 2'b11;
                case (functionCode)
                    6'b100000: ALUOperation = 3'b010;
                    6'b100010: ALUOperation = 3'b110; // sub
                    6'b100100: ALUOperation = 3'b000;
                    6'b100101: ALUOperation = 3'b001;
                    6'b101010: ALUOperation = 3'b111;
                endcase
            end
            6'b000010: {PCsignal, jumpSrc} = 2'b11;
            6'b000011: {RegWrite, RegWSrc, WriteSrc, PCsignal, jumpSrc} = 4'b1111;
            6'b000100: {PCSrc, ALUOperation} = {zeroFlag, 3'b110};
            6'b000101: {PCSrc, ALUOperation} = {~zeroFlag, 3'b110};
            6'b001000: {ALUSrc, RegWrite, ALUOperation} = 5'b11010;
            6'b001100: {ALUSrc, RegWrite, ALUOperation} = 5'b11000;
            6'b100011: {MemtoReg, MemRead, ALUSrc, RegWrite, ALUOperation} = 7'b1111010;
            6'b101011: {MemWrite, ALUSrc, ALUOperation} = 5'b11010;
            6'b100000: PCsignal = 1'b1;
            default: {ALUOperation, PCSrc, MemtoReg, MemRead, MemWrite, ALUSrc, RegWrite, RegWSrc, WriteSrc, RegDst, PCsignal, jumpSrc} = 14'b0;
        endcase
    end
endmodule