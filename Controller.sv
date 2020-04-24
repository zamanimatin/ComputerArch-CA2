module MIPSController (input [5:0]opCode, functionCode, input zeroFlag, output reg PCSrc, MemtoReg, MemRead, MemWrite, ALUSrc, RegWrite, RegWSrc, WriteSrc, RegDst, PCsignal, jumpSrc, output reg [2:0]ALUOperation);

    initial begin
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