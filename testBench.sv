module ATB();

    reg clk = 1'b0;
    reg rst = 1'b1;

    always #100 clk = ~clk;



    MIPSprocessor  mips(clk, rst);

    initial begin
        #310 rst = 1'b0;
        #50000 $stop;
    end


endmodule