`timescale 1ms/10ps

module tb_register_file;
    
    localparam CLK_PERIOD = 10;
    reg[31:0] writeData;
    reg[4:0] writeReg, readReg1, readReg2;
    logic clk, rst, writeEnable;
    logic [4:0] exp_read1, exp_read2;
    logic [31:0] regOut1, regOut2; //????
    integer tb_test_num;
    string tb_test_name;

    register_file regFile(.in(writeData), .clk(clk), .rst(rst), .write(writeEnable), .inAddress(writeReg), .outAddress1(readReg1), .outAddress2(readReg2));

    always begin
        clk = 0;
        #(CLK_PERIOD / 2);
        clk = 1;
        #(CLK_PERIOD / 2);
    end

    initial begin
        //CLK SIGNAL

        $dumpfile("dump.vcd");
        $dumpvars(0, tb_register_file);

        tb_test_num = -1;
        tb_test_name = "Test Bench Initialization";
        rst = 1'b0;
        writeEnable = 1'b0;
        //ADD OTHER VARIABLES

        #(0.5);
        
        //////////////////////////
        //Test 0: Power on Reset//
        //////////////////////////

        tb_test_num += 1;
        tb_test_name = "Power on reset";

        //set inputs to non-reset values
        writeReg = 5'b1;
        readReg1 = 5'b1;
        readReg2 = 5'b1;
        writeEnable = 1'b1;
        
        rst = 1'b0;

        #(CLK_PERIOD * 2)

        exp_read1 = 5'b0;
        exp_read2 = 5'b0;
        check_output(exp_read1, exp_read2);

        rst = 1'b1;

        //What are the expected outputs when back on
        exp_read1 = 5'b1; //???
        exp_read2 = 5'b1; //???
        check_output(exp_read1, exp_read2);
    end


    task check_output;
        input logic [4:0] exp_read1, exp_read2;
    begin
        @(negedge clk);
            if (exp_read1 == readReg1) begin
                $info("Correct exp_read1 value!");
            end else begin
                $error("Incorrect exp_read1 value :(. Actual: %0d, Expected: %0d.", readReg1, exp_read1);
            end
            if (exp_read2 == readReg2) begin
                $info("Correct exp_read2 value!");
            end else begin
                $error("Incorrect exp_read2 value :(. Actual: %0d, Expected: %0d.", readReg2, exp_read2);
            end
    end
    endtask

    task toggle_rst; 
        rst = 0; #10;
        rst = 1; #10;
        rst = 0; #10;
    endtask

endmodule