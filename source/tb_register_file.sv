`timescale 1ms/10ps

module tb_register_file;
    
    localparam CLK_PERIOD = 10;
    reg[31:0] writeData;
    reg[4:0] writeReg, readReg1, readReg2;
    logic clk, rst, writeEnable;
    logic [31:0] exp_read1, exp_read2;
    logic [31:0] regOut1, regOut2; //????
    integer tb_test_num;
    string tb_test_name;

    //add outputs!!!
    register_file regFile(.reg_write(writeData), .clk(clk), .rst(rst), .write(writeEnable), .rd(writeReg), .rs1(readReg1), .rs2(readReg2), .reg1(regOut1), .reg2(regOut2));

    //Toggle clock
    always begin
        clk = 0;
        #(CLK_PERIOD / 2);
        clk = 1;
        #(CLK_PERIOD / 2);
    end

    initial begin
        
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
        $info("Test 0: power on reset");

        tb_test_num += 1;
        tb_test_name = "Power on reset";
        toggle_rst;

        //set inputs to non-reset values
        writeReg = 5'b0;
        readReg1 = 5'b1;
        readReg2 = 5'b1;
        writeEnable = 1'b0;
        
        rst = 1'b1;

        #(CLK_PERIOD * 2);

        exp_read1 = 32'b0;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rst = 1'b0;

        //What are the expected outputs when back on
        exp_read1 = 32'b1; //???
        exp_read2 = 32'b1; //???
        check_output(exp_read1, exp_read2);

        //////////////////////////
        //Test 1: Test Writing  //
        //////////////////////////
        $info("Test 1: Test Writing");

        tb_test_num += 1;
        tb_test_name = "Test Writing";
        toggle_rst;

        writeReg = 5'b1;
        readReg1 = 5'b0;
        readReg2 = 5'b0;
        writeEnable = 1'b1;

        #(CLK_PERIOD * 2);

        //OUTPUTS????
        exp_read1 = 5'b0;
        exp_read2 = 5'b0;
        check_output(exp_read1, exp_read2);

        /////////////////////////////
        //Test 2: Nothing going on //
        /////////////////////////////
        $info("Test 2: Nothing going on");

        tb_test_num += 1;
        tb_test_name = "Nothing going on";
        toggle_rst;

        writeReg = 5'b0;
        readReg1 = 5'b0;
        readReg2 = 5'b0;
        writeEnable = 1'b0;

        #(CLK_PERIOD * 2);

        
        exp_read1 = 5'b0;
        exp_read2 = 5'b0;
        check_output(exp_read1, exp_read2);

        //////////////////////////////////////
        //Test 3: Test Reading/Sending value//
        //////////////////////////////////////
        $info("Test 3: Test Reading/Sending value");

        tb_test_num += 1;
        tb_test_name = "Test Reading/Sending Value";
        toggle_rst; 

        writeReg = 5'b0;
        readReg1 = 5'b1;
        readReg2 = 5'b1;
        writeEnable = 1'b0;

        #(CLK_PERIOD * 2);

        //OUYpUYS
        exp_read1 = 5'b1;
        exp_read2 = 5'b1;
        check_output(exp_read1, exp_read2);

    end


    task check_output;
        input logic [4:0] exp_read1, exp_read2;
    begin
        @(negedge clk);
            if (exp_read1 == regOut1) begin
                $info("Correct exp_read1 value!");
            end else begin
                $error("Incorrect exp_read1 value :(. Actual: %0d, Expected: %0d.", readReg1, exp_read1);
            end
            if (exp_read2 == regOut2) begin
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