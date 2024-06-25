`timescale 1ms/1ps

module tb_PC_ALU_integration();

    // Parameters
    localparam CLK_PERIOD = 10; // 100 MHz clock period

    // DUT Inputs
    logic tb_clk;
    logic tb_nRst;
    logic tb_load;
    logic tb_inc;
    logic tb_Disable;
    logic tb_ALU_source;
    logic tb_branch;
    logic [6:0] tb_opcode;
    logic [2:0] tb_funct3;
    logic [6:0] tb_funct7;
    logic [31:0] tb_data;
    logic [31:0] tb_imm_val;
    logic [31:0] tb_reg1;
    logic [31:0] tb_reg2;

    // DUT Outputs
    logic [31:0] tb_pc_val;
    logic [31:0] tb_read_ad;
    logic [31:0] tb_write_ad;
    logic [31:0] tb_result; 

    // Expected values for checks
    logic [31:0] tb_pc_val_exp;
    // logic [31:0] tb_read_ad_exp;
    // logic [31:0] tb_write_ad_exp;
    // logic [31:0] tb_result_exp;

     // Signal Dump
    initial begin
        $dumpfile("PC_ALU_integration.vcd");
        $dumpvars(0, tb_PC_ALU_integration);
    end

    // DUT Instance
    PC_ALU_integration dut (
        .clk(tb_clk),
        .nRst(tb_nRst),
        .load(tb_load),
        .inc(tb_inc),
        .ALU_source(tb_ALU_source),
        .Disable(tb_Disable),
        .branch(tb_branch),
        .opcode(tb_opcode),
        .funct3(tb_funct3),
        .funct7(tb_funct7),
        .data(tb_data),
        .imm_val(tb_imm_val),
        .reg1(tb_reg1),
        .reg2(tb_reg2),
        .pc_val(tb_pc_val),
        .read_ad(tb_read_ad),
        .write_ad(tb_write_ad),
        .result(tb_result)
    );

    // Clock Generation
    always begin
        tb_clk = 1'b0;
        #(CLK_PERIOD / 2);
        tb_clk = 1'b1;
        #(CLK_PERIOD / 2);
    end


    // Testbench Tasks
    task reset_dut;
    begin
        tb_nRst = 1'b0;
        // @(negedge tb_clk);
        @(negedge tb_clk);
        tb_nRst = 1'b1;
        @(negedge tb_clk);
        tb_imm_val = 32'd6;
    end
    endtask

    task check_pc_value(input logic [31:0] expected_pc_value);
    begin
        tb_pc_val_exp = expected_pc_value;
        if (tb_pc_val == tb_pc_val_exp) begin
            $display("PC Value Correct: %0d", tb_pc_val);
        end
        else begin
            $error("PC Value Incorrect: Actual: %0d, Expected: %0d", tb_pc_val, tb_pc_val_exp);
        end
    end
    endtask


    initial begin
        // Initialize inputs
        tb_nRst = 1'b0;
        tb_load = 1'b0;
        tb_inc = 1'b1;
        tb_Disable = 1'b0;
        tb_ALU_source = 1'b0;
        tb_opcode = 7'd0;
        tb_funct3 = 3'd0;
        tb_funct7 = 7'd0;
        tb_data = 32'd0;
        tb_imm_val = 32'd0;
        tb_reg1 = 32'd0;
        tb_reg2 = 32'd0;

        // Test 1: Fetch an instruction, verify that the counter has been incremented
        reset_dut();
       // tb_inc = 1'b1;
        //@(negedge tb_clk);
        check_pc_value(32'd4); // Expected value after first increment

        tb_inc = 1'b1;
        @(negedge tb_clk);
        check_pc_value(32'd8); // Expected value after second increment
        

        // Test 2: Test branch equal
        // reset_dut();
        tb_nRst = 1'b0;
        @(negedge tb_clk);
        tb_opcode = 7'b1100011;
        tb_reg1 = 32'd4;
        tb_reg2 = 32'd4;
        tb_funct3 = 3'd0;
        tb_imm_val = 32'd5;
        tb_nRst = 1'b1;
        tb_inc = 1'b1;  
        @(negedge tb_clk);
        check_pc_value(32'd20); // Expected value after reset

        // Test 3: Test branch not equal
        // reset_dut();
        tb_nRst = 1'b0;
        @(negedge tb_clk);
        tb_nRst = 1'b1;
        tb_reg1 = 32'd4;
        tb_reg2 = 32'd5;
        tb_funct3 = 3'd1;
        @(negedge tb_clk);
        check_pc_value(32'd20); // Expected value after reset

        // Test 4: Test branch less than
        // reset_dut();
        tb_nRst = 1'b0;
        @(negedge tb_clk);
        tb_nRst = 1'b1;
        tb_reg1 = 32'd12;
        tb_reg2 = 32'd5;
        tb_funct3 = 3'd4;
        @(negedge tb_clk);
        check_pc_value(32'd4); // Expected value after reset

        // Test 5: Test branch greater than and equal to 
        // reset_dut();
        tb_nRst = 1'b0;
        @(negedge tb_clk);
        tb_nRst = 1'b1;
        tb_reg1 = 32'd7;
        tb_reg2 = 32'd5;
        tb_funct3 = 3'd5;
        @(negedge tb_clk);
        check_pc_value(32'd20); // Expected value after reset

        $finish;
    end

endmodule