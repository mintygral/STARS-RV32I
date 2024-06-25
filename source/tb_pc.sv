`timescale 1ms/1ps

module tb_pc();

    // Parameters
    localparam CLK_PERIOD = 10; // 100 MHz clock period
    localparam RESET_ACTIVE = 1;
    localparam RESET_INACTIVE = 0;

    // DUT Inputs
    logic tb_clk;
    logic tb_clr;
    logic tb_load;
    logic tb_inc;
    logic tb_ALU_out;
    logic tb_Disable;
    logic [31:0] tb_data;
    logic [31:0] tb_immediate_value;

    // DUT Outputs
    logic [31:0] tb_pc_val;

    // Expected values for checks
    logic [31:0] tb_pc_val_exp;

    //  // Signal Dump
    // initial begin
    //     $dumpfile("pc.vcd");
    //     $dumpvars(0, tb_pc);
    // end

    // DUT Instance
    pc dut (
        .clk(tb_clk),
        .clr(tb_clr),
        .load(tb_load),
        .inc(tb_inc),
        .ALU_out(tb_ALU_out),
        .Disable(tb_Disable),
        .data(tb_data),
        .imm_val(tb_immediate_value),
        .pc_val(tb_pc_val)
    );

    // Clock Generation
    always begin
        tb_clk = 1'b0;
        #(CLK_PERIOD / 2);
        tb_clk = 1'b1;
        #(CLK_PERIOD / 2);
    end

    // Quick reset for 2 clock cycles
    task reset_dut;
        begin
            @(negedge tb_clk); // synchronize to negedge edge so there are not hold or setup time violations
            
            // Activate reset
            tb_clr = RESET_ACTIVE;

            // Wait 2 clock cycles
            @(negedge tb_clk);
            @(negedge tb_clk);

            // Deactivate reset
            tb_clr = RESET_INACTIVE; 
        end
    endtask

    task check_pc_value(input logic [31:0] expected_value);
    begin
        tb_pc_val_exp = expected_value;
        if (tb_pc_val === tb_pc_val_exp) begin
            $display("PC Value Correct: %0d", tb_pc_val);
        end
        else begin
            $error("PC Value Incorrect: Actual: %0d, Expected: %0d", tb_pc_val, tb_pc_val_exp);
        end
    end
    endtask

    initial begin
        $dumpfile("pc.vcd");
        $dumpvars(0, tb_pc);
        // Initialize inputs
        tb_load = 1'b0;
        tb_inc = 1'b0;
        tb_ALU_out = 1'b0;
        tb_Disable = 1'b0;
        tb_data = 32'd0;
        tb_immediate_value = 32'd0;

        // Test 1: Fetch an instruction, verify that the counter has been incremented
        reset_dut();
        tb_inc = 1'b1;
        @(negedge tb_clk);
        check_pc_value(32'd4); // Expected value after first increment

        // Test 2: Fetch an instruction, verify that the counter points to the address of the next instruction
        tb_inc = 1'b1;
        @(negedge tb_clk);
        check_pc_value(32'd8); // Expected value after second increment


        // Test 3: Test asynchronous reset, verify reset
        tb_inc = 0;
        tb_clr = 1'b1;
        @(negedge tb_clk);
        check_pc_value(32'd0); // Expected value after reset
        tb_clr = 1'b0;

        // Test 4: Load a specific value and verify the counter points to that value
        reset_dut;
        tb_load = 1'b1;
        tb_data = 32'd20;
        @(negedge tb_clk);
        check_pc_value(32'd24); // Expected value after loading specific value

        // Test 5: Verify that if no instruction data is being fetched, the counter does not increment
        tb_inc = 1'b0; // Disable increment
        tb_Disable = 1'b1; // Disable fetching
        tb_data = 32'd0;
        @(negedge tb_clk);
        #(CLK_PERIOD * 3); // Wait for 3 clock cycles
        check_pc_value(32'd24); // Expected value should remain the same as no increment or fetch

        // Test 6: Verify branch operation with immediate_value > 4 and ALU_out = 1 
        reset_dut();
        tb_load = 1'b0;
        tb_Disable = 1'b0;
        reset_dut;
        tb_immediate_value = 32'd2;
        tb_ALU_out = 1'd1; 
        #(CLK_PERIOD);
        check_pc_value(32'd8); // Expected value after branch (2*4)
        #(CLK_PERIOD * 3);  


        $finish;
    end

endmodule
