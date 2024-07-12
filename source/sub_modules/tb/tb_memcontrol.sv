`timescale 1ms/10ps
module tb_memcontrol;

    // states logic
    typedef enum logic [2:0] {
        INIT = 0,
        IDLE = 1,
        Read_Request = 2,
        Write_Request = 3,
        Read = 4,
        Write = 5,
        Wait = 6
    } state_t;

    /////////////////////
    // Testbench Setup //
    /////////////////////

    localparam CLK_PERIOD = 10; // 100 MHz 
    localparam RESET_ACTIVE = 1;
    localparam RESET_INACTIVE = 0;

    // Testbench Signals
    integer tb_test_num;
    string tb_test_name; 

    // DUT inputs
    logic [31:0] address_in, data_in_CPU, data_in_BUS;
    logic data_en, instr_en, bus_full, memWrite, memRead;
    logic clk, rst;

    // DUT outputs
    state_t state;
    logic [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR;

    // Expected outputs
    state_t exp_state;
    logic [31:0] exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR;

    // // Signal Dump
    // initial begin
    //     $dumpfile ("sim.vcd");
    //     $dumpvars(0, tb_memcontrol)
    // end

    ////////////////////////
    // Testbenching tasks //
    ////////////////////////

    // Quick reset for 2 clock cycles
    task reset_dut;
    begin
        @(negedge clk); // synchronize to negedge edge so there are not hold or setup time violations
        
        // Activate reset
        rst = RESET_ACTIVE;

        // Wait 2 clock cycles
        @(negedge clk);
        @(negedge clk);

        // Deactivate reset
        rst = RESET_INACTIVE; 
    end
    endtask

    // task for checking test outputs against actual expected values
    task check_outputs(
        input state_t exp_state,
        input logic [31:0] exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR
    );
        begin 
            @ (negedge clk);
            if (exp_state != state) $error("Incorrect state. Expected %b. Actual: %b", exp_state, state);
            else $display("Correct state Expected %b. Actual: %b", exp_state, state);
            if (exp_add_out != address_out) $error("Incorrect address_out. Expected %d. Actual: %d", exp_add_out, address_out);
            else $display("Correct address_out. Expected %d. Actual: %d", exp_add_out, address_out);
            if (exp_dout_CPU != data_out_CPU) $error("Incorrect data_out_CPU. Expected %d. Actual: %d", exp_dout_CPU , data_out_CPU);
            else $display("Correct data_out_CPU. Expected %d. Actual: %d", exp_dout_CPU , data_out_CPU);
            if (exp_dout_BUS != data_out_BUS) $error("Incorrect data_out_BUS. Expected %d. Actual: %d", exp_dout_BUS , data_out_BUS);
            else $display("Correct data_out_BUS. Expected %d. Actual: %d", exp_dout_BUS , data_out_BUS);;
            if (exp_dout_INSTR != data_out_INSTR) $error("Incorrect data_out_INSTR. Expected %d. Actual: %d", exp_dout_INSTR , data_out_INSTR);
            else $display("Correct data_out_INSTR. Expected %d. Actual: %d", exp_dout_INSTR , data_out_INSTR);
        end
    endtask

    // task for sending in inputs to memcontrol
    task stream_inputs(
        input logic [31:0] add_in, d_in_CPU, d_in_BUS,
        input logic data, instr, b_full, mWrite, mRead
        );

        begin 
            address_in = add_in;
            data_in_CPU = d_in_CPU;
            data_in_BUS = d_in_BUS;
            data_en = data;
            instr_en = instr;
            bus_full = b_full;
            memWrite = mWrite;
            memRead = mRead;
        end 

    endtask

    // task for assigning expected outputs
    task stream_outputs(
        input state_t test_exp_state,
        input logic [31:0] test_exp_add_out, test_exp_dout_CPU, test_exp_dout_BUS, test_exp_dout_INSTR
        );

        begin
            exp_state = test_exp_state;
            exp_add_out = test_exp_add_out;
            exp_dout_CPU = test_exp_dout_CPU;
            exp_dout_BUS = test_exp_dout_BUS; 
            exp_dout_INSTR = test_exp_dout_INSTR;
        end 

    endtask

    //////////
    // DUT //
    //////////

    // instantiate memcontrol module
    memcontrol DUT (
        .address_in(address_in), 
        .data_in_CPU(data_in_CPU),
        .data_in_BUS(data_in_BUS),
        .data_en(data_en),
        .instr_en(instr_en),
        .bus_full(bus_full),
        .memWrite(memWrite),
        .memRead(memRead),
        .clk(clk), 
        .rst(rst),
        // outputs
        .state(state),
        .address_out(address_out),
        .data_out_CPU(data_out_CPU),
        .data_out_BUS(data_out_BUS),
        .data_out_INSTR(data_out_INSTR)
        );

    // Clock generation block
    always begin
        clk = 0; // set clock initially to be 0 so that they are no time violations at the rising edge 
        #(CLK_PERIOD / 2);
        clk = 1;
        #(CLK_PERIOD / 2);
    end

    initial begin 
        $dumpfile("sim.vcd");
        $dumpvars(0, tb_memcontrol);
        
        // Initialize all test inputs
        tb_test_num = 0; // We haven't started testing yet
        tb_test_name = "Test Bench Initialization";
        stream_inputs(0, 0, 0, 0, 0, 0, 0, 0); // total 8 inputs
        #(0.5); // Wait some time before starting the first test case

        ////////////////////////////
        // Test 1: Power on reset //
        ////////////////////////////

        // NOTE: Do not use reset task during reset test case 
        tb_test_num+=1;
        tb_test_name = "Power on Reset, then Reading (dmem high)";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        // Set inputs to non-reset values
        stream_inputs(1, 1, 1, 1, 0, 0, 0, 1);

        // Activate reset
        rst = RESET_ACTIVE;

        #(CLK_PERIOD * 2); // Wait 2 clock periods before proceeding

        // Check that outputs are reset
        stream_outputs(INIT, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD); // In the INIT state these are all garbage values
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        // Deactivate Reset
        rst = RESET_INACTIVE;

        // Set inputs to non-reset values
        stream_inputs(1, 1, 1, 1, 0, 0, 0, 1);
        @(posedge clk);
        #(CLK_PERIOD * 2); // wait one clock period to transition by one state
        stream_outputs(Read, 1, 1, 0, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        // check if outputs are reset to garbage -- WORKS
        #(CLK_PERIOD); // should move back to idle
        stream_outputs(IDLE, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        // Format for EVERY test case
        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead) // 8
        // stream_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR) // 5
        // check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        ////////////////////////////
        // Test 2: Check Write    //
        ////////////////////////////
        reset_dut();
        tb_test_num+=1;
        tb_test_name = "Check Write capabilities (dmem high)";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        
        // Check Idle state
        stream_outputs(IDLE, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        // Send in inputs
        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead)
        stream_inputs(1, 1, 1, 1, 0, 0, 1, 0);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Write, 1, 0, 1, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR); // WORKS

        /////////////////////////////////////
        // Test 3: Bus_Full remains high   //
        /////////////////////////////////////
        reset_dut();
        tb_test_num+=1;
        tb_test_name = "Check Bus_Full remaining high";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        // Send in inputs
        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead)
        stream_inputs(1, 1, 1, 1, 0, 1, 1, 0);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Write_Request, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Wait, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Wait, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR); // WORKS

        /////////////////////////////////////
        // Test 4: Read vs. Write prec.    //
        /////////////////////////////////////
        reset_dut();
        tb_test_num+=1;
        tb_test_name = "Check Read vs. Write precedence";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        // Send in inputs
        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead)
        stream_inputs(1, 1, 1, 1, 0, 0, 1, 1);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Read_Request, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        @(posedge clk);
        // #(CLK_PERIOD);
        stream_outputs(Read, 1, 1, 0, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        /////////////////////////////////
        // Test 5: Dmem precedence     //
        /////////////////////////////////
        reset_dut();
        tb_test_num+=1;
        tb_test_name = "Check Dmem precedence";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        // This is the same as the previous test case except instr also = 1
        // Send in inputs
        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead)
        stream_inputs(1, 1, 1, 1, 1, 0, 1, 1);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Read_Request, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        @(posedge clk);
        // #(CLK_PERIOD);
        stream_outputs(Read, 1, 1, 0, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        /////////////////////////////////////
        // Test 6: MemWrite = MemRead = 0  //
        /////////////////////////////////////
        reset_dut();
        tb_test_num+=1;
        tb_test_name = "Check MemWrite and MemRead = 0";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead)
        stream_inputs(1, 1, 1, 0, 0, 0, 0, 0);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(IDLE, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        // make sure it stays in IDLE
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(IDLE, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        /////////////////////////////////////
        // Test 7: Read with instr_en high //
        /////////////////////////////////////
        reset_dut();
        tb_test_num+=1;
        tb_test_name = "Read with instr_en high";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead)
        stream_inputs(1, 1, 1, 0, 1, 0, 0, 1);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Read_Request, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        stream_outputs(Read, 1, 0, 0, 1);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        //////////////////////////////////////
        // Test 8: Write with instr_en high //
        //////////////////////////////////////
        reset_dut();
        tb_test_num+=1;
        tb_test_name = "Write with instr_en high";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead)
        stream_inputs(1, 1, 1, 0, 1, 0, 1, 0);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Write_Request, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        stream_outputs(Write, 1, 0, 1, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        /////////////////////////////////////////////////////
        // Test 9: Test Bus_Full then empty with instr_en  //
        /////////////////////////////////////////////////////

        //////////////////////////////////////////////
        // Test 10: Test mRead then mWrite  (dMem)  //
        /////////////////////////////////////////////
        reset_dut();
        tb_test_num+=1;
        tb_test_name = "Read then write with data_en high";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        stream_inputs(1, 1, 1, 1, 0, 0, 0, 1);
        @(posedge clk);
        #(CLK_PERIOD); // wait one clock period to transition by one state
        stream_outputs(Read_Request, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        stream_outputs(Read, 1, 1, 0, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        // Write
        stream_inputs(1, 1, 1, 1, 0, 0, 1, 0);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Write_Request, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        stream_outputs(Write, 1, 0, 1, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR); // WORKS

        //////////////////////////////////////////////
        // Test 11: Test mRead then mWrite  (iMem) //
        /////////////////////////////////////////////
        reset_dut();
        tb_test_num+=1;
        tb_test_name = "Read then write with instr_en high";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead)
        stream_inputs(1, 1, 1, 0, 1, 0, 0, 1);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Read_Request, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        stream_outputs(Read, 1, 0, 0, 1);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        stream_inputs(1, 1, 1, 0, 1, 0, 1, 0);
        @(posedge clk);
        #(CLK_PERIOD);
        stream_outputs(Write_Request, 32'hABCD, 32'hABCD, 32'hABCD, 32'hABCD);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        stream_outputs(Write, 1, 0, 1, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);
        $finish;

        
    end


endmodule
