`timescale 1ms/10ps
module tb;

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

    // Signal Dump
    initial begin
        $dumpfile ("dump.vcd");
        $dumpvars;
    end

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
            if (exp_state != state) $error("Incorrect state. Tested: %b. Should be: %b", exp_state, state);
            else $display("Coorect state. ested: %b. Should be: %b", exp_state, state);
            if (exp_add_out != address_out) $error("Incorrect address_out. Tested: %d. Should be: %d", exp_add_out, address_out);
            else $display("Correct address_out. Tested: %d. Should be: %h", exp_add_out, address_out);
            if (exp_dout_CPU != data_out_CPU) $error("Incorrect data_out_CPU. Tested: %d. Should be: %d", exp_dout_CPU , data_out_CPU);
            else $display("Correct data_out_CPU. Tested: %d. Should be: %d", exp_dout_CPU , data_out_CPU);
            if (exp_dout_BUS != data_out_BUS) $error("Incorrect data_out_BUS. Tested: %d. Should be: %d", exp_dout_BUS , data_out_BUS);
            else $display("Correct data_out_BUS. Tested: %d. Should be: %d", exp_dout_BUS , data_out_BUS);;
            if (exp_dout_INSTR != data_out_INSTR) $error("Incorrect data_out_INSTR. Tested: %d. Should be: %d", exp_dout_INSTR , data_out_INSTR);
            else $display("Correct.");
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
        $dumpvars(0, tb);
        
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
        tb_test_name = "Power on Reset";
        $display("Test %d: %s", tb_test_num, tb_test_name);

        // Set inputs to non-reset values
        stream_inputs(1, 1, 1, 1, 0, 0, 0, 1);

        // Activate reset
        rst = RESET_ACTIVE;

        #(CLK_PERIOD * 2); // Wait 2 clock periods before proceeding

        // Check that outputs are reset
        stream_outputs(INIT, 0, 0, 0, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);

        // Deactivate Reset
        rst = RESET_INACTIVE;

        stream_outputs(Read, 1, 1, 0, 0);
        check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);
        // NEED TO DO THIS FOR EVERY TEST CASEls

        // stream_inputs(add_in, d_in_CPU, d_in_BUS, data, instr, b_full, mWrite, mRead) // 8
        // stream_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR) // 5
        // check_outputs(exp_state, exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR);
    end


endmodule