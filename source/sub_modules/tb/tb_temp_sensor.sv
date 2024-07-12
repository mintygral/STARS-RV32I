`timescale 1ms/10ps

module tb_temp_sensor;
    
    // states logic
    typedef enum logic [2:0] {
        IDLE = 0,
        SKIP_ROM = 1,
        CONVERT_TEMP = 2,
        SKIP_ROM2 = 3,
        READ_SCRATCH = 4,
        READ = 5,
        RESET = 6
    } state_t;

    /////////////////////
    // Testbench Setup //
    /////////////////////

    localparam CLK_PERIOD = 10; // 100 MHz 
    localparam DIVIDE_CLK = 100 * CLK_PERIOD;
    localparam RESET_ACTIVE = 1;
    localparam RESET_INACTIVE = 0;

    // Testbench Signals
    integer tb_test_num = 0;
    string tb_test_name;

    // inputs
    logic clk, rst, read_request;
    // outputs
    logic out_wire;
    state_t state;
    logic [3:0] count_read; // read state
    logic [3:0] next_read;
    logic [2:0] count_index; // index
    logic [2:0] next_index; // every clk cycle
    logic [6:0] clk_divider; // writing state, when count_reg == 7

    // other variables
    // logic [3:0] count_read; // read state
    // logic [2:0] count_index; // index
    // logic [2:0] count_reg; // every clk cycle
    // logic [6:0] clk_divider; // writing state, when count_reg == 7
    state_t next_state, prev_state;

    logic [7:0] skip_rom = 8'hCC;
    logic [7:0] convert_t = 8'h44;
    logic [7:0] read_scratch = 8'hBE;

    logic strobe;

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

    // task for checking outputs
    task check_outputs ( 
            input state_t exp_state, input logic exp_wire
        );
        begin 
            @ (negedge clk)
            if (exp_state != state) $error("Incorrect state. Expected %b. Actual: %b", exp_state, state);
            else $display("Correct state Expected %b. Actual: %b", exp_state, state);
            if (exp_wire != out_wire) $error("Incorrect state. Expected %b. Actual: %b", exp_wire, out_wire);
            else $display("Correct state Expected %b. Actual: %b", exp_wire, out_wire);
        end

    endtask

    //////////
    // DUT //
    //////////
    temp_sensor DUT(
    // inputs
    .clk(clk),
    .rst(rst),
    .read_request(read_request),
    // outputs
    .out_wire(out_wire),
    .state(state),
    .count_read(count_read),
    .next_read(next_read),
    .count_index(count_index),
    .next_index(next_index),
    .clk_divider(clk_divider),
    .strobe(strobe)
    );

    // Clock generation block
    always begin
        clk = 0; // set clock initially to be 0 so that they are no time violations at the rising edge 
        #(CLK_PERIOD / 2);
        clk = 1;
        #(CLK_PERIOD / 2);
    end

    initial begin 
        $dumpfile("temp_sensor.vcd");
        $dumpvars(0, tb_temp_sensor);
        read_request = 0;
        #(0.5);

        ////////////////////////////
        // Test 1: Power on reset //
        ////////////////////////////

        // NOTE: Do not use reset task during reset test case 
        tb_test_num++;
        tb_test_name = "Power on Reset";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        read_request = 1;

        rst = RESET_ACTIVE;

        #(DIVIDE_CLK * 2);

        check_outputs(IDLE, 0);

        rst = RESET_INACTIVE;

        read_request = 1;

         #(DIVIDE_CLK * 70);

        $finish;
    end


endmodule