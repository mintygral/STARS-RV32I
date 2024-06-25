module tb_instruction_fetch;

    localparam CLK_PERIOD = 10;
    localparam RESET_ACTIVE = 1;
    localparam RESET_INACTIVE = 0;

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

    logic [31:0] data_in_CPU, data_in_BUS;
    logic data_en, bus_full, memWrite;
    logic clk, rst;

    logic [31:0] instruction_adr_i;

    state_t state;
    logic [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR;
    logic [31:0] instruction_o;

    // Expected outputs
    state_t exp_state;
    logic [31:0] exp_add_out, exp_dout_CPU, exp_dout_BUS, exp_dout_INSTR;
    logic exp_bus_full_CPU;
    logic [31:0] exp_instruction_o;

    instruction_fetch grab_instr(
        .data_in_CPU(data_in_CPU),
        .data_in_BUS(data_in_BUS),
        .data_en(data_en),
        .bus_full(bus_full),
        .memWrite(memWrite),
        .clk(clk),
        .rst(rst),
        .instruction_adr_i(instruction_adr_i),
        .state(state),
        .address_out(address_out),
        .data_out_CPU(data_out_CPU),
        .data_out_BUS(data_out_BUS),
        .data_out_INSTR(data_out_INSTR),
        .instruction_o(instruction_o)
    );


    always begin
        clk = 1'b0;
        #(CLK_PERIOD/2);
        clk = 1'b1;
        #(CLK_PERIOD/2);
    end

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

    task stream_inputs(
        input logic [31:0] d_in_BUS, instr_adr_i,
        input logic b_full
        );

        begin
            data_in_CPU = 32'b0;
            data_in_BUS = d_in_BUS;
            instruction_adr_i = instr_adr_i;
            data_en = 1'b0;
            bus_full = b_full;
            memWrite = 1'b0;
        end 

    endtask

    task stream_outputs(
        input state_t test_exp_state,
        input logic [31:0] test_exp_add_out, test_exp_dout_CPU, test_exp_dout_BUS, test_exp_dout_INSTR, exp_instr
        );

        begin
            exp_state = test_exp_state;
            exp_add_out = test_exp_add_out;
            exp_dout_CPU = test_exp_dout_CPU;
            exp_dout_BUS = test_exp_dout_BUS; 
            exp_dout_INSTR = test_exp_dout_INSTR;
            exp_instruction_o = exp_instr;
        end 

    endtask

    initial begin
        $dumpfile("instruction_fetch.vcd");
        $dumpvars(0, tb_instruction_fetch);
        rst = RESET_INACTIVE;
        //Test 1: Power on Reset
        stream_inputs(32'h12345678, 32'h12345678, 0);
        #(CLK_PERIOD * 2);
        stream_inputs(32'h12345678, 32'h12345678, 1);
        rst = RESET_ACTIVE;
        #(CLK_PERIOD * 2);
        rst = RESET_INACTIVE;

        stream_outputs(3'b000, 32'hABCDEF12, 32'hABCDEF12, 32'hABCDEF12, 32'hABCDEF12, 32'h0);

        stream_inputs(32'h12345678, 32'h12345678, 1);
        #(CLK_PERIOD);
        stream_inputs(32'h12345678, 32'h12345678, 0);
        #(CLK_PERIOD * 5);

        
        $finish;

    end


endmodule