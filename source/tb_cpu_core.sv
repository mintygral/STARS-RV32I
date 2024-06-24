module tb_cpu_core;

    localparam CLK_PERIOD = 100;

    logic [31:0] data_in_BUS; //input data from memory bus
    logic bus_full; //input from memory bus
    logic clk, rst; //external clock, reset
    logic [31:0] data_out_BUS, address_out; //output data +address to memory bus

    cpu_core core0(
        .data_in_BUS(data_in_BUS),
        .bus_full(bus_full),
        .clk(clk),
        .rst(rst),
        .data_out_BUS(data_out_BUS),
        .address_out(address_out)
    );

    always begin
        clk = 1'b0;
        #(CLK_PERIOD/2);
        clk = 1'b1;
        #(CLK_PERIOD/2);
    end

    initial begin
        $dumpfile("cpu_core.vcd");
        $dumpvars(0, tb_cpu_core);
        data_in_BUS = 32'b0;
        bus_full = 32'b0;
        rst = 1'b0;
        #(CLK_PERIOD);
        rst = 1'b1;
        #(CLK_PERIOD);
        rst = 1'b0;
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00000_00100_010_00001_0000011); //load data into register 1 (figure out how to load data)
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00000_00100_010_00010_0000011); //load data into register 2 (figure out how to load data)
        #(CLK_PERIOD);
        //load_instruction(32'b) //add register 1 & 2, store in register 3

        //read data from register 3
        $finish;
    end

    task load_instruction(input [31:0] instruction);
        data_in_BUS = instruction;
        bus_full = 1'b1;
        #(CLK_PERIOD);
        bus_full = 1'b0;
        #(CLK_PERIOD * 5);
    endtask
    


endmodule