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

// inputs
logic [31:0] address_in, data_in_CPU, data_in_BUS;
logic data_en, instr_en, bus_full, memWrite, memRead;
logic clk, rst;

// outputs
state_t state;
logic [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR;


state_t next_state; 
state_t prev_state;
logic [8:0] test_data;
memcontrol managemem (
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

initial begin 
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);
    rst = 0; 
    toggle_rst();
end

task toggle_rst; 
    rst = 0; #10;
    rst = 1; #10;
    rst = 0; #10;
endtask

task send_key;
    input logic [4:0] keytosend;
    begin 
        @ (negedge clk);
        // keyout = keytosend;
        @ (posedge clk);
        #10;
    end
endtask

task sendstream;
    input logic [4:0] stream [];
    begin 
        for (integer keynum = 0; keynum < stream.size(); keynum++) begin
            send_key(stream[keynum]); end
    end
endtask

endmodule