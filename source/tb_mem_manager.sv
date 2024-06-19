// inputs
logic [31:0] address_in, data_in_CPU, data_in_BUS;
logic data_en, instr_en, bus_full, memWrite, memRead;
logic clk, rst;


// outputs
state_t state;
logic [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR;
