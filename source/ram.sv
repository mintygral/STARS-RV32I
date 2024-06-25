module ram (
    input logic clk,
    input logic [11:0] address_data, address_instr,
    input logic [31:0] data_in,
    input logic write_enable,
    output logic [31:0] addr_out,
    output logic [31:0] instr_out
);

reg[31:0] memory [4095:0];

initial begin
    $readmemh("cpu.mem", memory);
end

always @(posedge clk) begin
    if(write_enable) begin
        memory[address_data] <= data_in;
    end
    addr_out <= memory[address_data];
    instr_out <= memory[address_instr];
    
end

endmodule