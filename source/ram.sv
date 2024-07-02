module ram (
    input logic clk,
    input logic [11:0] address_data, address_instr,
    input logic [31:0] data_in,
    input logic write_enable,
    input logic [7:0] keyboard_in,
    output logic [31:0] addr_out,
    output logic [31:0] instr_out
);

reg[31:0] memory [4095:0]; //6 bytes of reserved data
logic [31:0] output_word;

//reserved memory for I/O
//[4095:4092] -> LCD screen data out
//[4091] -> keyboard inputs
//[4090] -> temp input

initial begin
    $readmemh("cpu.mem", memory);
end

always_comb begin
    if(address_instr != 12'd25) begin
        output_word = memory[address_instr];
    end else begin
        case(address_in)
            (12'd25): output_word = {24'b0, keyboard_in}
        endcase
    end
end

always_ff @(posedge clk) begin
    if(write_enable) begin
        memory[address_data] <= data_in;
    end
    addr_out <= memory[address_data];
    instr_out <= output_word;
end

endmodule