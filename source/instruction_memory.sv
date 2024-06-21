module instruction_memory(
    input logic [31:0] instruction_adr_i,
    input logic sys_clk,
    output logic instr_fetch,
    output logic [31:0] instruction_adr_o
);

    always_ff @(posedge sys_clk) begin
        instruction_adr_o <= instruction_adr_i;
        instr_fetch = 1'b1;
    end

endmodule