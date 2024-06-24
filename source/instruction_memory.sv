module instruction_memory(
    input logic [31:0] instruction_adr_i, instruction_i,
    input logic clk, data_good, rst,
    output logic instr_fetch,
    output logic [31:0] instruction_adr_o, instruction_o
);

    logic next_fetch;
    logic [31:0] stored_instr, stored_instr_adr;

    always_comb begin
        next_fetch = 1'b0;
        if(data_good) begin
            next_fetch = 1'b0;
            stored_instr_adr = 32'b0;
            stored_instr = instruction_i;
        end else begin
            next_fetch = 1'b1;
            stored_instr_adr = instruction_adr_i;
            stored_instr = 32'b0;
        end
    end

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            instruction_adr_o <= 32'b0;
            instruction_o <= 32'b0;
            instr_fetch <= 1'b0;
        end else begin
            instruction_adr_o <= stored_instr_adr;
            instruction_o <= stored_instr;
            instr_fetch <= next_fetch;
        end
        $display("instr fetch: %b", instr_fetch);
    end

endmodule