
module PC(
    input logic clk, clr, load, inc, ALU_out
    input logic [31:0] data, immediate_value, 
    output logic [31:0] pc_val 
);
    logic [31:0] next_line_address;
    logic [31:0] jump_address;
    logic [31:0] next_pc;
    logic branch_choice;

   always_comb begin
       next_line_address = pc + 4;	// Calculate next line address
       jump_address = next_line_address + immediate_value;    // Calculate jump address
       branch_choice = ALU_out;     // branch choice based on ALU output
	
        // Mux choice between next line address and jump address
       if (inc) begin
            next_pc= next_line_address;
       end
       else begin
            next_pc = data;
       end

        
    // Register 
    always_ff @(posedge clk, negedge clr) begin
	if (disable) begin 
		pc_val <= pc_val 
	end

        else if (clr) begin
            pc_val <= 32'd0;
        end

        else if (load) begin
            if (branch_choice) begin
		pc_val <= jump_address;
	    end
	
	    else begin
		pc_val <= next_pc;
	    end
        end
endmodule
