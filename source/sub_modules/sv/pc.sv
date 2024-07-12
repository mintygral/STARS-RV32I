
module pc(
    input logic clk, clr, load, inc, Disable, ALU_out,
    input logic [31:0] data, imm_val, 
    output logic [31:0] pc_val 
);
    logic [31:0] next_line_ad;
    logic [31:0] jump_ad;
    logic [31:0] next_pc;
    logic branch_choice;
    logic [31:0] imm;

    assign imm = imm_val * 32'd4;

    // Register 
    always_ff @(posedge clk, posedge clr) begin

        if (clr) begin
            pc_val <= 32'd0;
        end

        else begin
            pc_val <= next_pc;
        end
    end


   always_comb begin
       next_pc = pc_val;
       next_line_ad = pc_val + 32'd4;	// Calculate next line address  
       jump_ad = pc_val + imm;    // Calculate jump address (jump and link)

	
        // Mux choice between next line address and jump address
        if (Disable) begin 
		    next_pc = pc_val; 
	    end

        else if (load) begin
            next_pc = data + next_line_ad;
        end
            
        else if (ALU_out) begin
		    next_pc = jump_ad ;
	    end
	
        else if (inc) begin
            next_pc= next_line_ad;
       end

        
       
   end       
    
 endmodule
