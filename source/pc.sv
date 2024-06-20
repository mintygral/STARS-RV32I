
module pc(
    input logic clk, clr, load, inc, ALU_out, Disable,
    input logic [31:0] data, imm_val, 
    output logic [31:0] pc_val 
);
    logic [31:0] next_line_ad;
    logic [31:0] jump_ad;
    logic [31:0] next_pc;
    logic branch_choice;
    //logic [31:0] next_ad;

    // Register 
    always_ff @(posedge clk, negedge clr) begin

        if (~clr) begin
            pc_val <= 32'd0;
        end

        else begin
            pc_val <= next_pc;
            //next_ad <= next_pc;
        end
    end

   always_comb begin
       next_pc = pc_val;
       next_line_ad = pc_val + 32'd4;	// Calculate next line address
       jump_ad = pc_val + imm_val;    // Calculate jump address
       branch_choice = ALU_out;     // branch choice based on ALU output
	
        // Mux choice between next line address and jump address
        if (load) begin
            if (branch_choice) begin
		        next_pc = jump_ad;
	        end
	
	        else begin
		        next_pc = pc_val;
	        end
        end
      
       else if (inc) begin
            next_pc= next_line_ad;
       end
    //    else begin
    //         next_pc = data;
    //    end

       else if (Disable) begin 
		    next_pc = pc_val; 
	    end

       
   end

        
    
 endmodule
