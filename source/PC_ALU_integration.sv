module PC_ALU_integration(
    input logic clk, nRst, load, inc, Disable, ALU_source,
    input logic [6:0] opcode,
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic [31:0] data, imm_val, reg1, reg2,
    output logic [31:0] pc_val, read_ad, write_ad, result ,
    output logic branch
);

    

    ALU ALU1(.ALU_source(ALU_source), .opcode(opcode), .funct3(funct3), .funct7(funct7), 
    .reg1(reg1), .reg2(reg2), .immediate(imm_val), .read_address(read_ad), .write_address(write_ad),
    .result(result), .branch(branch));

    pc pc1(.clk(clk), .clr(nRst), .load(load), .inc(inc), .Disable(Disable), .ALU_out(branch), .data(data), .imm_val(imm_val), .pc_val(pc_val));

endmodule




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
    always_ff @(posedge clk, negedge clr) begin

        if (~clr) begin
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


        // case (action)
        //     DISABLE: next_pc = pc_val;
        //     LOAD: next_pc = data + next_line_ad;
        //     BRANCH: next_pc = jump_ad ;
        //     INC: next_pc= next_line_ad;
        //     default: next_pc= next_line_ad;
        // endcase
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

 module ALU(
    input logic ALU_source,
    input logic [6:0] opcode,
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic [31:0] reg1, reg2, immediate,
    output logic [31:0] read_address, write_address, result,
    output logic branch
);

logic [31:0] val2, rd;


always_comb begin
    if (ALU_source) begin
        val2 = immediate;
    end else begin
        val2 = reg2;
    end end
    

always_comb begin
    read_address = 32'b0; 
    write_address = 32'b0; 
    rd = 32'b0;
    branch = 1'b0;
    //len = val2-1;
    case(opcode)
        7'b0110011, 7'b0010011, 7'b0100011:
            case(funct3)
                3'b000, 3'b010: begin
                    if (funct7==7'b0100000) begin //subtract based on f7
                        rd = reg1-val2;
                    end else begin
                        rd = reg1+val2;
                    end
                    if (opcode==7'b0000011) begin //read_address is rs1+imm 
                    read_address=rd; // rd = M[rs1+imm]
                end else begin
                    read_address=32'b0;
                end if (opcode==7'b0100011) begin //Same as above but writing
                        write_address=rd;
                        rd=reg2; // reg2 is data to be written to M[rs1+imm]
                end else begin
                        write_address = 32'b0;
                    end end 
                3'b100: rd = reg1^val2;
                3'b110: rd = reg1|val2;
                3'b111: rd = reg1&val2;
                3'b001: rd = reg1<<val2[4:0];
                3'b101: rd = reg1 >> val2[4:0];
                default: begin
                    rd=32'b0;
                    read_address=32'b0;
                    write_address=32'b0;
                end
            endcase 
        7'b1100011:begin
            case(funct3)
                3'b000: begin //branch ==
                    if (reg1==val2) branch=1'b1;
                    else branch=1'b0;
                end
                3'b001:  begin //branch !=
                    if (reg1!=val2) branch=1'b1;
                    else branch=1'b0;
                end
                3'b100:  begin //branch <
                    if (reg1<val2) branch=1'b1;
                    else branch=1'b0;
                end
                3'b101: begin //branch >=
                    if (reg1>=val2) branch=1'b1;
                    else branch=1'b0;
                end
                default: branch=1'b0;
            endcase end
        7'b1101111,7'b1100111: branch=1'b1;//jump and link, jalr
        7'b0110111: rd = {val2[19:0],12'b0}; // lui
        default: begin
            read_address = 32'b0; 
            write_address = 32'b0; 
            rd = 32'b0;
            branch = 1'b0;
        end 
    endcase
    end
endmodule