module register_ALU_integration(
    // inputs
    //registers
    input logic [31:0] write_data,
    input logic [4:0] rd, rs1, rs2,
    input logic clk, rst, writeEnable,
    // ALU
    input logic ALU_source,
    input logic [6:0] opcode, funct7,
    input logic [2:0] funct3,
    input logic [31:0] immediate,

    // Outputs
    // Register
    // ALU
    output logic [31:0] read_address, write_address, result,
    output logic branch,
    output logic [31:0] register1, register2
);

    logic [31:0] reg1, reg2;
    register_file regfile ( 
        .reg_write(write_data),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .clk(clk),
        .rst(rst),
        .write(writeEnable),
        //outputs
        .reg1(reg1),
        .reg2(reg2)
        );

    ALU alu_grab(
        .ALU_source(ALU_source),
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .reg1(reg1),
        .reg2(reg2),
        .immediate(immediate),
        //output
        .read_address(read_address),
        .write_address(write_address),
        .result(result),
        .branch(branch)
        );

    assign register1 = reg1; 
    assign register2 = reg2;

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

logic [31:0] val2;


always_comb begin
    if (ALU_source) begin
        val2 = immediate;
    end else begin
        val2 = reg2;
    end end
    

always_comb begin
    read_address = 32'b0; 
    write_address = 32'b0; 
    result = 32'b0;
    branch = 1'b0;
    //len = val2-1;
    case(opcode)
        7'b0110011, 7'b0010011, 7'b0100011:
            case(funct3)
                3'b000, 3'b010: begin
                    if (funct7==7'b0100000) begin //subtract based on f7
                        result = reg1-val2;
                    end else begin
                        result = reg1+val2;
                    end
                    if (opcode==7'b0000011) begin //read_address is rs1+imm 
                    read_address=result; // result = M[rs1+imm]
                end else begin
                    read_address=32'b0;
                end if (opcode==7'b0100011) begin //Same as above but writing
                        write_address=result;
                        result=reg2; // reg2 is data to be written to M[rs1+imm]
                end else begin
                        write_address = 32'b0;
                    end end 
                3'b100: result = reg1^val2;
                3'b110: result = reg1|val2;
                3'b111: result = reg1&val2;
                3'b001: result = reg1 << val2[4:0];
                3'b101: result = reg1 >> val2[4:0];
                default: begin
                    result=32'b0;
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
        7'b0110111: result = {val2[19:0],12'b0}; // lui
        default: begin
            read_address = 32'b0; 
            write_address = 32'b0; 
            result = 32'b0;
            branch = 1'b0;
        end 
    endcase
    end
endmodule

module register_file (
    input logic [31:0] reg_write, 
    input logic [4:0] rd, rs1, rs2, 
    input logic clk, rst, write,
    output logic [31:0] reg1, reg2 //array????
);
    reg[31:0][31:0] register; 
    reg[31:0][31:0] next_register; 


    //assign register = '{default:'0};

    always_comb begin
        next_register = register;
        if (write) begin
            if (rd != 0) begin
                next_register[rd] = reg_write;
            end
        end
        reg1 = register[rs1];
        reg2 = register[rs2];
    end

    always_ff @ (posedge clk, negedge rst) begin //reset pos or neg or no reset
        if (~rst) begin
            register <= '0;
        end
        else begin
            register <= next_register;
           
        end
    end
endmodule