module dmem_ALU (
    input logic MemToReg,
    input logic ALU_source,
    input logic [6:0] opcode,
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic [31:0] reg1, reg2, immediate,
    input logic [31:0] data_bus_i,
    input logic clk, data_good, rst,
    output logic [31:0] read_address, write_address, result,
    output logic branch,
    output logic data_read, data_write,
    output logic [31:0] data_adr_o, data_bus_o, data_cpu_o
);
    
    ALU alu_grab(
        //input
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

    data_memory init(
        //input
        .data_read_adr_i(read_address),
        .data_write_adr_i(write_address),
        .data_bus_i(data_bus_i),
        .data_cpu_i(result),
        .clk(clk),
        .data_good(data_good),
        .rst(rst),
        //output
        .data_read(data_read),
        .data_write(data_write),
        .data_adr_o(data_adr_o),
        .data_bus_o(data_bus_o),
        .data_cpu_o(data_cpu_o)
    );

endmodule

module data_memory(
    input logic [31:0] data_read_adr_i, data_write_adr_i, data_bus_i, data_cpu_i,
    input logic clk, data_good, rst,
    output logic data_read, data_write,
    output logic [31:0] data_adr_o, data_bus_o, data_cpu_o
    );

    logic next_read, next_write;
    logic [31:0] stored_read_data, stored_write_data, stored_data_adr;

    always_comb begin
        next_read = 1'b0;
        next_write = 1'b0;
        stored_read_data = 32'b0;
        stored_write_data = 32'b0;
        stored_data_adr = 32'b0;
        if(data_read_adr_i != 32'b0) begin
            if(data_good) begin
                stored_read_data = data_bus_i;
                next_read = 1'b0;
            end else begin
                stored_read_data = 32'b0;
                next_read = 1'b1;
                stored_data_adr = data_read_adr_i;
            end
        end else if(data_write_adr_i != 32'b0) begin
            if(data_good) begin
                stored_write_data = 32'b0;
                next_write = 1'b0;
            end else begin
                stored_write_data = data_cpu_i;
                next_write = 1'b1;
                stored_data_adr = data_write_adr_i;
            end
        end
    end

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            data_adr_o <= 32'b0;
            data_bus_o <= 32'b0;
            data_cpu_o <= 32'b0;
            data_read <= 1'b0;
            data_write <= 1'b0;
        end else begin
            data_read <= next_read;
            data_write <= next_write;
            data_adr_o <= stored_data_adr;
            data_cpu_o <= stored_read_data;
            data_bus_o <= stored_write_data;
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
            7'b0110011, 7'b0010011, 7'b0000011, 7'b0100011:
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
                        if (reg1 == val2) branch=1'b1;
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
