module ALU(
    input logic ALU_source,
    input logic [6:0] opcode,
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic [31:0] reg1, reg2,
    input logic [19:0] immediate,
    output logic [31:0] read_address,
    output logic [31:0] write_address,
    output logic [31:0] result,
    output logic branch
);

logic [31:0] val2, rd;

always_comb begin
    if (ALU_source) begin
        val2 = immediate;
    end else begin
        val2 = reg2;
    end
end

always_comb begin
    case(opcode)
        7'b0110011, 7'b0010011:begin
            case(funct3):
                3'b000: begin
                    if (funct7==7'b0100000): rd = reg1-val2;
                    else: rd = reg1+val2;
                end
                3'b100: rd = reg1^val2;
                3'b110: rd = reg1|val2;
                3'b111: rd = reg1&val2;
                3'b001: //shift left logical
                3'b101: //shift right logical
                default: rd=32'b0
            endcase end
        7'b0000011: //load word
        7'b0100011: //store word
        7'b1100011:begin
            case(funct3):
                3'b000: begin
                    if (reg1==reg2): branch=1'b1;
                    else: branch=1'b0;
                end
                3'b001:  begin
                    if (reg1!=reg2): branch=1'b1;
                    else: branch=1'b0;
                end
                3'b100:  begin
                    if (reg1<reg2): branch=1'b1;s
                    else: branch=1'b0;
                end
                3'b110: begin
                    if (reg1>=reg2): branch=1'b1;
                    else: branch=1'b0;
                end
                default: branch = 1'b0;
            endcase end
        7'b1101111: //jump and link
        7'b1100111: //jump and link reg
        7'b0110111: //load upper imm
        7'b0010111: //add upper imm to PC
        default: rd=32'b0
    endcase
end



endmodule