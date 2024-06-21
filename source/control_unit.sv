module control_unit(
    input logic [31:0] instruction,
    output logic [6:0] opcode, funct7,
    output logic [2:0] funct3,
    output logic [4:0] rs1, rs2, rd,
    output logic [31:0] imm_32,
    output logic ALU_source, //0 means register, 1 means immediate
    output logic memToReg //0 means use ALU output, 1 means use data from memory
);

    always_comb begin
        opcode = instruction[6:0];
        case(opcode)
            7'b0110011: //only r type instruction
                begin
                    funct3 = instruction[14:12];
                    funct7 = instruction[31:25];
                    rd = instruction[11:7];
                    rs1 = instruction[19:15];
                    rs2 = instruction[24:20];
                    imm_32 = 32'b0;
                    ALU_source = 1'b0;
                    memToReg = 1'b0;
                end
            7'b0010011, //i type instructions
            7'b0000011,
            7'b1100111:
                begin
                    funct3 = instruction[14:12];
                    rd = instruction[11:7];
                    rs1 = instruction[19:15];
                    imm_32 = {20'b0, instruction[31:20]};
                    funct7 = 7'b0;
                    rs2 = 5'b0;
                    ALU_source = 1'b1;
                    memToReg = (opcode == 7'b0000011) ? 1'b1 : 1'b0;
                end
            7'b0100011: //s type instructions
                begin
                    funct3 = instruction[14:12];
                    rs1 = instruction[19:15];
                    rs2 = instruction[24:20];
                    imm_32 = {20'b0, instruction[31:25], instruction[11:7]};
                    funct7 = 7'b0;
                    rd = 5'b0;
                    ALU_source = 1'b1;
                    memToReg = 1'b0;
                end
            7'b1100011: //b type instruction
                begin
                    funct3 = instruction[14:12];
                    rs1 = instruction[19:15];
                    rs2 = instruction[24:20];
                    imm_32 = {20'b0, instruction[31], instruction[7], instruction[30:25], instruction[11:8]};
                    funct7 = 7'b0;
                    rd = 5'b0;
                    ALU_source = 1'b1;
                    memToReg = 1'b0;
                end
            7'b1101111: //j type instruction
                begin
                    rd = instruction[11:7];
                    imm_32 = {12'b0, instruction[31], instruction[19:12], instruction[20], instruction[30:21]};
                    rs1 = 5'b0;
                    rs2 = 5'b0;
                    funct3 = 3'b0;
                    funct7 = 7'b0;
                    ALU_source = 1'b1;
                    memToReg = 1'b0;
                end
            7'b0110111: //u type instruction
                begin
                    rd = instruction[11:7];
                    imm_32 = {12'b0, instruction[31:12]};
                    rs1 = 5'b0;
                    rs2 = 5'b0;
                    funct3 = 3'b0;
                    funct7 = 7'b0;
                    ALU_source = 1'b1;
                    memToReg = 1'b0;
                end
            default:
                begin
                    rd = 5'b0;
                    imm_32 = 32'b0;
                    rs1 = 5'b0;
                    rs2 = 5'b0;
                    funct3 = 3'b0;
                    funct7 = 7'b0;
                    ALU_source = 1'b0;
                    memToReg = 1'b0;
                end
        endcase
    end

endmodule