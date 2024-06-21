module cpu_core(

);


    //Control Unit Inputs/Outputs
    logic [31:0] instruction;
    logic [6:0] opcode, funct7;
    logic [2:0] funct3;
    logic [4:0] rs1, rs2, rd;
    logic [31:0] imm_32;
    logic ALU_source; //0 means register, 1 means immediate
    logic memToReg; //0 means write ALu output, 1 means write data output

    //register outputs
    logic [31:0] reg_write;
    logic clk, rst, write;
    logic [31:0] reg1, reg2;

    //ALU outputs
    logic [31:0] read_address;
    logic [31:0] write_address;
    logic [31:0] result;
    logic branch;

    control_unit cont(.instruction(instruction), .opcode(opcode), .funct7(funct7), .funct3(funct3), .rs1(rs1), .rs2(rs2), .rd(rd), .imm_32(imm_32), .ALU_source(ALU_source), .memToReg(memToReg));

    register_file register(.reg_write(reg_write), .rs1(rs1), .rs2(rs2), .rd(rd), .clk(clk), .rst(rst), .write(write), .reg1(reg1), .reg2(reg2));

    ALU math(.opcode(opcode), .funct3(funct3), .funct7(funct7), .ALU_source(ALU_source), .reg1(reg1), .reg2(reg2), .immediate(imm_32), .read_address(read_address), .write_address(write_address), .result(result), .branch(branch));

    


endmodule