module tb_control_unit;

    logic [31:0] instruction;
    logic [6:0] opcode, funct7;
    logic [2:0] funct3;
    logic [4:0] rs1, rs2, rd;
    logic [19:0] imm_20;
    logic ALU_source; //0 means register, 1 means immediate

    control_unit cont(.instruction(instruction), .opcode(opcode), .funct7(funct7), .funct3(funct3), .rs1(rs1), .rs2(rs2), .rd(rd), .imm_20(imm_20), .ALU_source(ALU_source));

    initial begin
        $dumpfile("sim.vcd");
        $dumpvars(0, tb_control_unit);

        add_r1_r2_r3;

    end

    task add_r1_r2_r3;
        instruction = 32'b0000000_00001_00010_000_00011_0110011;
        #5;
        check_opcode(7'b0110011);
        check_funct7(7'b0);
        check_funct3(3'b0);
        check_registers(5'b00001, 5'b00010, 5'b00011);
        check_imm(20'b0);
    endtask

    task check_opcode(input [6:0] exp_opcode)
        if(opcode != exp_opcode) $display("Incorrect opcode. Expected opcode: %b, actual opcode: %b", exp_opcode, opcode);
    endtask

    task check_funct7(input [6:0] exp_funct7)
        if(funct7 != exp_funct7) $display("Incorrect funct7. Expected funct7: %b, actual funct7: %b", exp_funct7, funct7);
    endtask

    task check_funct3(input [2:0] exp_funct3)
        if(funct3 != exp_funct3) $display("Incorrect funct3. Expected funct3: %b, actual funct3: %b", exp_funct3, funct3);
    endtask

    task check_registers(input [4:0] exp_rs1, exp_rs2, exp_rd)
        if(rs1 != exp_rs1) $display("Incorrect rs1. Expected rs1: %b, actual rs1: %b", exp_rs1, rs1)
        if(rs2 != exp_rs2) $display("Incorrect rs2. Expected rs2: %b, actual rs2: %b", exp_rs2, rs2)
        if(rd != exp_rd) $display("Incorrect rd. Expected rd: %b, actual rd: %b", exp_rd, rd)
    endtask

    task check_imm(input [20:0] exp_imm)
        if(imm_20 != exp_imm) $display("Incorrect imm_20. Expected imm_20: %b, actual imm_20: %b", exp_imm, imm_20)
    endtask

endmodule