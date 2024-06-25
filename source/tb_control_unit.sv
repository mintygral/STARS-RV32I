module tb_control_unit;

    logic [31:0] instruction;
    logic [6:0] opcode, funct7;
    logic [2:0] funct3;
    logic [4:0] rs1, rs2, rd;
    logic [31:0] imm_32;
    logic ALU_source; //0 means register, 1 means immediate
    logic memToReg; //0 means write ALu output, 1 means write data output

    control_unit cont(.instruction(instruction), .opcode(opcode), .funct7(funct7), .funct3(funct3), .rs1(rs1), .rs2(rs2), .rd(rd), .imm_32(imm_32), .ALU_source(ALU_source), .memToReg(memToReg));

    initial begin
        $dumpfile("control_unit.vcd");
        $dumpvars(0, tb_control_unit);

        add_r1_r2_r3;
        sub_r11_r20_r31;
        xor_r11_r20_r31;
        or_r11_r20_r31;
        and_r11_r12_r14;
        sll_r11_r12_r14;
        srl_r11_r12_r14;
        addi_r4_r5_16;
        ori_r4_r5_16;
        andi_r4_r5_16;
        slli_r4_r5_16;
        srli_r4_r5_16;
        xori_r4_r5;
        beq_r1_r2_48;
        bneq_r1_r2_48;
        blt_r1_r2_48;
        bgte_r1_r2_48;
        lw_to_r1;
        sw_to_mem_0x11111111;
        jal_r4;

        $finish;

    end

    task add_r1_r2_r3;
        $info("add_r1_r2_r3");
        instruction = 32'b0000000_00001_00010_000_00011_0110011;
        #5;
        check_opcode(7'b0110011);
        check_funct7(7'b0);
        check_funct3(3'b0);
        check_registers(5'b00010, 5'b00001, 5'b00011);
        check_imm(32'b0);
        check_alu_source(1'b0);
        check_memToReg(1'b0);
    endtask

    task sub_r11_r20_r31;
        $info("sub_r11_r20_r31");
        instruction = 32'b0100000_01011_10100_000_11111_0110011;
        #5;
        check_opcode(7'b0110011);
        check_funct7(7'b0100000);
        check_funct3(3'b0);
        check_registers(5'b10100, 5'b01011, 5'b11111);
        check_imm(32'b0);
        check_alu_source(1'b0);
        check_memToReg(1'b0);
    endtask

    task xor_r11_r20_r31;
        $info("xor_r11_r20_r31");
        instruction = 32'b0100000_01011_10100_100_11111_0110011;
        #5;
        check_opcode(7'b0110011);
        check_funct7(7'b0100000);
        check_funct3(3'b100);
        check_registers(5'b10100, 5'b01011, 5'b11111);
        check_imm(32'b0);
        check_alu_source(1'b0);
        check_memToReg(1'b0);
    endtask

    task or_r11_r20_r31;
        $info("or_r11_r20_r31");
        instruction = 32'b0100000_01011_10100_110_11111_0110011;
        #5;
        check_opcode(7'b0110011);
        check_funct7(7'b0100000);
        check_funct3(3'b110);
        check_registers(5'b10100, 5'b01011, 5'b11111);
        check_imm(32'b0);
        check_alu_source(1'b0);
        check_memToReg(1'b0);
    endtask

    task and_r11_r12_r14;
        $info("and_r11_r12_r14");
        instruction = 32'b0000000_01100_01011_111_01110_0110011;
        #5;
        check_opcode(7'b0110011);
        check_funct7(7'b0000000);
        check_funct3(3'b111);
        check_registers(5'b01011, 5'b01100, 5'b01110);
        check_imm(32'b0);
        check_alu_source(1'b0);
        check_memToReg(1'b0);
    endtask

    task sll_r11_r12_r14;
        $info("sll_r11_r12_r14");
        instruction = 32'b0000000_01100_01011_001_01110_0110011;
        #5;
        check_opcode(7'b0110011);
        check_funct7(7'b0000000);
        check_funct3(3'b001);
        check_registers(5'b01011, 5'b01100, 5'b01110);
        check_imm(32'b0);
        check_alu_source(1'b0);
        check_memToReg(1'b0);
    endtask

    task srl_r11_r12_r14;
        $info("srl_r11_r12_r14");
        instruction = 32'b0000000_01100_01011_101_01110_0110011;
        #5;
        check_opcode(7'b0110011);
        check_funct7(7'b0000000);
        check_funct3(3'b101);
        check_registers(5'b01011, 5'b01100, 5'b01110);
        check_imm(32'b0);
        check_alu_source(1'b0);
        check_memToReg(1'b0);
    endtask

    task addi_r4_r5_16;
        $info("addi_r4_r5_16");
        instruction = 32'b000000010000_00100_000_00101_0010011;
        #5;
        check_opcode(7'b0010011);
        check_funct7(7'b0000000);
        check_funct3(3'b000);
        check_registers(5'b00100, 5'b00000, 5'b00101);
        check_imm(32'h00000010);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task ori_r4_r5_16;
        $info("ori_r4_r5_16");
        instruction = 32'b000000010000_00100_110_00101_0010011;
        #5;
        check_opcode(7'b0010011);
        check_funct7(7'b0000000);
        check_funct3(3'b110);
        check_registers(5'b00100, 5'b00000, 5'b00101);
        check_imm(32'h00000010);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task andi_r4_r5_16;
        $info("andi_r4_r5_16");
        instruction = 32'b000000010000_00100_111_00101_0010011;
        #5;
        check_opcode(7'b0010011);
        check_funct7(7'b0000000);
        check_funct3(3'b111);
        check_registers(5'b00100, 5'b00000, 5'b00101);
        check_imm(32'h00000010);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task xori_r4_r5;
        $info("xori_r4_r5");
        instruction = 32'b000000010000_00100_100_00101_0010011;
        #5;
        check_opcode(7'b0010011);
        check_funct7(7'b0000000);
        check_funct3(3'b100);
        check_registers(5'b00100, 5'b00000, 5'b00101);
        check_imm(32'h00000010);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task slli_r4_r5_16;
        $info("slli_r4_r5_16");
        instruction = 32'b000000010000_00100_001_00101_0010011;
        #5;
        check_opcode(7'b0010011);
        check_funct7(7'b0000000);
        check_funct3(3'b001);
        check_registers(5'b00100, 5'b00000, 5'b00101);
        check_imm(32'h00000010);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task srli_r4_r5_16;
        $info("srli_r4_r5_16");
        instruction = 32'b000000010000_00100_101_00101_0010011;
        #5;
        check_opcode(7'b0010011);
        check_funct7(7'b0000000);
        check_funct3(3'b101);
        check_registers(5'b00100, 5'b00000, 5'b00101);
        check_imm(32'h00000010);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task beq_r1_r2_48;
        $info("beq_r1_r2_48");
        instruction = 32'b0000011_00010_00001_000_00000_1100011;
        #5;
        check_opcode(7'b1100011);
        check_funct7(7'b0000000);
        check_funct3(3'b000);
        check_registers(5'b00001, 5'b00010, 5'b00000);
        check_imm(32'h00000030);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task bneq_r1_r2_48;
        $info("bneq_r1_r2_48");
        instruction = 32'b0000011_00010_00001_001_00000_1100011;
        #5;
        check_opcode(7'b1100011);
        check_funct7(7'b0000000);
        check_funct3(3'b001);
        check_registers(5'b00001, 5'b00010, 5'b00000);
        check_imm(32'h00000030);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task blt_r1_r2_48;
        $info("blt_r1_r2_48");
        instruction = 32'b0000011_00010_00001_100_00000_1100011;
        #5;
        check_opcode(7'b1100011);
        check_funct7(7'b0000000);
        check_funct3(3'b100);
        check_registers(5'b00001, 5'b00010, 5'b00000);
        check_imm(32'h00000030);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task bgte_r1_r2_48;
        $info("bgte_r1_r2_48");
        instruction = 32'b0000011_00010_00001_101_00000_1100011;
        #5;
        check_opcode(7'b1100011);
        check_funct7(7'b0000000);
        check_funct3(3'b101);
        check_registers(5'b00001, 5'b00010, 5'b00000);
        check_imm(32'h00000030);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task lw_to_r1;
        $info("lw_to_r1");
        instruction = 32'b0000011_00000_00010_010_00001_0000011;
        #5;
        check_opcode(7'b0000011);
        check_funct7(7'b0000000);
        check_funct3(3'b010);
        check_registers(5'b00010, 5'b00000, 5'b00001);
        check_imm(32'h00000060);
        check_alu_source(1'b1);
        check_memToReg(1'b1);
    endtask

    task sw_to_mem_0x11111111;
        $info("sw_to_mem_0x11111111");
        instruction = 32'b0000011_00001_00010_010_00001_0100011;
        #5;
        check_opcode(7'b0100011);
        check_funct7(7'b0000000);
        check_funct3(3'b010);
        check_registers(5'b00010, 5'b00001, 5'b00000);
        check_imm(32'h00000061);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task jal_r4;
        $info("jal_r4");
        instruction = 32'b0_0000010000_0_00000000_00100_1101111;
        #5;
        check_opcode(7'b1101111);
        check_funct7(7'b0000000);
        check_funct3(3'b000);
        check_registers(5'b00000, 5'b00000, 5'b00100);
        check_imm(32'h00000010);
        check_alu_source(1'b1);
        check_memToReg(1'b0);
    endtask

    task check_opcode(input [6:0] exp_opcode);
        if(opcode != exp_opcode) $display("Incorrect opcode. Expected opcode: %b, actual opcode: %b", exp_opcode, opcode);
    endtask

    task check_funct7(input [6:0] exp_funct7);
        if(funct7 != exp_funct7) $display("Incorrect funct7. Expected funct7: %b, actual funct7: %b", exp_funct7, funct7);
    endtask

    task check_funct3(input [2:0] exp_funct3);
        if(funct3 != exp_funct3) $display("Incorrect funct3. Expected funct3: %b, actual funct3: %b", exp_funct3, funct3);
    endtask

    task check_registers(input [4:0] exp_rs1, exp_rs2, exp_rd);
        if(rs1 != exp_rs1) $display("Incorrect rs1. Expected rs1: %b, actual rs1: %b", exp_rs1, rs1);
        if(rs2 != exp_rs2) $display("Incorrect rs2. Expected rs2: %b, actual rs2: %b", exp_rs2, rs2);
        if(rd != exp_rd) $display("Incorrect rd. Expected rd: %b, actual rd: %b", exp_rd, rd);
    endtask

    task check_imm(input [20:0] exp_imm);
        if(imm_32 != exp_imm) $display("Incorrect imm_32. Expected imm_32: %b, actual imm_32: %b", exp_imm, imm_32);
    endtask

    task check_alu_source(input exp_source);
        if(ALU_source != exp_source) $display("Incorrect ALU_source. Expected ALU_source: %b, actual ALU_source: %b", exp_source, ALU_source);
    endtask

    task check_memToReg(input exp_memToReg);
        if(memToReg != exp_memToReg) $display("Incorrect memToReg. Expected memToReg: %b, actual memToReg: %b", exp_memToReg, memToReg);
    endtask

endmodule