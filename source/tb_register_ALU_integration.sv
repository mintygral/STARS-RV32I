//timescale
`timescale 1ms/10ps

module tb_register_ALU_integration;
    localparam CLK_PERIOD = 10;
    reg[31:0] write_data, exp_read1, exp_read2;
    logic [4:0] rd, rs1, rs2;
    logic clk, rst, writeEnable;

    logic [31:0] reg1, reg2;

    logic ALU_source;
    logic [6:0] opcode, funct7;
    logic [2:0] funct3;
    logic [31:0] immediate, read_address, write_address, result;
    logic branch;
    integer tb_test_num;
    string tb_test_name;


    register_ALU_integration test(
        //register input
        .write_data(write_data),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .clk(clk),
        .rst(rst),
        .writeEnable(writeEnable),
        //ALU input
        .ALU_source(ALU_source),
        .opcode(opcode),
        .funct7(funct7),
        .funct3(funct3),
        .immediate(immediate),
        //output
        .read_address(read_address),
        .write_address(write_address),
        .result(result),
        .branch(branch),
        .register1(reg1),
        .register2(reg2)
    );
    
    //toggle clk
    always begin 
        clk = 0;
        #(CLK_PERIOD / 2);
        clk = 1;
        #(CLK_PERIOD / 2);
    end

    initial begin

        $dumpfile("tb_register_ALU_integration.vcd");
        $dumpvars(0, tb_register_ALU_integration);

        tb_test_num = -1;
        tb_test_name = "Test bench initialization";
        rst = 1'b1;
        write_data = 32'b0;
        rd = 5'b0;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b0;
        ALU_source = 1'b0;
        opcode = 7'b0;
        funct7 = 7'b0;
        funct3 = 3'b0;
        immediate = 32'b0;

        #(0.5);

        ///////////////////////
        // Test : addr2 test //
        ///////////////////////

        tb_test_num += 1;
        tb_test_name = "addr2 test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        addr2;

        //////////////////////
        // Test : addi test //
        //////////////////////

        tb_test_num += 1;
        tb_test_name = "addi test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        addi;

        ///////////////////////
        // Test : subr2 test //
        ///////////////////////

        tb_test_num += 1;
        tb_test_name = "subr2 test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        subr2;

        //////////////////////
        // Test : subi test //
        //////////////////////

        tb_test_num += 1;
        tb_test_name = "subi test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        subi;

        ///////////////////////
        // Test : XORr2 test //
        ///////////////////////

        tb_test_num += 1;
        tb_test_name = "XORr2 test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        XORr2;

        //////////////////////
        // Test : XORi test //
        //////////////////////

        tb_test_num += 1;
        tb_test_name = "XORi test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        XORi;

        //////////////////////
        // Test : ORr2 test //
        //////////////////////

        tb_test_num += 1;
        tb_test_name = "ORr2 test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        ORr2;

        /////////////////////
        // Test : ORi test //
        /////////////////////

        tb_test_num += 1;
        tb_test_name = "ORi test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        ORi;

        ///////////////////////
        // Test : ANDr2 test //
        ///////////////////////

        tb_test_num += 1;
        tb_test_name = "ANDr2 test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        ANDr2;

        //////////////////////
        // Test : ANDi test //
        //////////////////////

        tb_test_num += 1;
        tb_test_name = "ANDi test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        toggle_rst;

        rd = 5'b1;
        write_data = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;

        rs1 = 5'b1;
        rs2 = 5'b0;

        exp_read1 = 32'b1;
        exp_read2 = 32'b0;
        check_output(exp_read1, exp_read2);

        rd = 5'b00010;
        write_data = 32'b1;
        writeEnable = 1'b1;

        rs2 = 5'b00010;

        exp_read1 = 32'b1;
        exp_read2 = 32'b1;
        check_output(exp_read1, exp_read2);

        ANDi;

    end

    task check_output;
        input logic [31:0] exp_read1, exp_read2;
    begin
        @(negedge clk);
            if (exp_read1 == reg1) begin
                $info("Correct exp_read1 value!");
            end else begin
                $error("Incorrect exp_read1 value :(. Actual: %0d, Expected: %0d.", reg1, exp_read1);
            end
            if (exp_read2 == reg2) begin
                $info("Correct exp_read2 value!");
            end else begin
                $error("Incorrect exp_read2 value :(. Actual: %0d, Expected: %0d.", reg2, exp_read2);
            end
    end
    endtask

    task toggle_rst; 
        rst = 1; #10;
        rst = 0; #10;
        rst = 1; #10;
    endtask

    task gen_vals;
    //tbd
    endtask

    task gen_eq;
    endtask

    task gen_r1_less;
    endtask

    task gen_r1_geq;
    endtask

    task gen_neq;
    endtask

    task rst_r2R;
        ALU_source = 1'b0;
        funct7 = 7'b0;
        opcode = 7'b0110011;
        gen_vals;
    endtask

    task rst_immR;
        ALU_source = 1'b1;
        funct7 = 7'b0;
        opcode = 7'b0010011;
        gen_vals;
    endtask

    task rst_branch;
        opcode = 7'b1100011;
        ALU_source = 1'b0;
        funct7 = 7'b0;
    endtask

    // only difference between rst is opc
    //maybe implement neg functionality
    //maybe implement zero
    //need to also implement total systemwide reset ck for zero

    task addr2;
        $info("addr2");
        rst_r2R;
        funct3 = 3'b000;
        #5;
        ck_sum(32'b1);
    endtask

    task addi;
        $info("addi");
        rst_immR;
        funct3 = 3'b000;
        #5;
        ck_sum(32'b1);
    endtask 

    task subr2;
        $info("subr2");
        rst_r2R;
        funct3 = 3'b000;
        funct7 = 7'b0100000;
        #5;
        ck_diff(32'b1);
    endtask

    task subi;
        rst_immR;
        funct3 = 3'b000;
        funct7 = 7'b0100000;
        reg1 = 32'b1;
        immediate = 32'b1;
        #5;
        ck_diff(32'b0);
    endtask

    task XORr2;
        $info("XORr2");
        rst_r2R;
        funct3 = 3'b100;
        #5;
        ck_logical(32'b1);
    endtask

    task XORi;
        $info("XORi");
        rst_immR;
        funct3 = 3'b100;
        #5;
        ck_logical(32'b1);
    endtask

    task ORr2;
        $info("ORr2");
        rst_r2R;
        funct3 = 3'b110;
        #5;
        ck_logical(32'b1);
    endtask

    task ORi;
        $info("ORi");
        rst_immR;
        funct3 = 3'b110;
        #5;
        ck_logical(32'b1);
    endtask

    task ANDr2;
        $info("ANDr2");
        rst_r2R;
        funct3 = 3'b111;
        #5;
        ck_logical(32'b1);
    endtask
    
    task ANDi;
        $info("ANDi");
        rst_immR;
        funct3 = 3'b111;
        #5;
        ck_logical(32'b1);
    endtask

    task ck_sll_r2;
        $info("sll_r2");
        rst_r2R;
        reg2=32'd32; //full bit shift
        funct3 = 3'b001; 
        #5;
        ck_shift(32'b0);
        rst_r2R;
        reg2=32'd16; //half bit shift 
        #5;
        ck_shift(32'b1);
        rst_r2R;
        reg2=32'd1; // 1 bit shift
        #5;
        ck_shift(32'b1);
    endtask

    task ck_srl_r2;
        $info("srl_r2");
        rst_r2R;
        reg2=32'd32; //full bit shift
        funct3 = 3'b101; 
        #5;
        ck_shift(32'b0);
        rst_r2R;
        reg2=32'd16; //half bit shift 
        #5;
        ck_shift(32'b1);
        rst_r2R;
        reg2=32'd1; // 1 bit shift
        #5;
        ck_shift(32'b1);
    endtask

    task ck_sll_imm;
        $info("sll_imm");
        rst_immR;
        immediate = 32'd32; //full bit shift
        funct3 = 3'b001; 
        #5;
        ck_shift(32'b0);
        rst_immR;
        immediate = 32'd16; //half bit shift 
        #5;
        ck_shift(32'b1);
        rst_r2R;
        immediate = 32'd1; // 1 bit shift
        #5;
        ck_shift(32'b1);
    endtask

    task ck_srl_imm;
        funct3 = 3'b101;
        $info("srl_imm");
        rst_immR;
        immediate = 32'd32; //full bit shift 
        #5;
        ck_shift(32'b0);
        rst_immR;
        immediate =32'd16; //half bit shift 
        #5;
        ck_shift(32'b1);
        rst_r2R;
        immediate = 32'd1; // 1 bit shift
        #5;
        ck_shift(32'b1);
    endtask
    
    task branch_eq;
        $info("branch_eq");
        rst_branch;
        gen_eq;
        funct3 = 3'b0;
        ck_branch(1'b1);
        rst_branch;
        gen_neq;
        ck_branch(1'b0);
    endtask

    task branch_neq;
        $info("branch_neq");
        rst_branch;
        gen_neq;
        funct3 = 3'b001;
        ck_branch(1'b1);
        rst_branch;
        gen_eq;
        ck_branch(1'b0);
    endtask

    task branch_r1_less;
        $info("branch_r1_less");
        rst_branch;
        gen_r1_less;
        funct3 = 3'b110;
        ck_branch(1'b1);
        rst_branch;
        gen_r1_geq;
        ck_branch(1'b0);
    endtask

    task branch_r1_geq;
        $info("branch_r1_geq");
        rst_branch;
        gen_r1_geq;
        funct3 = 3'b110;
        ck_branch(1'b1);
        rst_branch;
        gen_r1_less;
        ck_branch(1'b0);
    endtask


    task ck_sum(input [31:0] exp_sum);
        if(result != exp_sum) $display("Incorrect result. Expected sum: %b, actual result(sum): %b", exp_sum, result);
    endtask

    task ck_diff(input [31:0] exp_diff);
        if(result != exp_diff) $display("Incorrect result. Expected difference: %b, actual result(difference): %b", exp_diff, result);
    endtask

    task ck_logical(input [31:0] exp_comp);
        if(result != exp_comp) $display("Incorrect result. Expected comparison: %b, actual result(logical): %b", exp_comp, result);
    endtask

    task ck_shift(input [31:0] exp_shift);
        if(result != exp_shift) $display("Incorrect result. Expected shift: %b, actual result(shift): %b", exp_shift, result);
    endtask

    task ck_branch(input exp_branch);
        if(branch != exp_branch) $display("Incorrect branch. Expected branch: %b, actual branch value: %b", exp_branch, branch);
    endtask

endmodule