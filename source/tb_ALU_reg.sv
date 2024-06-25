//timescale
`timescale 1ms/10ps

module tb_ALU_reg;
    /////////////////////
    // Testbench Setup //
    /////////////////////
    localparam CLK_PERIOD = 10;
    localparam RESET_ACTIVE = 1;
    localparam RESET_INACTIVE = 0;

    // Testbench signals
    integer tb_test_num;
    string tb_test_name;

    // DUT inputs / outputs
    // All inputs
    // register_file
    logic [31:0] reg_write;
    logic [4:0] rd, rs1, rs2; 
    logic clk, rst, writeEnable;
    // ALU
    logic ALU_source;
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic [31:0] regfile1, regfile2, immediate;
     
    // All outputs
    // register file
    logic [31:0] regALU1, regALU2; //array????
    // ALU
    logic [31:0] read_address, write_address, result;
    logic branch;

    ALU_reg DUT(
        //register input
        .reg_write(reg_write),
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
        // .regfile1(regALU1),
        // .regfile2(regALU2),
        .regALU1(regALU1),
        .regALU2(regALU2),
        .read_address(read_address),
        .write_address(write_address),
        .result(result),
        .branch(branch)
    );
    

    ////////////////////////
    // Testbenching tasks //
    ////////////////////////

    // toggle clk
    always begin 
        clk = 0;
        #(CLK_PERIOD / 2);
        clk = 1;
        #(CLK_PERIOD / 2);
    end

    // Quick reset for 2 clock cycles
    task reset_dut;
        begin
            @(negedge clk); // synchronize to negedge edge so there are not hold or setup time violations
            
            // Activate reset
            rst = RESET_ACTIVE;

            // Wait 2 clock cycles
            @(negedge clk);
            @(negedge clk);

            // Deactivate reset
            rst = RESET_INACTIVE; 
        end
    endtask

    task ck_result(input [31:0] exp_result); 
    begin
        @(posedge clk)
        if (exp_result != result) $error("You suck :(. Expected:  %b, actual result: %b", exp_result, result);
        else $info("Correct output! :)");    end
    endtask

    task rst_r2R;
        ALU_source=1'b0;
        funct7=7'b0;
        opcode=7'b0110011;
        // gen_vals;
    endtask

    task addr2;
        // $info("addr2");
        rst_r2R;
        funct3=3'b000;
        // reg1 = 32'b1;
        // reg2 = 32'b0;
        #5;
    endtask

    task rst_immR;
        ALU_source=1'b1;
        funct7=7'b0;
        opcode=7'b0010011;
        // gen_vals;
    endtask

    initial begin

        $dumpfile("ALU_reg.vcd");
        $dumpvars(0, tb_ALU_reg);

        tb_test_num = -1;
        tb_test_name = "Test Bench Initialization";
        rst = 1'b0;
        writeEnable = 1'b0;
        reg_write = 32'b0;
        rd = 5'b0;
        rs1 = 5'b0;
        rs2 ='0;
        ALU_source = 0;

        #(0.5);
        
        //////////////////////////
        //Test 0: Power on Reset//
        //////////////////////////
        // $info("Test 0: power on reset");

        tb_test_num += 1;
        tb_test_name = "Power on reset";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        

        //set inputs to non-reset values
        rd = 5'b1;
        reg_write = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;
        immediate = 32'b1;
        
        rs1 = 5'b1;
        rs2 = 5'b0;

        // exp_read1 = 32'b1;
        // exp_read2 = 32'b0;
        // check_output(exp_read1, exp_read2);
        // #5;
        tb_test_num += 1;
        tb_test_name = "Checking Addition";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        rst = RESET_ACTIVE;
        #(CLK_PERIOD * 2); // Wait 2 clock periods before proceeding
        rst = RESET_INACTIVE;

        addr2;
        @(posedge clk);
        #(CLK_PERIOD * 2); // wait one clock period to transition by one state
        // #10;
        ck_result(32'b1);
        

        // rd = 5'b00010;
        // reg_write = 32'b1;
        // writeEnable = 1'b1;

        // rs2 = 5'b00010;

        // exp_read1 = 32'b1;
        // exp_read2 = 32'b1;
        // check_output(exp_read1, exp_read2);

        reset_dut();
        #10;

        //set inputs to non-reset values
        rd = 5'b1;
        reg_write = 32'b1;
        rs1 = 5'b0;
        rs2 = 5'b0;
        writeEnable = 1'b1;
        
        rs1 = 5'b1;
        rs2 = 5'b0;

        // exp_read1 = 32'b1;
        // exp_read2 = 32'b0;

        tb_test_num += 1;
        tb_test_name = "Checking Addition w/ Imm";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // #(CLK_PERIOD * 2); // Wait 2 clock periods before proceeding
        rst_immR;
        funct3=3'b000;
        @(posedge clk);
        #(CLK_PERIOD * 2); // wait one clock period to transition by one state
        // #10;
        ck_result(2);

        // exp_read1 = 32'b0;
        // exp_read2 = 32'b1; //still being written too
        // check_output(exp_read1, exp_read2);

    $finish;
    end

endmodule