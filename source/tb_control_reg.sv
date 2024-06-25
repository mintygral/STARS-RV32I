`timescale 1ms/10ps

module tb_control_reg;

    /////////////////////
    // Testbench Setup //
    /////////////////////
    localparam CLK_PERIOD = 10;
    localparam RESET_ACTIVE = 1;
    localparam RESET_INACTIVE = 0;

    // Testbench signals
    integer tb_test_num;
    string tb_test_name;

    logic [31:0] instruction;
    //output
    logic [6:0] opcode, funct7;
    logic [2:0] funct3;
    logic [4:0] rs1, rs2, rd, exp_rs1, exp_rs2, exp_rd;
    logic [31:0] immediate;
    logic ALU_source, memToReg, load_pc;

    //register
    //input
    logic [31:0] reg_write;
    logic clk, rst, writeEnable;
    logic [31:0] reg1, reg2;

    control_reg DUT (
        .instruction(instruction),
        .opcode(opcode),
        .funct7(funct7),
        .funct3(funct3),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .immediate(immediate),
        .ALU_source(ALU_source),
        .memToReg(memToReg),
        .load_pc(load_pc),
        .reg_write(reg_write),
        .clk(clk),
        .rst(rst),
        .writeEnable(writeEnable),
        .reg1(reg1),
        .reg2(reg2)
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

    task check_registers([4:0] exp_rs1, exp_rs2, exp_rd);
        if(rs1 != exp_rs1) $display("Incorrect rs1. Expected rs1: %b, actual rs1: %b", exp_rs1, rs1);
        if(rs2 != exp_rs2) $display("Incorrect rs2. Expected rs2: %b, actual rs2: %b", exp_rs2, rs2);
        if(rd != exp_rd) $display("Incorrect rd. Expected rd: %b, actual rd: %b", exp_rd, rd);
    endtask

    initial begin
        
        $dumpfile("control_reg.vcd");
        $dumpvars(0, tb_control_reg);

        tb_test_num = -1;
        tb_test_name = "Test Bench Initialization";

        rst = 1'b0;
        writeEnable = 1'b0;
        reg_write = 32'b0;
        instruction = 32'b0;
        
        ////////////
        // Test 0 //
        ////////////

        tb_test_num += 1;
        tb_test_name = "Test 0";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        reset_dut();

        instruction = 32'b0000000_00001_00010_000_00011_0110011; //sent through control --> send values to registers
        exp_rs1 = 5'b00010;
        exp_rs2 = 5'b00001;
        exp_rd = 5'b00011;

        @ (posedge clk);
         
        check_registers(exp_rs1, exp_rs2, exp_rd);

        ////////////
        // Test 1 //
        ////////////

        tb_test_num += 1;
        tb_test_name = "Test 1";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        reset_dut();

        instruction = 32'b0100000_01011_10100_000_11111_0110011;

        @ (posedge clk);

        check_registers(5'b10100, 5'b01011, 5'b11111);

        #10;

        $finish;
    end
endmodule