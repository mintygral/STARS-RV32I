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
        .regfile1(regfile1),
        .regfile2(regfile2),
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

    initial begin

        $dumpfile("ALU_reg.vcd");
        $dumpvars(0, tb_ALU_reg);

    $finish;
    end

endmodule