`timescale 1ms/10ps

module tb_dmem_reg;

    /////////////////////
    // Testbench Setup //
    /////////////////////
    localparam CLK_PERIOD = 10;
    localparam RESET_ACTIVE = 1;
    localparam RESET_INACTIVE = 0;

    // Testbench signals
    integer tb_test_num;
    string tb_test_name;

    // DUT inputs/outputs
    logic MemToReg;
    logic ALU_source;
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic [31:0] reg1, reg2, immediate;
    logic [31:0] data_bus_i;
    logic clk, data_good, rst;
    logic [31:0] read_address, write_address, result;
    logic branch;
    logic data_read, data_write;
    logic [31:0] data_adr_o, data_bus_o, data_cpu_o;
    logic [4:0] rd, rs1, rs2;
    logic writeEnable;
    logic [31:0] regWrite;
    logic [31:0] register1, register2;

    // DUT instance
    dmem_reg DUT (.MemToReg(MemToReg),
                  .ALU_source(ALU_source),
                  .opcode(opcode),
                  .funct3(funct3),
                  .funct7(funct7),
                  .reg1(reg1),
                  .reg2(reg2),
                  .immediate(immediate),
                  .data_bus_i(data_bus_i),
                  .clk(clk),
                  .data_good(data_good),
                  .rst(rst),
                  .read_address(read_address),
                  .write_address(write_address),
                  .result(result),
                  .branch(branch),
                  .data_read(data_read),
                  .data_write(data_write),
                  .data_adr_o(data_adr_o),
                  .data_bus_o(data_bus_o),
                  .data_cpu_o(data_cpu_o),
                  .rd(rd),
                  .rs1(rs1),
                  .rs2(rs2),
                  .writeEnable(writeEnable),
                  .regWrite(regWrite),
                  .register1(register1),
                  .register2(register2));
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

    task check_ALU(input [31:0] exp_read_add,
                        input [31:0] exp_write_add, exp_result);
        if (exp_read_add != read_address) $error("Incorrect read_address! Expected: %b. Actual %b", exp_read_add, read_address);
            else $info("Correct read_address! :)");
        if (exp_write_add != write_address) $error("Incorrect read_address! Expected: %b. Actual %b", exp_write_add, write_address);
            else $info("Correct write_address! :)");
        if (exp_result != result) $error("Incorrect result! Expected:  %b, actual result: %b", exp_result, result);
            else $info("Correct result! :)");
    endtask

    task check_dmem (input exp_data_read, exp_data_write,
                     input [31:0] exp_data_adr_o, exp_data_bus_o, exp_data_cpu_o);
        if (exp_data_read != data_read) $error("Incorrect data_read! Expected: %b. Actual %b", exp_data_read, data_read);
            else $info("Correct data_read! :)");
        if (exp_data_write != data_write) $error("Incorrect data_write! Expected: %b. Actual %b", exp_data_write, data_write);
            else $info("Correct data_write! :)");
        if (exp_data_adr_o != data_adr_o) $error("Incorrect data_adr_o! Expected: %b. Actual %b", exp_data_adr_o, data_adr_o);
            else $info("Correct data_adr_o! :)");
        if (exp_data_bus_o != data_bus_o) $error("Incorrect data_bus_o! Expected: %b. Actual %b", exp_data_bus_o, data_bus_o);
            else $info("Correct data_bus_o! :)");
        if (exp_data_cpu_o != data_cpu_o) $error("Incorrect data_cpu_o! Expected: %b. Actual %b", exp_data_cpu_o, data_cpu_o);
            else $info("Correct data_cpu_o! :)");
    endtask

    task check_reg(
        input logic [31:0] exp_read1, exp_read2);
    begin
        @(negedge clk);
            if (exp_read1 == register1) begin
                $info("Correct exp_read1 value!");
            end else begin
                $error("Incorrect exp_read1 value :(. Actual: %0d, Expected: %0d.", register1, exp_read1);
            end
            if (exp_read2 == register2) begin
                $info("Correct exp_read2 value!");
            end else begin
                $error("Incorrect exp_read2 value :(. Actual: %0d, Expected: %0d.", register2, exp_read2);
            end
    end
    endtask

    task check_regWrite (input logic [31:0] exp_reg_write);
        if(exp_reg_write != regWrite) $error("Incorrect regWrite! Expected: %b. Actual: %b. MemToReg: %d.", exp_reg_write, regWrite, MemToReg);
        else $info("Correct regWrite! Expected: %b. Actual: %b. MemToReg: %d.", exp_reg_write, regWrite, MemToReg);
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
        $dumpfile("dmem_reg.vcd");
        $dumpvars(0, tb_dmem_reg);

        tb_test_num = 0;
        tb_test_name = "Test";
        reset_dut();
        //////////////////////////
        //Test 1: ALU to Reg   //
        //////////////////////////
        tb_test_num += 1;
        tb_test_name = "ALU to Reg";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        MemToReg = 0;
        ALU_source = 0;
        funct7 = 7'b0;
        funct3 = 3'b0;
        opcode = 7'b0110011;
        reg1 = 32'd1;
        reg2 = 32'd1;
        immediate = 32'b0;
        data_good = 0;
        
        @(posedge clk);
        #(CLK_PERIOD * 2); // wait one clock period to transition by one state
        // read_add, write_add, result
        check_ALU(32'b0, 32'b0, 32'd2);
        // data_read, data_write, data_adr_o, data_bus_o, data_cpu_o
        check_dmem(0, 0, 0, 0, 0);
        check_regWrite(32'd2);

        /////////////////////////////
        //Test 2: Dmem to Reg      //
        /////////////////////////////
        tb_test_num += 1;
        tb_test_name = "Dmem to Reg";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        reset_dut();
        // check that read_address from ALU is updated to dmem
        MemToReg = 1;
        ALU_source = 0;
        funct7 = 7'b0;
        funct3 = 3'b0;
        opcode = 7'b0000011;
        // reg1 = 32'd1;
        // reg2 = 32'd1;
        immediate = 32'b1;
        

        // dmem inputs
        data_good = 1;
        // data_read_adr_i, data_write_bus_i, data_cpu_i set by ALU
        data_bus_i = 32'b1;
        
        @(posedge clk);
        #(CLK_PERIOD * 2); // wait one clock period to transition by one state
        // read_add, write_add, result
        check_ALU(32'd2, 32'b0, 32'd2);
        // data_read, data_write, data_adr_o, data_bus_o, data_cpu_o
        check_dmem(1'b0, 1'b0, 32'b0, 0, data_bus_i);
        check_regWrite(data_bus_i);

        ////////////////////////////////
        //Test 3: Dmem to Reg LOOP    //
        ///////////////////////////////
        tb_test_num += 1;
        tb_test_name = "Dmem to Reg LOOP test";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);

        reset_dut();
        // check that read_address from ALU is updated to dmem
        MemToReg = 1;
        ALU_source = 0;
        funct7 = 7'b0;
        funct3 = 3'b0;
        opcode = 7'b0000011;
        reg1 = 32'd1;
        reg2 = 32'd1;
        immediate = 32'b1;
        
        // dmem inputs
        data_good = 1;
        // data_read_adr_i, data_write_bus_i, data_cpu_i set by ALU
        data_bus_i = 32'b1;
        
        @(posedge clk);
        #(CLK_PERIOD * 2); // wait one clock period to transition by one state
        // read_add, write_add, result
        check_ALU(32'd2, 32'b0, 32'd2);
        // data_read, data_write, data_adr_o, data_bus_o, data_cpu_o
        check_dmem(1'b0, 1'b0, 32'b0, 0, data_bus_i);
        check_regWrite(data_bus_i);

        rd = 5'b1;
        rs1 = 5'b1;
        rs2 = 5'b0;
        writeEnable = 1;
        @(posedge clk);
        #(CLK_PERIOD * 2);
        check_reg(data_bus_i, 32'b0);

        // try going back to ALU
        reg1 = register1;
        reg2 = register2;

        addr2;
        @(posedge clk);
        #(CLK_PERIOD * 2); // wait one clock period to transition by one state
        // #10;
        ck_result(32'b1);

        $finish;
    end
endmodule