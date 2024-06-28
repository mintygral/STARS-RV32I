`timescale 1ms/10ps

module tb_cpu_core;

    localparam CLK_PERIOD = 100;
    integer tb_test_num;
    string tb_test_name;

    logic [31:0] data_in_BUS, pc_jump, pc_data; //input data from memory bus
    logic bus_full; //input from memory bus
    logic clk, rst; //external clock, reset
    logic [31:0] data_out_BUS, address_out; //output data +address to memory bus
    //testing to verify control unit
    logic [31:0] imm_32, reg1, reg2, data_cpu_o, write_address, reg_write, pc_val;
    logic [31:0] result;
    logic [4:0] rs1, rs2, rd;
    logic memToReg, instr_wait, reg_write_en, branch_ff, branch;

    cpu_core core0(
        .pc_data(pc_data),
        .data_in_BUS(data_in_BUS),
        .pc_jump(pc_jump),
        .bus_full(bus_full),
        .clk(clk),
        .rst(rst),
        .data_out_BUS(data_out_BUS),
        .address_out(address_out),
        .result(result),
        .imm_32(imm_32),
        .reg1(reg1),
        .reg2(reg2),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .memToReg_flipflop(memToReg),
        .data_cpu_o(data_cpu_o),
        .write_address(write_address),
        .instr_wait(instr_wait),
        .reg_write(reg_write),
        .reg_write_en(reg_write_en),
        .pc_val(pc_val),
        .branch_ff(branch_ff),
        .branch(branch)
    );

    always begin
        clk = 1'b0;
        #(CLK_PERIOD/2);
        clk = 1'b1;
        #(CLK_PERIOD/2);
    end

    initial begin
        $dumpfile("cpu_core.vcd");
        $dumpvars(0, tb_cpu_core);
        data_in_BUS = 32'b0;
        pc_data = 32'b0;
        bus_full = 1'b0;
        rst = 1'b0;
        #(CLK_PERIOD);
        rst = 1'b1;
        #(CLK_PERIOD);
        rst = 1'b0;
        tb_test_num = 0;

        // $display("\n All Register 2 Value Tests Below");
        // // Add 1 + 1
        // tb_test_num++;
        // tb_test_name = "Testing Hardcoded Add 1+1";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // add_1plus1;

        // // Sub 32 - 2
        // tb_test_num++;
        // tb_test_name = "Testing Hardcoded Subtract 32-2";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // sub_32minus2;

        // // Add/Sub Tasks
        // tb_test_num++;
        // tb_test_name = "Testing Add/Sub Tasks for Maximum 32-bit values";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);

        // reset_dut;
        // add(32'h0000FFFF, 32'hFFFF0000, 32'hFFFFFFFF);
        // reset_dut;
        // sub(32'hFFFFFFFF, 32'hFFFF0000, 32'h0000FFFF);

        // reset_dut;
        // tb_test_num++;
        // tb_test_name = "Testing Consecutive Add then Sub";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // add_then_sub(32'd40, 32'd10, 32'd25, 32'd50, 32'd25);
        
        // reset_dut;
        // tb_test_num++;
        // tb_test_name = "Multiplication 42x7";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // multiply (32'd42, 7, 32'd294);
        // tb_test_num++;
        // tb_test_name = "Multiplication 294x2";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // multiply (reg2, 2, 32'd588);
        // tb_test_num++;
        // tb_test_name = "Division 588/14";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // divide (result, 14, 32'd42);
        // tb_test_num++;
        // tb_test_name = "Division 42/7";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // divide (reg2, 7, 32'd6);
        // tb_test_num++;
        // tb_test_name = "Division 6/6";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // divide (reg2, 6, 32'd1);
        // tb_test_num++;
        // tb_test_name = "Try to multiply 6x6";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // divide (reg2, 6, 32'd36);



        // tb_test_num++;
        // tb_test_name = "Multiplication 2";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // multiply (result, 6, 32'd36);

        // tb_test_num++;
        // tb_test_name = "Multiplication 3";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // multiply (result, 7, 32'd42);

        // tb_test_num++;
        // tb_test_name = "Multiplication 4";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // multiply (result, 7, 32'd249);
        
        // // XOR
      	// tb_test_num++;
        // tb_test_name = "Testing XOR";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_xor(32'd1, 32'd1, 0);
        // reset_dut;
        // test_xor(32'd1, 32'd0, 1);

        // // OR
        // tb_test_num++;
        // tb_test_name = "Testing OR";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_or(32'd1, 32'd0, 1);
        // reset_dut;
        // test_or(32'd0, 32'd0, 0);

        // // AND
        // tb_test_num++;  
        // tb_test_name = "Testing AND";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_and(32'd1, 32'd1, 1);
        // reset_dut;
        // test_and(32'd1, 32'd0, 0);

        // // sll
        // tb_test_num++;  
        // tb_test_name = "Testing full bit SLL";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_sll(32'hFFFFFFFF, 32'd31, 32'h80000000); // full bit shift
        // tb_test_num++;  
        // tb_test_name = "Testing half bit SLL";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_sll(32'hFFFFFFFF, 32'd16, 32'hFFFF0000); // half bit shift
        // tb_test_num++;  
        // tb_test_name = "Testing one bit SLL";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_sll(32'hFFFFFFFF, 32'd1, 32'hFFFFFFFE); // half bit shift

        // // srl
        // tb_test_num++;  
        // tb_test_name = "Testing full bit SRL";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_srl(32'hFFFFFFFF, 32'd31, 32'b1); // full bit shift
        // tb_test_num++;  
        // tb_test_name = "Testing half bit SRL";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_srl(32'hFFFFFFFF, 32'd16, 32'h0000FFFF); // half bit shift
        // tb_test_num++;  
        // tb_test_name = "Testing one bit SRL";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_srl(32'hFFFFFFFF, 32'd1, 32'h7FFFFFFF); // half bit shift
        
        // Immediate Value Tests
        // $display("\n All Immediate Value Tests Below");
        // tb_test_num++;  
        // tb_test_name = "Testing add (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // add_imm(32'd500, 32'd502);

        // reset_dut;
        // tb_test_num++;  
        // tb_test_name = "Testing add consecutively (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // add_imm_cons(32'd50, 32'd52, 32'd54);

        // tb_test_num++;  
        // tb_test_name = "Testing XOR (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_xor_imm(32'b1, 32'b0);
      
      	// tb_test_num++;  
        // tb_test_name = "Testing OR (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
      	// test_or_imm(32'b0, 32'b1); // imm is set to 1
      
      	// tb_test_num++;  
        // tb_test_name = "Testing AND (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
      	// test_and_imm(32'b0, 32'b0); // imm is set to 1
      	// reset_dut;
      	// test_and_imm(32'b1, 32'b1); // imm is set to 1

        // // sll
        // tb_test_num++;  
        // tb_test_name = "Testing full bit SLL (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_sll_imm_full(32'hFFFFFFFF, 32'h80000000); // full bit shift
        // tb_test_num++;  
        // tb_test_name = "Testing half bit SLL (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_sll_imm_half(32'hFFFFFFFF, 32'hFFFF0000); // half bit shift
        // tb_test_num++;  
        // tb_test_name = "Testing one bit SLL (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_sll_imm_one(32'hFFFFFFFF, 32'hFFFFFFFE); // half bit shift

        // // srl
        // tb_test_num++;  
        // tb_test_name = "Testing full bit SRL (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_srl_imm_full(32'hFFFFFFFF, 32'b1); // full bit shift
        // tb_test_num++;  
        // tb_test_name = "Testing half bit SRL (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_srl_imm_half(32'hFFFFFFFF, 32'h0000FFFF); // half bit shift
        // tb_test_num++;  
        // tb_test_name = "Testing one bit SRL (imm)";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // reset_dut;
        // test_srl_imm_one(32'hFFFFFFFF, 32'h7FFFFFFF); // half bit shift

        // $display("Checking branch tasks");
        // tb_test_num++;  
        // tb_test_name = "Testing Branch Equal task";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // // reset_dut;
        // test_beq(32'd1, 32'd1, 1);
        // $info("pc_val: %b", pc_val); // pc should update
        // // reset_dut;
        // test_beq(32'd0, 32'd1, 1);
        // $info("pc_val: %b", pc_val); // pc should not update

        // tb_test_num++;  
        // tb_test_name = "Testing Branch Inequal task";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // // reset_dut;
        // test_bneq(32'd1, 32'd1, 1);
        // $info("pc_val: %b", pc_val); // pc should not update
        // // reset_dut;
        // test_bneq(32'd1, 32'd0, 1); // pc should update
        // $info("pc_val: %b", pc_val);

        // tb_test_num++;  
        // tb_test_name = "Testing Branch less than task";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // // reset_dut;
        // test_blt(32'd1, 32'd1, 1);
        // $info("pc_val: %b", pc_val); // pc should not update
        // // reset_dut;
        // test_blt(32'd0, 32'd1, 1); // pc should update
        // $info("pc_val: %b", pc_val);

        // tb_test_num++;  
        // tb_test_name = "Testing Branch >= task";
        // $display("\nTest %d: %s", tb_test_num, tb_test_name);
        // // reset_dut;
        // test_bge(32'd1, 32'd1, 1);
        // $info("pc_val: %b", pc_val); // pc should update
        // // reset_dut;
        // test_bge(32'd0, 32'd1, 1); // pc should not update
        // $info("pc_val: %b", pc_val);
        
        // reset_dut;
        tb_test_num++;  
        tb_test_name = "Testing JAL";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        test_jal;

        // reset_dut;
        tb_test_num++;  
        tb_test_name = "Testing JAL (reg)";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        test_jalr;

        tb_test_num++;
        tb_test_name = "Testing LUI";
        $display("\nTest %d: %s", tb_test_num, tb_test_name);
        check_lui;


        
        // test_loop_multiply;

        $finish;
    end

    ////////////////////////
    // Testbenching tasks //
    ////////////////////////
  
    task reset_dut;
        #(CLK_PERIOD);
        data_in_BUS = 32'b0;
        bus_full = 1'b0;
        // pc_val = 32'b0;
        rst = 1'b1;
        #(CLK_PERIOD);
        rst = 1'b0;
    endtask

    task load_instruction(input [31:0] instruction, 
                        input check_enable,
                        input [31:0] exp_result);
        data_in_BUS = instruction;
        bus_full = 1'b1;
        #(CLK_PERIOD);
        bus_full = 1'b0;
    	#(CLK_PERIOD * 2);
    	if (check_enable) check_output(exp_result);
    	#(CLK_PERIOD * 2);
    endtask

    task load_data(input [31:0] data);
        data_in_BUS = data;
        bus_full = 1'b1;
        #(CLK_PERIOD);
        bus_full = 1'b0;
        #(CLK_PERIOD * 5);
    endtask

    task check_output (input [31:0] exp_result); 
        begin
          @(negedge clk) // check away from clk idk why but this works
          if (exp_result != result) $error("You suck :(. Expected:  %d, actual result: %d", exp_result, result);
          else $info("Correct output :). Expected:  %d, actual result: %d", exp_result, result);
        end
    endtask

    task add_1plus1;
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, 32'd2); //load data into register 1 (figure out how to load data)
        load_data(32'h00000001);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, 32'd2); //load data into register 2 (figure out how to load data)
        load_data(32'h00000001);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_000_00011_0110011, 1, 32'd2); //add register 1 & 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, 32'd2); //read data from register 3
    endtask

    task sub_32minus2; 
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, 32'd30); //load data into register 1 (figure out how to load data)
        load_data(32'd32);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, 32'd30); //load data into register 2 (figure out how to load data)
        load_data(32'd2);
        #(CLK_PERIOD);
        load_instruction(32'b0100000_00010_00001_000_00011_0110011, 1, 32'd30); //add register 1 & 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, 32'd30); //read data from register 3
    endtask
    
    task add (input [31:0] register1, register2, exp_result);
        $display("Now adding %d + %d = %d", register1, register2, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_000_00011_0110011, 1, exp_result); //add register 1 & 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task sub (input [31:0] register1, register2, exp_result);
        $display("Now subtracting %d - %d = %d", register1, register2, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0100000_00010_00001_000_00011_0110011, 1, exp_result); //subtract register 1 - 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask
    
    task multiply (input [31:0] register1, 
                   integer multiply_by, 
                   input [31:0] exp_result);
        // $info("Expected Final: %d. Current Result: %d.",exp_result, result);
        logic [31:0] multiplier;
        multiplier = 32'd0;
        for (integer i = 1; i < multiply_by; i++) begin
            register1 += (register1 / i);
            multiplier = register1;
            // $info("Expected Final: %d. Current Result: %d.", exp_result, multiplier);
        end
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(multiplier);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(32'd0);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_000_00011_0110011, 1, exp_result); //add register 1 & 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask 

    task divide (input [31:0] register1, 
                   integer divide_by, 
                   input [31:0] exp_result);
        logic [31:0] divisor;
        divisor = 32'd0;
        for (integer i = 0; i < exp_result; i++) begin
            register1 -= divide_by;
            divisor++;
            // $info("Expected Final: %d. Current Result: %d.", exp_result, register1);
        end
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(divisor);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(32'd0);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_000_00011_0110011, 1, exp_result); //add register 1 & 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask 

    task add_then_sub (input [31:0] register1, register2, register4, exp_sum, exp_diff);
        $display("Now adding %d + %d = %d", register1, register2, exp_sum);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_sum); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_sum); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_000_00011_0110011, 1, exp_sum); //add register 1 & 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00100_0000011, 0, exp_diff); //load data into register 4 (figure out how to load data)
        load_data(register4);
        #(CLK_PERIOD);
        load_instruction(32'b0100000_00100_00011_000_00101_0110011, 1, exp_diff); //subtract register 3 - 4, store in register 5
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_diff); //read data from register 3
    endtask

    task test_xor (input [31:0] register1, register2, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0100000_00010_00001_100_00011_0110011, 1, exp_result); //XOR register 1 & 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_or (input [31:0] register1, register2, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0100000_00010_00001_110_00011_0110011, 1, exp_result); //XOR register 1 & 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_and (input [31:0] register1, register2, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0100000_00010_00001_111_00011_0110011, 1, exp_result); //XOR register 1 & 2, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_sll (input [31:0] register1, register2, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_001_00011_0110011, 1, exp_result); //sll register 1 & 2, store in register 3
                                                                                    // rd = rs1 << rs2
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_srl (input [31:0] register1, register2, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000011_00100_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_101_00011_0110011, 1, exp_result); //sll register 1 & 2, store in register 3
                                                                                    // rd = rs1 >> rs2
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    // Immediate Commands
    task add_imm (input [31:0] register1, exp_result);
        // $display("Now adding %d + %d = %d", register1, register2, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_000_00011_0010011, 1, exp_result); //add register 1 & imm, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task add_imm_cons (input [31:0] register1, exp_sum1, exp_sum2);
        // $display("Now adding %d + %d = %d", register1, register2, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_sum1); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_000_00011_0010011, 1, exp_sum1); //add register 1 & imm, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00011_000_00100_0010011, 1, exp_sum2); //add register 3 & imm, store in register 4
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00100_00010_010_00001_0100011, 0, exp_sum2); //read data from register 3
    endtask

    task test_xor_imm (input [31:0] register1, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00001_00001_100_00011_0010011, 1, exp_result); //XOR register 1 - imm, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

  	task test_or_imm (input [31:0] register1, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
    	load_instruction(32'b0000000_00001_00001_110_00011_0010011, 1, exp_result); //OR register 1 & imm, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_and_imm (input [31:0] register1, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
      	load_instruction(32'b0000000_00001_00001_111_00011_0010011, 1, exp_result); //AND register 1 & imm, store in register 3
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_sll_imm_full (input [31:0] register1, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_11111_00001_001_00011_0010011, 1, exp_result); //sll register 1 & imm, store in register 3
                                                                                    // rd = rs1 << imm
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_sll_imm_half (input [31:0] register1, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_10000_00001_001_00011_0010011, 1, exp_result); //sll register 1 & imm, store in register 3
                                                                                    // rd = rs1 << imm
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_sll_imm_one (input [31:0] register1, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00001_00001_001_00011_0010011, 1, exp_result); //sll register 1 & imm, store in register 3
                                                                                    // rd = rs1 << imm
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_srl_imm_full (input [31:0] register1, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b0000001_11111_00001_101_00011_0010011, 1, exp_result); //sll register 1 & imm, store in register 3
                                                                                    // rd = rs1 << imm
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_srl_imm_half (input [31:0] register1, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_10000_00001_101_00011_0010011, 1, exp_result); //sll register 1 & imm, store in register 3
                                                                                    // rd = rs1 << imm
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_srl_imm_one (input [31:0] register1, exp_result);
        load_instruction(32'b000000000011_00100_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00001_00001_101_00011_0010011, 1, exp_result); //sll register 1 & imm, store in register 3
                                                                                    // rd = rs1 << imm
        #(CLK_PERIOD);
        load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    // Branch tasks
    task test_beq (input [31:0] register1, register2, exp_result);
        load_instruction(32'b000000000001_00001_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000001_00001_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_000_00001_1100011, 0, exp_result);
        // #(CLK_PERIOD);
        // load_instruction(32'b0000011_00011_00010_010_00001_0100011, 0, exp_result); //read data from register 3
    endtask

    task test_bneq (input [31:0] register1, register2, exp_result);
        load_instruction(32'b000000000001_00001_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000001_00001_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_001_00001_1100011, 0, exp_result);
    endtask

    task test_blt (input [31:0] register1, register2, exp_result);
        load_instruction(32'b000000000001_00001_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000001_00001_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_100_00001_1100011, 0, exp_result);
    endtask

    task test_bge (input [31:0] register1, register2, exp_result);
        load_instruction(32'b000000000001_00001_010_00001_0000011, 0, exp_result); //load data into register 1 (figure out how to load data)
        load_data(register1);
        #(CLK_PERIOD);
        load_instruction(32'b000000000001_00001_010_00010_0000011, 0, exp_result); //load data into register 2 (figure out how to load data)
        load_data(register2);
        #(CLK_PERIOD);
        load_instruction(32'b0000000_00010_00001_101_00001_1100011, 0, exp_result);
    endtask

    // jump and link
    task test_jal;
        // imm[20|10:1|11|19:12]  |  rd  | opcode
        load_instruction(32'b00000000011000000000_00010_1101111, 0, 0); 
        #(CLK_PERIOD);
        // imm [11:5]   | rs2  | rs1  | funct3  | imm[4:0]  |  opcode
        // load_instruction(32'b0000000_00011_00010_010_00001_0100011, 0, 0); //read data from register 3
    endtask

    // jump and link reg
    task test_jalr;
        // imm[11:0]  |  rs1  |  funct3  |  rd  |  opcode
        load_instruction(32'b000000000110_00001_000_00010_1100111, 0, 0); 
    endtask

    // lui
    task check_lui;
        // imm [31:12]    |    rd    |   opcode
        load_instruction(32'b000000000000000011_00010_0110111, 0, 0);
        // #(CLK_PERIOD);
        // imm [11:5]   | rs2  | rs1  | funct3  | imm[4:0]  |  opcode
        load_instruction(32'b0000000_00011_00010_010_00011_0100011, 0, 0); //read data from register 3
    endtask
endmodule