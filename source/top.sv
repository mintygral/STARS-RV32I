`default_nettype none
// Empty top module

module top (
  // I/O ports
  input  logic hz100, reset,
  input  logic [20:0] pb,
  output logic [7:0] left, right,
         ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0,
  output logic red, green, blue,

  // UART ports
  output logic [7:0] txdata,
  input  logic [7:0] rxdata,
  output logic txclk, rxclk,
  input  logic txready, rxready
);

  logic [31:0] data_in_BUS, pc_data, temp; //input data from memory bus
  logic strobe, branch_ff; //input from memory bus
  logic [31:0] data_out_BUS, address_out, reg_write, result, register_out, register_out_2; //output data +address to memory bus
  logic [31:0] memory_address_out, imm_32_x;
  logic [3:0] key_button;

  /**edge_detector dec(
    .button_sync(pb[0]),
    .clk(hz100),
    .nrst(!reset),
    .posedge_button(strobe)
  );**/

  synckey sync(
    .in(pb[0]),
    .clock(hz100),
    .reset(reset),
    .strobe(strobe)
  );

    logic temp1, temp2, temp3, temp4;
  cpu_core core0(
    .data_in_BUS(data_in_BUS),
    .pc_data(pc_data),
    .bus_full(1'b0),
    .clk(strobe),
    .rst(reset),
    .data_out_BUS(data_out_BUS),
    .address_out(address_out),
    .reg_write(reg_write),
    .result(result), 
    .instruction_x(temp),
    .data_good_x(temp1),
    .instr_fetch_x(temp2),
    .instr_wait_x(temp3),
    .reg_write_en_x(temp4),
    .register_out_x(register_out),
    .register_out_x_2(register_out_2),
    .imm_32_x(imm_32_x),
    .branch_ff(branch_ff)
  );

  logic [11:0] address_real;

  always_comb begin
    if(address_out[11] == 1'b1) begin
      address_real = {2'b11, address_out[11:2]};
    end else begin
      address_real = {2'b00, address_out[11:2]};
    end
  end

  ram mem(
    .clk(strobe),
    .address_data(address_real),
    .address_instr(address_real),
    .data_in(data_out_BUS),
    .keyboard_in({4'b0, key_button}),
    .write_enable(1'b0),
    .addr_out(memory_address_out),
    .instr_out(data_in_BUS)
  );

  keypad_interface keypad0(
    .clk(hz100),
    .rst(reset),
    .columns({pb[13], pb[15], pb[17], pb[19]}),
    .rows({right[1], right[3], right[5], right[7]}),
    .out(key_button)
  );

  logic [127:0] row_1, row_2;
  logic lcd_en, lcd_rw;
  reg lcd_rs;
  reg [7:0] lcd_data;

  lcd_controller lcd_display(.clk(hz100), 
                               .rst(reset),
                               .row_1({data_in_BUS, 96'b0}),
                               .row_2({data_in_BUS, 96'b0}),
                               .lcd_en(green),
                               .lcd_rw(right[2]),
                               .lcd_rs(right[4]),
                               .lcd_data(left[7:0]));

  //assign right[7:0] = address_real[7:0];
//   assign left[6:3] = key_button;
  //{result[7:0], register_out[7:0], register_out_2[7:0], imm_32_x[7:0]}
  display displaying(.seq({result[7:0], register_out[7:0], register_out_2[7:0], imm_32_x[7:0]}), .ssds({ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0}));

endmodule

// Add more modules down here...
module edge_detector(
    input logic button_sync, clk, nrst,
    output logic posedge_button
);

    logic [1:0] flipflops;

    always_ff @(posedge clk, negedge nrst) begin
        if(!nrst) begin
            posedge_button <= 1'b0;
            flipflops <= 2'b0;
        end else begin
            flipflops[1] <= flipflops[0];
            flipflops[0] <= button_sync;
            if(flipflops[0] & ~flipflops[1]) posedge_button <= 1'b1;
            else posedge_button <= 1'b0;
        end
    end

endmodule

module synckey(
    input logic in,
    input logic clock, reset,
    //output logic [4:0] out,
    output logic strobe
);

    //logic [5:0] i;
    logic [1:0] flipflops;
    logic button_pressed;
    
    always_ff @(posedge clock, posedge reset) begin
        if(reset) begin
            flipflops <= 2'b00;
        end else begin  
            flipflops[1] <= flipflops[0];
            flipflops[0] <= button_pressed;
        end
    end
    
    always_comb begin
        button_pressed = in;
        //out = 0;
        /**for (integer i = 0; i < 20; i++) begin
            if(in[i])
                out = i[4:0];
        end**/
        strobe = flipflops[1];
    end

endmodule


module display(
    input logic [31:0] seq,
    output logic [63:0] ssds
);

    //logic [63:0] ssdec_data = ssds;
    logic [7:0] enable = 8'hff;

    ssdec sdd7(.in(seq[31:28]), .enable(enable[7]), .out({ssds[62:56]}));
    ssdec sdd6(.in(seq[27:24]), .enable(enable[6]), .out({ssds[54:48]}));
    ssdec sdd5(.in(seq[23:20]), .enable(enable[5]), .out({ssds[46:40]}));
    ssdec sdd4(.in(seq[19:16]), .enable(enable[4]), .out({ssds[38:32]}));
    ssdec sdd3(.in(seq[15:12]), .enable(enable[3]), .out({ssds[30:24]}));
    ssdec sdd2(.in(seq[11:8]), .enable(enable[2]), .out({ssds[22:16]}));
    ssdec sdd1(.in(seq[7:4]), .enable(enable[1]), .out({ssds[14:8]}));
    ssdec sdd0(.in(seq[3:0]), .enable(enable[0]), .out({ssds[6:0]}));

endmodule

module ram (
    input logic clk,
    input logic [11:0] address_data, address_instr,
    input logic [31:0] data_in,
    input logic write_enable,
    input logic [7:0] keyboard_in,
    output logic [31:0] addr_out,
    output logic [31:0] instr_out
);

reg[31:0] memory [4095:0]; //6 bytes of reserved data
logic [31:0] output_word;

//reserved memory for I/O
//[4095:4092] -> LCD screen data out
//[4091] -> keyboard inputs
//[4090] -> temp input

initial begin
    $readmemh("cpu.mem", memory);
end

always_comb begin
    if(address_instr != 12'd25) begin
        output_word = memory[address_instr];
    end else begin
        case(address_instr)
            (12'd25): output_word = {24'b0, keyboard_in};
            default: output_word = 32'b0;
        endcase
    end
end

always_ff @(posedge clk) begin
    if(write_enable) begin
        memory[address_data] <= data_in;
    end
    addr_out <= memory[address_data];
    instr_out <= output_word;
end

endmodule

module ssdec(
    input logic [3:0] in,
    input logic enable,
    output logic [6:0] out
);

    assign out[0] = enable & (~in[3] & in[1] | in[3] & ~in[0] | ~in[2] & ~in[0] | in[2] & in[1] | in[3] & ~in[2] & ~in[1] | ~in[3] & in[2] & in[0]);
    assign out[1] = enable & (~in[3] & ~in[2] | ~in[2] & ~in[1] | ~in[2] & ~in[0] | ~in[3] & in[1] & in[0] | ~in[3] & ~in[1] & ~in[0] | in[3] & ~in[1] & in[0]);
    assign out[2] = enable & (in[3] & ~in[2] | ~in[3] & in[2] | ~in[1] & in[0] | ~in[3] & ~in[1] | ~in[3] & in[0]);
    assign out[3] = enable & (~in[2] & in[1] & in[0] | ~in[3] & in[1] & ~in[0] | in[2] & ~in[1] & in[0] | ~in[2] & ~in[1] & ~in[0] | in[3] & in[2] & ~in[0]);
    assign out[4] = enable & (in[3] & in[2] | in[1] & ~in[0] | ~in[2] & ~in[0] | in[3] & in[1]);
    assign out[5] = enable & (in[3] & ~in[2] | ~in[1] & ~in[0] | in[3] & in[1] | in[2] & ~in[0] | ~in[3] & in[2] & ~in[1]);
    assign out[6] = enable & (in[3] & ~in[2] | in[1] & ~in[0] | ~in[2] & in[1] | ~in[3] & in[2] & ~in[1] | in[3] & in[2] & in[0]);

endmodule

typedef enum logic [2:0] {
    INIT = 0,
    IDLE = 1,
    Read = 4,
    Write = 5,
    Wait = 6
} state_t;

module cpu_core(
    input logic [31:0] data_in_BUS, pc_data,//input data from memory bus, memory starting point
    input logic bus_full, 
    output logic data_good_x, instr_fetch_x, instr_wait_x, reg_write_en_x, //input from memory bus
    input logic clk, rst, //external clock, reset
    output logic [31:0] data_out_BUS, address_out, reg_write, result, instruction_x, register_out_x, register_out_x_2, imm_32_x, //output data +address to memory bus
    output logic branch_ff
);

    assign {data_good_x, instr_fetch_x, instr_wait_x} = {data_good, instr_fetch, instr_wait};
    assign instruction_x = pc_val;
    assign reg_write_en_x = reg_write_en;
    assign {register_out_x, register_out_x_2} = {reg1, reg2};
    assign imm_32_x = imm_32;
    logic memToReg_flipflop, instr_wait;

    //Instruction Memory -> Control Unit
    logic [31:0] instruction;

    //Control Unit -> ALU
    logic [6:0] funct7, opcode;
    logic [2:0] funct3;
    logic ALU_source; //0 means register, 1 means immediate
    
    //Control Unit -> ALU + Program Counter
    logic [31:0] imm_32;

    //Control Unit -> Registers
    logic [4:0] rs1, rs2, rd;
    
    //Control Unit -> Data Memory
    logic memToReg; //0 means use ALU output, 1 means use data from memory

    //Control Unit -> Program Counter
    logic load_pc; //0 means leave pc as is, 1 means need to load in data

    //Data Memory -> Registers
    //logic [31:0] reg_write;

    //Register Input (double check where its coming from)
    logic reg_write_en;

    //Registers -> ALU
    logic [31:0] reg1, reg2;//, result;

    //ALU -> Data Memory
    logic [31:0] read_address, write_address;//, result;

    //ALU -> Program Counter
    logic branch;

    //Memcontrol
    logic [31:0] address_in, data_in_CPU;
    logic data_en, instr_en, memWrite, memRead;

    // outputs
    state_t state; //not currently used, it's just kind of there rn
    logic [31:0] data_out_CPU, data_out_INSTR;
    
    //Program Counter
    logic inc;

    //Data Memory
    logic [31:0] data_read_adr_i, data_write_adr_i, data_bus_i;
    logic data_good, bus_full_CPU;
    logic data_read, data_write;
    logic [31:0] data_adr_o, data_bus_o, data_cpu_o;

    //(ALU or external reset) -> Program Counter 
    //logic [31:0] pc_data; //external reset value only now

    //Program Counter -> Instruction Memory
    logic [31:0] pc_val;

    //Memory Manager -> Instruction Memory
    logic [31:0] instruction_i;

    //Instruction Memory -> Memory Manager
    logic instr_fetch;
    logic [31:0] instruction_adr_o; 

    logic [31:0] mem_adr_i, pc_jump;
    logic mem_read;
    
    always_comb begin
        mem_adr_i = (data_adr_o == 32'b0) ? instruction_adr_o : data_adr_o;
        data_en = data_read | data_write;
        mem_read = data_read | instr_fetch;
        instr_wait = ((~(read_address == 32'b0) | ~(write_address == 32'b0)) & ~data_good);
    end

    logic [31:0] load_data_flipflop, reg_write_flipflop;

    always_ff @(posedge clk) begin
        memToReg_flipflop <= memToReg;
        reg_write_flipflop <= reg_write;
        load_data_flipflop <= data_cpu_o;
    end


    instruction_memory instr_mem(
        .instruction_adr_i(pc_val),
        .instruction_i(data_out_INSTR),
        .clk(clk),
        .data_good(data_good),
        .rst(rst),
        .instr_fetch(instr_fetch),
        .instruction_adr_o(instruction_adr_o),
        .instruction_o(instruction),
        .instr_wait(instr_wait));
    
    control_unit ctrl(
        .instruction(instruction), 
        .opcode(opcode), 
        .funct7(funct7), 
        .funct3(funct3), 
        .rs1(rs1), 
        .rs2(rs2), 
        .rd(rd), 
        .imm_32(imm_32), 
        .ALU_source(ALU_source),
        .memToReg(memToReg),
        .load(load_pc));

        //assign result = imm_32;

    //multiplexer for register input
    always_comb begin
        if((opcode != 7'b0100011) && (opcode != 7'b1100011)) begin
            if(memToReg_flipflop == 1'b1) reg_write = (load_data_flipflop | data_cpu_o);
            else reg_write = result;
            reg_write_en = (!instr_fetch) ? 1'b1 : 1'b0;
        end else begin
            reg_write = 32'b0;
            reg_write_en = 1'b0;
        end
    end

    logic [31:0] register_out;
    
    register_file regFile(
        .reg_write(reg_write | reg_write_flipflop), 
        .clk(clk), 
        .rst(rst), 
        .write(reg_write_en), 
        .rd(rd),
        .rs1(rs1), 
        .rs2(rs2),
        .reg1(reg1),
        .reg2(reg2),
        .register_out(register_out));
 
    logic branch_temp;
    ALU math(
        .ALU_source(ALU_source), 
        .opcode(opcode), 
        .funct3(funct3), 
        .funct7(funct7), 
        .reg1(reg1), 
        .reg2(reg2), 
        .immediate(imm_32),
        .pc_val(pc_val), 
        .read_address(read_address), 
        .write_address(write_address), 
        .result(result), 
        .branch(branch),
        .pc_data(pc_jump));

    always_comb begin
        data_good = !bus_full_CPU & (state == Read | state == Write);
    end

    logic [31:0] val2;
    //logic branch_ff;
    always_comb begin 
        val2 = reg2;
        branch_ff = ((opcode == 7'b1100011) && ((funct3 == 3'b000 && (reg1 == val2)) | (funct3 == 3'b100 && (reg1 < val2)) | (funct3 == 3'b001 && (reg1 != val2)) | (funct3 == 3'b101 && (reg1 >= val2)))) | (opcode == 7'b1101111) | (opcode == 7'b1100111);
    end

    //sort through mem management inputs/outputs
    data_memory data_mem(
        .data_read_adr_i(read_address),
        .data_write_adr_i(write_address),
        .data_cpu_i(reg2),
        .data_bus_i(data_out_CPU),
        .clk(clk),
        .rst(rst),
        .data_good(data_good),
        .data_read(data_read),
        .data_write(data_write),
        .data_adr_o(data_adr_o),
        .data_bus_o(data_bus_o),
        .data_cpu_o(data_cpu_o));

    //need to figure out these inputs
    memcontrol mem_ctrl(
        .address_in(mem_adr_i), //only works if non-active addresses are set to 0 
        .data_in_CPU(data_bus_o),
        .data_in_BUS(data_in_BUS), //external info
        .data_en(data_en),
        .instr_en(instr_fetch),
        .bus_full(bus_full), //external info
        .memWrite(data_write),
        .memRead(mem_read),
        .clk(clk),
        .rst(rst),
        // outputs
        .state(state),
        .address_out(address_out), //to external output
        .data_out_CPU(data_out_CPU), //to data mem
        .data_out_BUS(data_out_BUS), //to external output
        .data_out_INSTR(data_out_INSTR), //to instr mem
        .bus_full_CPU(bus_full_CPU)); 

    // assign address_out = mem_adr_i;

    pc program_count(
        .clk(clk),
        .clr(rst),
        .load(load_pc),
        .inc(data_good & instr_fetch),
        .ALU_out(branch_ff),
        .Disable(instr_wait),
        .data(pc_data | pc_jump),
        .imm_val(imm_32),
        .pc_val(pc_val));

endmodule

module ALU(
    input logic ALU_source,
    input logic [6:0] opcode,
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic [31:0] reg1, reg2, immediate, pc_val,
    output logic [31:0] read_address, write_address, result, pc_data,
    output logic branch
);

    logic [31:0] val2;


    always_comb begin
        if (ALU_source) begin
            val2 = immediate;
        end else begin
            val2 = reg2;
        end end
        

    always_comb begin
        pc_data = 32'b0;
        read_address = 32'b0;
        write_address = 32'b0; 
        result = 32'b0;
        branch = 1'b0;
        //len = val2-1;
        case(opcode)
            7'b0000011:
                read_address = reg1 + val2;
            7'b0100011:
                begin
                    write_address = reg1 + val2;
                end
            7'b0110011, 7'b0010011:
                begin
                    case(funct3)
                        3'b000, 3'b010: begin
                            if (funct7==7'b0100000) begin //subtract based on f7
                                result = reg1-val2;
                            end else begin
                                result = reg1+val2;
                            end
                        end
                        3'b100: result = reg1^val2;
                        3'b110: result = reg1|val2;
                        3'b111: result = reg1&val2;
                        3'b001: result = reg1 << val2[4:0];
                        3'b101: result = reg1 >> val2[4:0];
                        default: begin
                            result=32'b0;
                            read_address=32'b0;
                            write_address=32'b0;
                        end
                    endcase 
                end
            7'b1100011:
                begin
                    case(funct3)
                        3'b000: begin //branch ==
                            if (reg1 == val2) branch=1'b1;
                            else branch=1'b0;
                        end
                        3'b001:  begin //branch !=
                            if (reg1!=val2) branch=1'b1;
                            else branch=1'b0;
                        end
                        3'b100:  begin //branch <
                            if (reg1<val2) branch=1'b1;
                            else branch=1'b0;
                        end
                        3'b101: begin //branch >=
                            if (reg1>=val2) branch=1'b1;
                            else branch=1'b0;
                        end
                        default: branch=1'b0;
                    endcase 
                end
            7'b1101111:
              begin
                branch = 1'b1;
                result = pc_val + 32'd4;
              end
            7'b1100111:
              begin 
                branch=1'b1;//jump and link, jalr
                result = pc_val + 32'd4;
                pc_data = reg1 + val2;
              end
            7'b0110111: result = {val2[19:0],12'b0}; // lui
            default: 
                begin
                    read_address = 32'b0;
                    write_address = 32'b0; 
                    result = 32'b0;
                    branch = 1'b0;
                end 
        endcase
    end
endmodule


module control_unit(
    input logic [31:0] instruction,
    output logic [6:0] opcode, funct7,
    output logic [2:0] funct3,
    output logic [4:0] rs1, rs2, rd,
    output logic [31:0] imm_32,
    output logic ALU_source, //0 means register, 1 means immediate
    output logic memToReg, //0 means use ALU output, 1 means use data from memory
    output logic load //0 means leave pc as is, 1 means need to load in data
);

    always_comb begin
        opcode = instruction[6:0];
        rd = 5'b0;
        imm_32 = 32'h00000000;
        rs1 = 5'b0;
        rs2 = 5'b0;
        funct3 = 3'b0;
        funct7 = 7'b0;
        ALU_source = 1'b0;
        memToReg = 1'b0;
        load = 1'b0;
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
                    load = 1'b0;
                end
            7'b0010011, //i type instructions
            7'b0000011,
            7'b1100111:
                begin
                    funct3 = instruction[14:12];
                    rd = instruction[11:7];
                    rs1 = instruction[19:15];
                    if(instruction[31] == 1'b0) begin
                      imm_32 = {20'b0, instruction[31:20]};
                    end else begin
                      imm_32 = {20'hfffff, instruction[31:20]};
                    end 
                    funct7 = 7'b0;
                    rs2 = 5'b0;
                    ALU_source = 1'b1;
                    memToReg = (opcode == 7'b0000011) ? 1'b1 : 1'b0;
                    load = (opcode == 7'b1100111) ? 1'b1 : 1'b0;
                end
            7'b0100011: //s type instructions
                begin
                    funct3 = instruction[14:12];
                    rs1 = instruction[19:15];
                    rs2 = instruction[24:20];
                    if(instruction[31] == 1'b0) begin
                      imm_32 = {20'b0, instruction[31:25], instruction[11:7]};
                    end else begin
                      imm_32 = {20'hfffff, instruction[31:25], instruction[11:7]};
                    end 
                    funct7 = 7'b0;
                    rd = 5'b0;
                    ALU_source = 1'b1;
                    memToReg = 1'b0;
                    load = 1'b0;
                end
            7'b1100011: //b type instruction
                begin
                    funct3 = instruction[14:12];
                    rs1 = instruction[19:15];
                    rs2 = instruction[24:20];
                    if(instruction[31] == 1'b0) begin
                      imm_32 = {20'b0, instruction[31], instruction[7], instruction[30:25], instruction[11:8]};
                    end else begin
                      imm_32 = {20'hfffff, instruction[31], instruction[7], instruction[30:25], instruction[11:8]};
                    end 
                    funct7 = 7'b0;
                    rd = 5'b0;
                    ALU_source = 1'b1;
                    memToReg = 1'b0;
                    load = 1'b0;
                end
            7'b1101111: //j type instruction
                begin
                    rd = instruction[11:7];
                    if(instruction[31] == 1'b0) begin
                      imm_32 = ({12'b0, instruction[31], instruction[19:12], instruction[20], instruction[30:21]} << 1) - 32'd4;
                    end else begin
                      imm_32 = ({12'hfff, instruction[31], instruction[19:12], instruction[20], instruction[30:21]<< 1}) - 32'd4;
                    end 
                    rs1 = 5'b0;
                    rs2 = 5'b0;
                    funct3 = 3'b0;
                    funct7 = 7'b0;
                    ALU_source = 1'b1;
                    memToReg = 1'b0;
                    load = 1'b0;
                end
            7'b0110111: //u type instruction
                begin
                    rd = instruction[11:7];
                    if(instruction[31] == 1'b0) begin
                      imm_32 = {12'b0, instruction[31:12]};
                    end else begin
                      imm_32 = {12'hfff, instruction[31:12]};
                    end 
                    rs1 = 5'b0;
                    rs2 = 5'b0;
                    funct3 = 3'b0;
                    funct7 = 7'b0;
                    ALU_source = 1'b1;
                    memToReg = 1'b0;
                    load = 1'b0;
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
                    load = 1'b0;
                end
        endcase
    end
endmodule

module data_memory(
    input logic [31:0] data_read_adr_i, data_write_adr_i, data_bus_i, data_cpu_i,
    input logic clk, data_good, rst,
    output logic data_read, data_write,
    output logic [31:0] data_adr_o, data_bus_o, data_cpu_o
);

    logic next_read, next_write;
    logic [31:0] stored_read_data, stored_write_data, stored_data_adr;

    always_comb begin
        next_read = 1'b0;
        next_write = 1'b0;
        stored_read_data = 32'b0;
        stored_write_data = 32'b0;
        data_adr_o = data_read_adr_i | data_write_adr_i;
        data_cpu_o = data_bus_i;
        data_bus_o = data_cpu_i;
        if(~(data_read_adr_i == 32'b0)) begin
            if(data_good & data_read) begin
                next_read = 1'b0;
            end else begin
                next_read = 1'b1;
            end
        end else if(~(data_write_adr_i == 32'b0)) begin
            if(data_good & data_write) begin
                next_write = 1'b0;
            end else begin
                next_write = 1'b1;
            end
        end
    end

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            //data_adr_o <= 32'b0;
            //data_bus_o <= 32'b0;
            //data_cpu_o <= 32'b0;
            data_read <= 1'b0;
            data_write <= 1'b0;
        end else begin
            data_read <= next_read;
            data_write <= next_write;
            //data_adr_o <= stored_data_adr;
            //data_cpu_o <= stored_read_data;
            //data_bus_o <= stored_write_data;
        end
    end
endmodule

module instruction_memory(
    input logic [31:0] instruction_adr_i, instruction_i,
    input logic clk, data_good, rst, instr_wait,
    output logic instr_fetch,
    output logic [31:0] instruction_adr_o, instruction_o
);

    logic next_fetch;
    logic [31:0] stored_instr, stored_instr_adr;

    always_comb begin
        next_fetch = 1'b0;
        if(data_good & instr_fetch) begin
            next_fetch = 1'b0;
            stored_instr_adr = instruction_adr_i;
            stored_instr = instruction_i;
        end else if(!instr_wait) begin
            next_fetch = 1'b1;
            stored_instr_adr = instruction_adr_i;
            stored_instr = 32'b0;
        end else begin
            next_fetch = 1'b0;
            stored_instr_adr = instruction_adr_i;
            stored_instr = instruction_i;
        end
    end

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            instruction_adr_o <= 32'b0;
            instruction_o <= 32'b0;
            instr_fetch <= 1'b0;
        end else if(instr_wait) begin
            instruction_adr_o <= instruction_adr_o;
            instruction_o <= instruction_o;
            instr_fetch <= 1'b0;
        end else begin
            instruction_adr_o <= stored_instr_adr;
            instruction_o <= stored_instr;
            instr_fetch <= next_fetch;
        end
    end
endmodule

module memcontrol(
    // inputs
    // data_in_BUS and bus_full are the only inputs from the bus manager, so we need to figure those out on wednesday
    input logic [31:0] address_in, data_in_CPU, data_in_BUS,
    input logic data_en, instr_en, bus_full, memWrite, memRead,
    input logic clk, rst,
    // outputs
    output state_t state,
    output logic bus_full_CPU,
    output logic [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR
);

    state_t next_state, prev_state;

    always_ff @(posedge clk, posedge rst) begin : startFSM
        if (rst) begin
            state <= INIT;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin : changeState
        bus_full_CPU = bus_full;
        // garbage values for testing
        address_out = address_in;
        data_out_BUS = 32'h0;
        data_out_CPU = 32'h0;
        data_out_INSTR = 32'h0;
        next_state = state;
        prev_state = state;
        case(state)
            INIT: begin 
                if (!rst) next_state = IDLE;
                else next_state = INIT;
            end
            
            IDLE: begin
                if (memRead) begin
                    next_state = Read;
                    prev_state = Read;
                end else if (memWrite) begin
                    next_state = Write;
                    prev_state = Write;
                end else if (prev_state == Read | prev_state == Write) begin
                    address_out = address_in;
                    prev_state = IDLE;
                end else begin
                    prev_state = IDLE;
                    next_state = IDLE;
                    address_out = 32'b0;
                end
            end
            
            Read: begin 
                address_out = address_in;
                data_out_BUS = 32'b0; 
                if (data_en) begin
                    data_out_CPU = data_in_BUS;
                    data_out_INSTR = 32'b0; // going to MUX
                end
                else if (instr_en) begin
                    data_out_CPU = 32'b0;
                    data_out_INSTR = data_in_BUS; // going to CU
                end
                next_state = IDLE;
            end
            
            Write: begin 
                address_out = address_in;
                data_out_BUS = data_in_CPU;
                data_out_INSTR = 32'b0;
                data_out_CPU = 32'b0;
                next_state = IDLE; 
            end

            Wait: begin 
                if (!bus_full) begin
                    if (prev_state == Read) begin
                        next_state = Read;
                    end else if (prev_state == Write) begin
                        next_state = Write;
                    end else begin
                        next_state = IDLE;
                    end
                end else begin
                    next_state = Wait;
                end
            end

            default: next_state = IDLE;
            
        endcase
    end
endmodule

module pc(
    input logic clk, clr, load, inc, Disable, ALU_out,
    input logic [31:0] data, imm_val,
    output logic [31:0] pc_val 
);
    logic [31:0] next_line_ad;
    logic [31:0] jump_ad;
    logic [31:0] next_pc;
    logic branch_choice;

    // Register 
    always_ff @(negedge clk, posedge clr) begin

        if (clr) begin
            pc_val <= 32'd0;
        end

        else begin
            pc_val <= next_pc;
        end
    end


   always_comb begin
       next_pc = pc_val;
       next_line_ad = pc_val + 32'd4;	// Calculate next line address  
       jump_ad = pc_val + imm_val;    // Calculate jump address (jump and link)

	
        // Mux choice between next line address and jump address
        if (Disable) begin 
		      next_pc = pc_val; 
	      end

        else if (load) begin
          next_pc = data + next_line_ad;
        end
            
        else if (ALU_out) begin
		      next_pc = jump_ad;
	      end
	
        else if (inc) begin
          next_pc = next_line_ad;
        end

   end       
endmodule

module register_file (
    input logic [31:0] reg_write, 
    input logic [4:0] rd, rs1, rs2, 
    input logic clk, rst, write,
    output logic [31:0] reg1, reg2,
    output logic [31:0] register_out//array????
);
    reg[31:0][31:0] register;
    //reg[31:0][31:0] next_register; 

    logic [31:0] write_data;

    //assign register = '{default:'0};

    always_comb begin
        write_data = reg_write;
        if (write) begin
            if (rd != 0) begin
                write_data = reg_write;
            end else begin
                write_data = 32'b0;
            end
        end
        reg1 = register[rs1];
        reg2 = register[rs2];
        register_out = register[5'd2];
    end

    always_ff @ (posedge clk, posedge rst) begin //reset pos or neg or no reset
        if (rst) begin
            register <= '0;
        end
        else begin
            //register <= next_register;
            if(write) begin
                register[rd] <= write_data;
            end
        end
    end
endmodule


typedef enum {KEY_IDLE, SCAN} key_state;

module keypad_interface(
    input logic clk, rst,
    input logic [3:0] columns,
    output logic [3:0] rows,
    output logic [3:0] out
);

    logic [7:0] code;
    key_state state, next_state;
    logic [3:0] next_rows, next_out;


    always_comb begin
        code = {columns, rows};
        next_rows = rows;
        next_out = out;
        /**if(state == KEY_IDLE) begin
            if(columns != 4'b0000) begin
                next_state = SCAN;
                next_rows = 4'b1110;
            end
            else next_state = KEY_IDLE;
        end else begin**/
            case(rows)
                4'b1110:
                    begin
                        case(columns)
                            4'b0001: next_out = 4'b0001;
                            4'b0010: next_out = 4'b0010;
                            4'b0100: next_out = 4'b0011;
                            4'b1000: next_out = 4'b1010;
                            default: next_out = out;
                        endcase
                        next_rows = 4'b1101;
                        next_state = SCAN;
                    end
                4'b1101:
                    begin
                        case(columns)
                            4'b0001: next_out = 4'b0100;
                            4'b0010: next_out = 4'b0101;
                            4'b0100: next_out = 4'b0110;
                            4'b1000: next_out = 4'b1011;
                            default: next_out = out;
                        endcase
                        next_rows = 4'b1011;
                        next_state = SCAN;
                    end
                4'b1011:
                    begin
                        case(columns)
                            4'b0001: next_out = 4'b0111;
                            4'b0010: next_out = 4'b1000;
                            4'b0100: next_out = 4'b1001;
                            4'b1000: next_out = 4'b1100;
                            default: next_out = out;
                        endcase
                        next_rows = 4'b0111;
                        next_state = SCAN;
                    end
                4'b0111:
                    begin
                        case(columns)
                            4'b0001: next_out = 4'b1110;
                            4'b0010: next_out = 4'b0000;
                            4'b0100: next_out = 4'b1111;
                            4'b1000: next_out = 4'b1101;
                            default: next_out = out;
                        endcase
                        next_rows = 4'b1110;
                        next_state = SCAN;
                    end
                default:
                  begin
                    next_state = SCAN;
                    next_rows = 4'b1110;
                  end
            endcase
        //end
    end

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            rows <= 4'b1110;
            state <= KEY_IDLE;
            out <= 4'b0000;
        end else begin
            rows <= next_rows;
            state <= next_state;
            out <= next_out;
        end
    end
endmodule

module lcd_controller #(parameter clk_div = 24_000)(
    input clk,
    input rst,
    // Data to be displayed
    input [127:0] row_1,
    input [127:0] row_2,
   
    // LCD control signal
    output lcd_en,
    output lcd_rw,
    output reg lcd_rs,
    output reg [7:0] lcd_data
    );

    logic lcd_ctrl; // added declaration

    reg [7:0] currentState; // updated bits from 6 to 8
    reg [7:0] nextState; // updated bits from 6 to 8
    reg [17:0] cnt_20ms;
    reg [14:0] cnt_500hz;
    wire delay_done;
 
    localparam TIME_500HZ = clk_div;
    // Wait for 20 ms before intializing.
    localparam TIME_20MS = TIME_500HZ * 10;
   
    // Set lcd_data accroding to datasheet
    localparam LCD_IDLE = 8'h00,                
               SET_FUNCTION = 8'h01,      
               DISP_OFF = 8'h03,
               DISP_CLEAR = 8'h02,
               ENTRY_MODE = 8'h06,
               DISP_ON = 8'h07,
               ROW1_ADDR = 8'h05,      
               ROW1_0 = 8'h04,
               ROW1_1 = 8'h0C,
               ROW1_2 = 8'h0D,
               ROW1_3 = 8'h0F,
               ROW1_4 = 8'h0E,
               ROW1_5 = 8'h0A,
               ROW1_6 = 8'h0B,
               ROW1_7 = 8'h09,
               ROW1_8 = 8'h08,
               ROW1_9 = 8'h18,
               ROW1_A = 8'h19,
               ROW1_B = 8'h1B,
               ROW1_C = 8'h1A,
               ROW1_D = 8'h1E,
               ROW1_E = 8'h1F,
               ROW1_F = 8'h1D,
               ROW2_ADDR = 8'h1C,
               ROW2_0 = 8'h14,
               ROW2_1 = 8'h15,
               ROW2_2 = 8'h17,
               ROW2_3 = 8'h16,
               ROW2_4 = 8'h12,
               ROW2_5 = 8'h13,
               ROW2_6 = 8'h11,
               ROW2_7 = 8'h10,
               ROW2_8 = 8'h30,
               ROW2_9 = 8'h31,
               ROW2_A = 8'h33,
               ROW2_B = 8'h32,
               ROW2_C = 8'h36,
               ROW2_D = 8'h37,
               ROW2_E = 8'h35,
               ROW2_F = 8'h34;

    assign delay_done = (cnt_20ms==TIME_20MS-1) ? 1'b1 : 1'b0;
    always @(posedge clk) begin
        if (!rst) begin
            cnt_20ms <= 0;
        end
        else if (cnt_20ms == TIME_20MS-1) begin
            cnt_20ms <= cnt_20ms;
        end
        else
            cnt_20ms <= cnt_20ms + 1;
    end

    //500HZ for lcd
    always  @(posedge clk) begin
        if(!rst)begin
            cnt_500hz <= 0;
        end
        else if(delay_done)begin
            if(cnt_500hz == TIME_500HZ - 1)
                cnt_500hz <= 0;
            else
                cnt_500hz<=cnt_500hz + 1 ;
        end
        else
            cnt_500hz <= 0;
    end

    assign lcd_en = (cnt_500hz > (TIME_500HZ-1)/2)? 1'b0 : 1'b1;
    assign lcd_ctrl = (cnt_500hz == TIME_500HZ - 1) ? 1'b1 : 1'b0;

    always  @(posedge clk) begin
        if(!rst)
            currentState <= LCD_IDLE;
        else if (lcd_ctrl)
            currentState <= nextState;
        else
            currentState <= currentState;
    end

    always  @(*) begin
        case (currentState)
            LCD_IDLE: nextState = SET_FUNCTION;
            SET_FUNCTION: nextState = DISP_OFF;
            DISP_OFF: nextState = DISP_CLEAR;
            DISP_CLEAR: nextState = ENTRY_MODE;
            ENTRY_MODE: nextState = DISP_ON;
            DISP_ON: nextState = ROW1_ADDR;
            ROW1_ADDR: nextState = ROW1_0;
            ROW1_0: nextState = ROW1_1;
            ROW1_1: nextState = ROW1_2;
            ROW1_2: nextState = ROW1_3;
            ROW1_3: nextState = ROW1_4;
            ROW1_4: nextState = ROW1_5;
            ROW1_5: nextState = ROW1_6;
            ROW1_6: nextState = ROW1_7;
            ROW1_7: nextState = ROW1_8;
            ROW1_8: nextState = ROW1_9;
            ROW1_9: nextState = ROW1_A;
            ROW1_A: nextState = ROW1_B;
            ROW1_B: nextState = ROW1_C;
            ROW1_C: nextState = ROW1_D;
            ROW1_D: nextState = ROW1_E;
            ROW1_E: nextState = ROW1_F;
            ROW1_F: nextState = ROW2_ADDR;
            ROW2_ADDR: nextState = ROW2_0;
            ROW2_0: nextState = ROW2_1;
            ROW2_1: nextState = ROW2_2;
            ROW2_2: nextState = ROW2_3;
            ROW2_3: nextState = ROW2_4;
            ROW2_4: nextState = ROW2_5;
            ROW2_5: nextState = ROW2_6;
            ROW2_6: nextState = ROW2_7;
            ROW2_7: nextState = ROW2_8;
            ROW2_8: nextState = ROW2_9;
            ROW2_9: nextState = ROW2_A;
            ROW2_A: nextState = ROW2_B;
            ROW2_B: nextState = ROW2_C;
            ROW2_C: nextState = ROW2_D;
            ROW2_D: nextState = ROW2_E;
            ROW2_E: nextState = ROW2_F;
            ROW2_F: nextState = ROW1_ADDR;
            default: nextState = LCD_IDLE;
        endcase
    end  

    // LCD control sigal
    assign lcd_rw = 1'b0;
    always  @(posedge clk) begin
        if(!rst) begin
            lcd_rs <= 1'b0;   //order or data  0: order 1:data
        end
        else if (lcd_ctrl) begin
            if((nextState==SET_FUNCTION) || (nextState==DISP_OFF) || (nextState==DISP_CLEAR) || (nextState==ENTRY_MODE)||
                (nextState==DISP_ON ) || (nextState==ROW1_ADDR)|| (nextState==ROW2_ADDR))
                lcd_rs <= 1'b0;
            else
                lcd_rs <= 1'b1;
        end
        else begin
            lcd_rs <= lcd_rs;
        end    
    end                  

    always  @(posedge clk) begin
        if (!rst) begin
            lcd_data <= 8'h00;
        end
        else if(lcd_ctrl) begin
            case(nextState)
                LCD_IDLE: lcd_data <= 8'hxx;
                SET_FUNCTION: lcd_data <= 8'h38; //2 lines and 5×7 matrix
                DISP_OFF: lcd_data <= 8'h08;
                DISP_CLEAR: lcd_data <= 8'h01;
                ENTRY_MODE: lcd_data <= 8'h06;
                DISP_ON: lcd_data <= 8'h0C;  //Display ON, cursor OFF
                ROW1_ADDR: lcd_data <= 8'h80; //Force cursor to beginning of first line
                ROW1_0: lcd_data <= row_1 [127:120];
                ROW1_1: lcd_data <= row_1 [119:112];
                ROW1_2: lcd_data <= row_1 [111:104];
                ROW1_3: lcd_data <= row_1 [103: 96];
                ROW1_4: lcd_data <= row_1 [ 95: 88];
                ROW1_5: lcd_data <= row_1 [ 87: 80];
                ROW1_6: lcd_data <= row_1 [ 79: 72];
                ROW1_7: lcd_data <= row_1 [ 71: 64];
                ROW1_8: lcd_data <= row_1 [ 63: 56];
                ROW1_9: lcd_data <= row_1 [ 55: 48];
                ROW1_A: lcd_data <= row_1 [ 47: 40];
                ROW1_B: lcd_data <= row_1 [ 39: 32];
                ROW1_C: lcd_data <= row_1 [ 31: 24];
                ROW1_D: lcd_data <= row_1 [ 23: 16];
                ROW1_E: lcd_data <= row_1 [ 15:  8];
                ROW1_F: lcd_data <= row_1 [  7:  0];

                ROW2_ADDR: lcd_data <= 8'hC0;      //Force cursor to beginning of second line
                ROW2_0: lcd_data <= row_2 [127:120];
                ROW2_1: lcd_data <= row_2 [119:112];
                ROW2_2: lcd_data <= row_2 [111:104];
                ROW2_3: lcd_data <= row_2 [103: 96];
                ROW2_4: lcd_data <= row_2 [ 95: 88];
                ROW2_5: lcd_data <= row_2 [ 87: 80];
                ROW2_6: lcd_data <= row_2 [ 79: 72];
                ROW2_7: lcd_data <= row_2 [ 71: 64];
                ROW2_8: lcd_data <= row_2 [ 63: 56];
                ROW2_9: lcd_data <= row_2 [ 55: 48];
                ROW2_A: lcd_data <= row_2 [ 47: 40];
                ROW2_B: lcd_data <= row_2 [ 39: 32];
                ROW2_C: lcd_data <= row_2 [ 31: 24];
                ROW2_D: lcd_data <= row_2 [ 23: 16];
                ROW2_E: lcd_data <= row_2 [ 15:  8];
                ROW2_F: lcd_data <= row_2 [  7:  0];
                default: lcd_data <= 8'hxx;
            endcase                    
        end
        else
            lcd_data <= lcd_data;
    end

endmodule