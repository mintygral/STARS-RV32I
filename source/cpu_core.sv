module cpu_core(
    input logic [31:0] data_in_BUS, //input data from memory bus
    output logic [31:0] data_out_BUS //output data to memory bus
);

    //Standard Signals
    logic clk, rst;

    //Instruction Memory -> Control Unit
    logic [31:0] instruction;

    //Control Unit -> ALU
    logic [6:0] opcode, funct7;
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
    logic [31:0] reg_write;

    //Register Input (double check where its coming from)
    logic write;

    //Registers -> ALU
    logic [31:0] reg1, reg2;

    //ALU -> Data Memory
    logic [31:0] read_address, write_address, result;

    //ALU -> Program Counter
    logic branch;

    //NEED TO FIGURE THESE OUT, but in a bit


    //Memcontrol
    input logic [31:0] address_in, data_in_CPU;
    input logic data_en, instr_en, bus_full, memWrite, memRead,

    // outputs
    output state_t state;
    output logic [31:0] address_out, data_out_CPU, data_out_INSTR;
    
    //
    input logic inc, Disable,


    //Data Memory
    input logic [31:0] data_read_adr_i, data_write_adr_i, data_bus_i, data_cpu_i,
    input logic clk, data_good, rst,
    output logic data_read, data_write,
    output logic [31:0] data_adr_o, data_bus_o, data_cpu_o

    //(ALU or external reset) -> Program Counter 
    logic [31:0] load; 

    //Program Counter -> Instruction Memory
    logic [31:0] pc_val;

    //Memory Manager -> Instruction Memory
    logic [31:0] instruction_i;
    logic data_good;

    //Instruction Memory -> Memory Manager
    logic instr_fetch;
    logic [31:0] instruction_adr_o; 

    instruction_memory instr_mem(
        .instruction_adr_i(pc_val),
        .instruction_i(data_out_INSTR),
        .clk(clk),
        .data_good(good_data),
        .rst(rst),
        .instr_fetch(instr_fetch),
        .instruction_adr_o(address_in),
        .instruction_o(instruction));
    
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
        .memToReg(memToReg));

    register_file regFile(
        .reg_write(writeData), 
        .clk(clk), 
        .rst(rst), 
        .write(writeEnable), 
        .rd(rd), 
        .rs1(rs1), 
        .rs2(rs2), 
        .reg1(reg1), 
        .reg2(reg2));
 
    ALU math(
        .ALU_source(src), 
        .opcode(opcode), 
        .funct3(funct3), 
        .funct7(funct7), 
        .reg1(reg1), 
        .reg2(reg2), 
        .immediate(imm_32), 
        .read_address(read_address), 
        .write_address(write_address), 
        .result(result), 
        .branch(branch));

    //sort through mem management inputs/outputs
    data_memory data_mem(
        .data_read_adr_i(read_address),
        .data_write_adr_i(write_address),
        .data_cpu_i(result),
        .data_bus_i(data_out_CPU), //need to arbitrate between when the instr_mem needs data
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
        .address_in(data_adr_o | instruction_adr_o), //only works if non-active addresses are set to 0 
        .data_in_CPU(data_cpu_o),
        .data_in_BUS(data_in_BUS), //external info
        .data_en(data_read | data_write), //may need to split to differentiate read vs write on data side -> nvm, already done
        .instr_en(instr_fetch),
        .bus_full(bus_full), //external info
        .memWrite(data_write),
        .memRead(data_read | instr_fetch),
        .clk(clk),
        .rst(rst),
        // outputs
        .state(state),
        .address_out(address_out), //to external output
        .data_out_CPU(data_out_CPU), //to data mem
        .data_out_BUS(data_out_BUS), //to external output
        .data_out_INSTR(data_out_INSTR));

    pc program_count(
        .clk(clk),
        .clr(rst),
        .load(load),
        .inc(inc),
        .ALU_out(tb_ALU_out),
        .Disable(tb_Disable),
        .data(tb_data),
        .imm_val(imm_32),
        .pc_val(pc_val));

endmodule