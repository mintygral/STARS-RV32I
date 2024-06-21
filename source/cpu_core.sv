module cpu_core(

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

    //Memcontrol
    input logic [31:0] address_in, data_in_CPU, data_in_BUS,
    input logic data_en, instr_en, bus_full, memWrite, memRead,
    input logic clk, rst,
    // outputs
    output state_t state,
    output logic [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR
    
    //
    input logic inc, Disable,

    //(ALU or external reset) -> Program Counter 
    logic [31:0] data, 

    //Program Counter -> Instruction Memory
    logic [31:0] pc_val;

    //Memory Manager -> Instruction Memory
    logic [31:0] instruction_i;
    logic data_good;

    //Instruction Memory -> Memory Manager
    logic instr_fetch;
    logic [31:0] instruction_adr_o; 
    
    


endmodule