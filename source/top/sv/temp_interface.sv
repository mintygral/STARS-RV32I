
`default_nettype none
module top (
  // I/O ports
  input  logic clk, rst,
  input logic [127:0] row_1, row_2,
  output logic lcd_en, lcd_rw,
  output reg [7:0] lcd_rs,
  output reg [7:0] lcd_data
);

  logic [4:0] button_val;
  logic [31:0] data_in_BUS, pc_data; //input data from memory bus
  logic bus_full; //input from memory bus
  logic [31:0] data_out_BUS, address_out, reg_write; //output data +address to memory bus
  logic [31:0] memory_address_out;

  synckey sync(
    .in(pb[19:0]),
    .clock(clk),
    .reset(rst),
    .out(button_val),
    .strobe(bus_full)
  );

  cpu_core core0(
    .data_in_BUS(data_in_BUS),
    .pc_data(pc_data),
    .bus_full(!bus_full),
    .clk(clk),
    .rst(rst),
    .data_out_BUS(data_out_BUS),
    .address_out(address_out),
    .reg_write(reg_write)
  );

  ram mem(
    .clk(clk),
    .address_data(address_out[11:0]),
    .address_instr(address_out[11:0]),
    .data_in(data_out_BUS),
    .write_enable(1'b1),
    .addr_out(memory_address_out),
    .instr_out(data_in_BUS)
  );

    // display displaying(.seq(data_in_BUS), .ssds({ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0}));

    // logic lcd_en, lcd_rw;
    // reg lcd_rs;
    // reg [7:0] lcd_data;
    lcd_controller lcd_display(.clk(clk), 
                               .rst(rst),
                               .row_1({data_in_BUS, 96'b0}),
                                .row_2({data_in_BUS, 96'b0}),
                                .lcd_en(lcd_en),
                                .lcd_rw(lcd_rw),
                                .lcd_rs(lcd_rs),
                                .lcd_data(lcd_data));

endmodule


// Add more modules down here...
module synckey(
    input logic [19:0] in,
    input logic clock, reset,
    output logic [4:0] out,
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
        button_pressed = |in;
        out = 0;
        for (integer i = 0; i < 20; i++) begin
            if(in[i])
                out = i[4:0];
        end
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
    output logic [31:0] addr_out,
    output logic [31:0] instr_out
);

reg[31:0] memory [4095:0];

initial begin
    $readmemh("cpu.mem", memory);
end

always @(posedge clk) begin
    if(write_enable) begin
        memory[address_data] <= data_in;
    end
    addr_out <= memory[address_data];
    instr_out <= memory[address_instr];
    
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
    Read_Request = 2,
    Write_Request = 3,
    Read = 4,
    Write = 5,
    Wait = 6
} state_t;

module cpu_core(
    input logic [31:0] data_in_BUS, pc_data,//input data from memory bus, memory starting point
    input logic bus_full, //input from memory bus
    input logic clk, rst, //external clock, reset
    output logic [31:0] data_out_BUS, address_out, reg_write //output data +address to memory bus
);

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
    logic [31:0] reg1, reg2;

    //ALU -> Data Memory
    logic [31:0] read_address, write_address, result;

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

    logic [31:0] mem_adr_i;
    logic mem_read;
    
    always_comb begin
        mem_adr_i = (data_adr_o | instruction_adr_o);
        data_en = data_read | data_write;
        mem_read = data_read | instr_fetch;
        instr_wait = ((~(read_address == 32'b0) | ~(write_address == 32'b0)) & ~data_good);
    end

    logic [31:0] load_data_flipflop;

    always_ff @(posedge clk) begin
        memToReg_flipflop <= memToReg;
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

    //multiplexer for register input
    always_comb begin
        if((opcode != 7'b0100011) && (opcode != 7'b1100011)) begin
            if(memToReg_flipflop == 1'b1) reg_write = (load_data_flipflop | data_cpu_o);
            else reg_write = result;
            reg_write_en = 1'b1;
        end else begin
            reg_write = 32'b0;
            reg_write_en = 1'b0;
        end
    end

    register_file regFile(
        .reg_write(reg_write), 
        .clk(clk), 
        .rst(rst), 
        .write(reg_write_en), 
        .rd(rd),
        .rs1(rs1), 
        .rs2(rs2),
        .reg1(reg1),
        .reg2(reg2));
 
    ALU math(
        .ALU_source(ALU_source), 
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

    always_comb begin
        data_good = !bus_full_CPU & (state == Read | state == Write);
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

    pc program_count(
        .clk(clk),
        .clr(rst),
        .load(load_pc),
        .inc(data_good),
        .ALU_out(branch),
        .Disable(instr_wait),
        .data(pc_data),
        .imm_val(imm_32),
        .pc_val(pc_val));

endmodule

module ALU(
    input logic ALU_source,
    input logic [6:0] opcode,
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic [31:0] reg1, reg2, immediate,
    output logic [31:0] read_address, write_address, result,
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
            7'b1101111,7'b1100111: branch=1'b1;//jump and link, jalr
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
                    imm_32 = {20'b0, instruction[31:20]};
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
                    imm_32 = {20'b0, instruction[31:25], instruction[11:7]};
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
                    imm_32 = {20'b0, instruction[31], instruction[7], instruction[30:25], instruction[11:8]};
                    funct7 = 7'b0;
                    rd = 5'b0;
                    ALU_source = 1'b1;
                    memToReg = 1'b0;
                    load = 1'b0;
                end
            7'b1101111: //j type instruction
                begin
                    rd = instruction[11:7];
                    imm_32 = {12'b0, instruction[31], instruction[19:12], instruction[20], instruction[30:21]};
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
                    imm_32 = {12'b0, instruction[31:12]};
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
        stored_data_adr = data_read_adr_i | data_write_adr_i;
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
            data_adr_o <= 32'b0;
            //data_bus_o <= 32'b0;
            //data_cpu_o <= 32'b0;
            data_read <= 1'b0;
            data_write <= 1'b0;
        end else begin
            data_read <= next_read;
            data_write <= next_write;
            data_adr_o <= stored_data_adr;
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
            stored_instr_adr = 32'b0;
            stored_instr = instruction_i;
        end else if(!instr_wait) begin
            next_fetch = 1'b1;
            stored_instr_adr = instruction_adr_i;
            stored_instr = 32'b0;
        end else begin
            next_fetch = 1'b0;
            stored_instr_adr = 32'b0;
            stored_instr = instruction_i;
        end
    end

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            instruction_adr_o <= 32'b0;
            instruction_o <= 32'b0;
            instr_fetch <= 1'b0;
        end else if(instr_wait) begin
            instruction_adr_o <= 32'b0;
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
        address_out = 32'h0;
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
                    next_state = Read_Request;
                    prev_state = Read_Request;
                end else if (memWrite) begin
                    next_state = Write_Request;
                    prev_state = Write_Request;
                end else begin
                    next_state = IDLE;
                end
            end

            Read_Request: begin 
                if (bus_full) begin
                    next_state = Wait;
                end else begin
                    next_state = Read;
                end
            end
            
            Write_Request: begin 
                if (bus_full) begin
                    next_state = Wait;
                end else begin
                    next_state = Write;
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
                    if (prev_state == Read_Request) begin
                        next_state = Read;
                    end else if (prev_state == Write_Request) begin
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
    always_ff @(posedge clk, posedge clr) begin

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
       jump_ad = next_line_ad + imm_val;    // Calculate jump address (jump and link)

	
        // Mux choice between next line address and jump address
        if (Disable) begin 
		    next_pc = pc_val; 
	    end

        else if (load) begin
            next_pc = data + next_line_ad;
        end
            
        else if (ALU_out) begin
		    next_pc = jump_ad ;
	    end
	
        else if (inc) begin
            next_pc= next_line_ad;
       end

        
       
   end       
endmodule

module register_file (
    input logic [31:0] reg_write, 
    input logic [4:0] rd, rs1, rs2, 
    input logic clk, rst, write,
    output logic [31:0] reg1, reg2 //array????
);
    reg[31:0][31:0] register;
    //reg[31:0][31:0] next_register; 

    logic [31:0] write_data;

    //assign register = '{default:'0};

    always_comb begin
        write_data = 32'b0;
        if (write) begin
            if (rd != 0) begin
                write_data = reg_write;
            end else begin
                write_data = 32'b0;
            end
        end
        reg1 = register[rs1];
        reg2 = register[rs2];
    end

    always_ff @ (negedge clk, posedge rst) begin //reset pos or neg or no reset
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
    localparam IDLE = 8'h00,                
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
            currentState <= IDLE;
        else if (lcd_ctrl)
            currentState <= nextState;
        else
            currentState <= currentState;
    end

    always  @(*) begin
        case (currentState)
            IDLE: nextState = SET_FUNCTION;
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
            default: nextState = IDLE;
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
                IDLE: lcd_data <= 8'hxx;
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