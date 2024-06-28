typedef enum logic [2:0] {
    INIT = 0,
    IDLE = 1,
    Read = 4,
    Write = 5,
    Wait = 6
} state_t;

module cpu_core(
        input logic [31:0] data_in_BUS, pc_data,//input data from memory bus, memory starting point
        input logic bus_full, //input from memory bus
        input logic clk, rst, //external clock, reset
        output logic [31:0] data_out_BUS, address_out, result, imm_32, reg1, reg2, data_cpu_o, write_address, reg_write, //instruction, result, reg1, reg2 //output data +address to memory bus
        //testing vals from control unit
        output logic [4:0] rs1, rs2, rd,
        output logic memToReg_flipflop, instr_wait, reg_write_en, data_write,
        output logic [6:0] opcode,
        output logic [31:0] pc_val, pc_jump,
        output logic branch_ff, branch, load_pc
);
    //Instruction Memory -> Control Unit
    logic [31:0] instruction;

    //Control Unit -> ALU
    logic [6:0] funct7;
    logic [2:0] funct3;
    logic ALU_source; //0 means register, 1 means immediate
    
    //Control Unit -> ALU + Program Counter
    // logic [31:0] imm_32;
    // logic [31:0] pc_jump;

    //Control Unit -> Registers
    // logic [4:0] rs1, rs2, rd;
    
    //Control Unit -> Data Memory
    logic memToReg; //0 means use ALU output, 1 means use data from memory

    //Control Unit -> Program Counter
    // logic load_pc; //0 means leave pc as is, 1 means need to load in data

    //Data Memory -> Registers
    //logic [31:0] reg_write;

    //Register Input (double check where its coming from)
    // logic reg_write_en;

    //Registers -> ALU
    // logic [31:0] reg1, reg2, resultx;

    //ALU -> Data Memory
    logic [31:0] read_address;
    //  write_address;//, result;

    //ALU -> Program Counter
    // logic branch;

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
    logic data_read;
    // data_write;
    logic [31:0] data_adr_o, data_bus_o;
    // data_cpu_o;

    //(ALU or external reset) -> Program Counter 
    //logic [31:0] pc_data; //external reset value only now

    //Program Counter -> Instruction Memory
    // logic [31:0] pc_val;

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

        // assign result = imm_32;

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
        .read_address(read_address), 
        .write_address(write_address), 
        .result(result), 
        .branch(branch),
        .pc_data(pc_jump),
        .pc_val(pc_val));

    always_comb begin
        data_good = !bus_full_CPU & (state == Read | state == Write);
    end

    logic [31:0] val2;
    always_comb begin
        // if (ALU_source) val2 = imm_32;
        // else val2 = reg2;
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
    logic [31:0] pc_input;
    assign pc_input = (pc_jump != 32'b0) ? pc_jump : pc_data;
    pc program_count(
        .clk(clk),
        .clr(rst),
        .load(load_pc),
        .inc(data_good),
        .ALU_out(branch_ff),
        .Disable(instr_wait),
        .data(pc_input),
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
                    rd = instruction[11:7] ;
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
       jump_ad = pc_val + imm_val;    // Calculate jump address (jump and link)

	
        // Mux choice between next line address and jump address
        if (Disable) begin 
		      next_pc = pc_val; 
	      end

        else if (load) begin
          next_pc = data;
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