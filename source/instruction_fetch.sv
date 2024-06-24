`default_nettype none

typedef enum logic [2:0] {
    INIT = 0,
    IDLE = 1,
    Read_Request = 2,
    Write_Request = 3,
    Read = 4,
    Write = 5,
    Wait = 6
} state_t;

module instruction_fetch(
    input logic [31:0] data_in_CPU, data_in_BUS,
    input logic data_en, bus_full, memWrite,
    input logic clk, rst,

    input logic [31:0] instruction_adr_i,

    output state_t state,
    output logic [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR,
    output logic [31:0] instruction_o
);
    logic [31:0] address_in;
    logic bus_full_CPU, instr_fetch;
    logic good_data;

    always_comb begin
        good_data = bus_full_CPU & (state == Read);
    end

    memcontrol memory (
        .address_in(address_in), 
        .data_in_CPU(data_in_CPU),
        .data_in_BUS(data_in_BUS),
        .data_en(data_en),
        .instr_en(instr_fetch),
        .bus_full(bus_full),
        .memWrite(memWrite),
        .memRead(instr_fetch), //only valid for this integration module
        .clk(clk),
        .rst(rst),
        // outputs
        .state(state),
        .address_out(address_out),
        .data_out_CPU(data_out_CPU),
        .data_out_BUS(data_out_BUS),
        .data_out_INSTR(data_out_INSTR),
        .bus_full_CPU(bus_full_CPU)
        );

    instruction_memory instr_mem(
        .instruction_adr_i(instruction_adr_i),
        .instruction_i(data_out_INSTR),
        .clk(clk),
        .data_good(good_data),
        .rst(rst),
        .instr_fetch(instr_fetch),
        .instruction_adr_o(address_in),
        .instruction_o(instruction_o)
        );

endmodule

module instruction_memory(
    input logic [31:0] instruction_adr_i, instruction_i,
    input logic clk, data_good, rst,
    output logic instr_fetch,
    output logic [31:0] instruction_adr_o, instruction_o
);

    logic next_fetch;
    logic [31:0] stored_instr, stored_instr_adr;

    always_comb begin
        next_fetch = 1'b0;
        if(data_good) begin
            next_fetch = 1'b0;
            stored_instr_adr = 32'b0;
            stored_instr = instruction_i;
        end else begin
            next_fetch = 1'b1;
            stored_instr_adr = instruction_adr_i;
            stored_instr = 32'b0;
        end
    end

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            instruction_adr_o <= 32'b0;
            instruction_o <= 32'b0;
            instr_fetch <= 1'b0;
        end else begin
            instruction_adr_o <= stored_instr_adr;
            instruction_o <= stored_instr;
            instr_fetch <= next_fetch;
        end
        $display("instr fetch: %b", instr_fetch);
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
    output logic [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR,
    output logic bus_full_CPU
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
        bus_full_CPU = ~bus_full;
        // garbage values for testing
        address_out = 32'hABCDEF12;
        data_out_BUS = 32'hABCDEF12;
        data_out_CPU = 32'hABCDEF12;
        data_out_INSTR = 32'hABCDEF12;
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

