typedef enum logic [2:0] {
    INIT = 3'b000,
    IDLE = 3'b001,
    Read_Request = 3'b010,
    Write_Request = 3'b011,
    Read = 3'b100,
    Write = 3'b101,
    Wait = 3'b110
} state_t;

module mem_manager(
    // inputs
    input logic [31:0] address_in, data_in_CPU, data_in_BUS,
    input logic data_en, instr_en, bus_full, memWrite, memRead,
    input logic clk, rst,
    // outputs
    output state_t state,
    output logic [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR
    );

    state_t next_state, prev_state;

    always_ff @(posedge clk || posedge rst) begin : startFSM
        if (rst) begin
            state <= INIT;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin : changeState
        next_state = state;
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
                if (instr_en) begin
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
