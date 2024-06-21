typedef enum logic [2:0] {
    INIT = 0,
    IDLE = 1,
    Read_Request = 2,
    Write_Request = 3,
    Read = 4,
    Write = 5,
    Wait = 6
} state_t;

module memcontrol(
    // inputs
    // data_in_BUS and bus_full are the only inputs from the bus manager, so we need to figure those out on wednesday
    input logic [31:0] address_in, data_in_CPU, data_in_BUS,
    input logic data_en, instr_en, bus_full, memWrite, memRead,
    input logic clk, rst,
    // outputs
    output state_t state,
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
        // garbage values for testing
        address_out = 32'hABCD;
        data_out_BUS = 32'hABCD;
        data_out_CPU = 32'hABCD;
        data_out_INSTR = 32'hABCD;
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
