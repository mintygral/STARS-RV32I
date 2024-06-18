typedef enum logic [2:0] {
    INIT = 0; IDLE = 1;
    Read_Request = 2; Write_Request = 3;
    Read = 4; Write = 5;
    Wait = 6;
} state_t;

module mem_manager(
    // inputs
    input [31:0] address_in, data_in_CPU, data_in_BUS;
    input data_en, instr_en, bus_full, memWrite, memRead;
    input clk, rst;
    // outputs
    output state_t state;
    output [31:0] address_out, data_out_CPU, data_out_BUS, data_out_INSTR;
);

    state_t next_state, prev_state;
    assign address_out = address_in;
    assign logic bus_empty = (!bus_full);

    always_ff @( posedge clk ) begin : startFSM
        if (rst) begin state <= INIT; end 
        else begin state <= next_state; end
    end

    always_comb begin : changeState
        next_state = state;
        case(state) 
            INIT: if (!rst) begin next_state = IDLE; end 
                  else begin next_state = INIT; end
            
            IDLE: if (memRead) begin next_state = Read_Request; prev_state = Read_Request; data_en = 1; end
                  else if (memWrite) begin next_state = Write_Request; prev_state = Write_Request; data_en = 1; end
                  else begin next_state = IDLE; end
        endcase
    end
 endmodule