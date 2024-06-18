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
            INIT: if (!rst) next_state = IDLE;  
                  else next_state = INIT; 
            
            IDLE: if (memRead) next_state = Read_Request; prev_state = Read_Request;
                  else if (memWrite) next_state = Write_Request; prev_state = Write_Request;
                  else next_state = IDLE; 

            Read_Request: if (bus_full) next_state = Wait; prev_state = Read_Request;
                          else next_state = Read;
            
            Write_Request: if (bus_full) next_state = Wait; prev_state = Write_Request;
                          else next_state = Write;
            
            Read: address_out = address_in; data_out_BUS = 32'b0; 
                if (data_en) data_out_CPU = data_in_BUS; data_out_INSTR = 32'b0; // going to MUX
                if (instr_en) data_out_CPU = 32'b0; data_out_INSTR = data_in_BUS; // going to CU
                next_state = IDLE;
            
            Write: address_out = address_in; data_out_BUS = data_in_CPU; 
                   data_out_INSTR = 32'b0; data_out_CPU = 32'b0;
                   next_state = IDLE;

            Default: next_state = IDLE:
            
        endcase
    end
 endmodule