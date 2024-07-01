typedef enum logic [2:0] {
    IDLE = 0,
    WRITE_ROM = 1,
    WRITE_READ = 2,
    READ = 3,
    WRITE_STOP = 5
} state_t;

module temp_sensor (
    input clk, rst, 
    input read_request,
    // outputs
    output read_command, read_signal, read_clk, out_wire,
    output state_t state
);
    integer counter;
    state_t next_state, prev_state;

    always_ff @(posedge clk, posedge rst) begin : startFSM
        if (rst) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin : changeState
        next_state = state;
        prev_state = state;
        counter = 0;
        case(state)
            IDLE: begin
                if (read_request) begin
                    next_state = READ;
                    counter = 0;
                end
                else begin
                    next_state = IDLE;
                    counter = 0;
                end
            end
            WRITE_ROM: begin
                if (counter == 16) begin
                    next_state = WRITE_READ;
                end
                else begin 
                    counter++;
                    next_state = WRITE_ROM;
                end
            end
            WRITE_READ: begin 
                if (counter == 16) begin
                    next_state = READ;
                end
                else begin 
                    counter++;
                    next_state = WRITE_READ;
                end
            end
            READ: begin 
                if (counter == 16) begin
                    next_state = WRITE_STOP;
                end
                else begin 
                    counter++;
                    next_state = READ;
                end
            end
            WRITE_STOP: begin
                if (counter == 16) begin
                    next_state = IDLE;
                end
                else begin 
                    counter++;
                    next_state = WRITE_STOP;
                end
            end
            default: begin
                next_state = IDLE;
                counter = 0;
            end

        endcase
    end

endmodule 