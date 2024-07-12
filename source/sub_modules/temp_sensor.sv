typedef enum logic [2:0] {
    IDLE = 0,
    SKIP_ROM = 1,
    CONVERT_TEMP = 2,
    SKIP_ROM2 = 3,
    READ_SCRATCH = 4,
    READ = 5,
    RESET = 6
} state_t;

module temp_sensor (
    // inputs
    input logic clk, rst,
    input logic read_request,
    // outputs
    output logic out_wire,
    output state_t state,
    output logic [3:0] count_read, // read state
    output logic [3:0] next_read,
    output logic [2:0] count_index, // index
    output logic [2:0] next_index, // every clk cycle
    output logic [6:0] clk_divider, // writing state, when count_reg == 7
    output logic strobe
    );

    // logic [3:0] count_read; // read state
    // logic [2:0] count_index; // index
    // logic [2:0] count_reg; // every clk cycle
    // logic [6:0] clk_divider; // writing state, when count_reg == 7
    state_t next_state, prev_state;

    logic [7:0] skip_rom = 8'hCC;
    logic [7:0] convert_t = 8'h44;
    logic [7:0] read_scratch = 8'hBE;

    always_ff @(posedge clk, posedge rst) begin : startFSM
        if (rst) begin
            clk_divider <= 0;
            strobe <= 0;
        end else begin
            clk_divider <= clk_divider + 1;
            strobe <= 0;
            if (clk_divider == 100) begin
                clk_divider <= 0;
                strobe <= 1;
            end
        end
    end

    always_ff @ (posedge strobe, posedge rst) begin : clk_divide
        if (rst) begin
            state <= IDLE;
        end else begin
            state <= next_state;
            count_index <= next_index;
            count_read <= next_read;
        end
    end

    always_comb begin : changeState
        next_index = 0;
        next_state = state;
        prev_state = state;
        next_read = 0;
        // count_index = 0;
        // count_reg = 0;
        out_wire = 0;
        case(state)
            IDLE: begin
                // count_read = 0;
                if (read_request) begin
                    next_state = SKIP_ROM;
                    next_index = 0;
                end
                else begin
                    next_state = IDLE;
                    next_read = 0;
                end
            end
            SKIP_ROM: begin
                if (count_index == 7) begin
                    // clk_divider++;
                    next_state = CONVERT_TEMP;
                    next_index = 0;
                end
                else begin
                    out_wire = skip_rom[count_index]; 
                    next_index = count_index + 1;
                    next_state = SKIP_ROM;
                end
            end
            CONVERT_TEMP: begin
                if (count_index == 7) begin
                    next_state = SKIP_ROM2;
                end
                else begin
                    out_wire = convert_t[count_index]; 
                    next_index = count_index + 1;
                    next_state = CONVERT_TEMP;
                end
            end
            SKIP_ROM2: begin 
                if (count_index == 7) begin
                    // clk_divider++;
                    next_state = READ_SCRATCH;
                end
                else begin 
                    next_index = count_index + 1;
                    next_state = SKIP_ROM2;
                end
            end
            READ_SCRATCH: begin
                if (count_index == 7) begin
                    next_state = READ;
                end
                else begin 
                    out_wire = read_scratch[count_index]; 
                    next_index = count_index + 1;
                    next_state = READ_SCRATCH;
                end
            end
            READ: begin
                if (count_read == 15) begin
                    next_state = RESET;
                end
                else begin 
                    next_read = count_read + 1;
                    next_state = READ;
                end
            end
            RESET: begin
                if (count_read == 15) begin
                    next_state = IDLE;
                end
                else begin 
                    next_read = count_read + 1;
                    next_state = RESET;
                end
            end
            default: begin
                next_state = IDLE;
                next_read = 0;
                // count_index = 0;
            end

        endcase
    end

endmodule 