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
    output state_t state
    );

    logic [3:0] count_read; // read state
    logic [2:0] count_index; // index
    logic [2:0] count_reg; // every clk cycle
    logic [6:0] clk_divider; // writing state, when count_reg == 7
    state_t next_state, prev_state;

    logic [7:0] skip_rom = 8'hCC;
    logic [7:0] convert_t = 8'h44;
    logic [7:0] read_scratch = 8'hBE;

    logic strobe = (clk_divider == 100);

    always_ff @(posedge strobe, posedge rst) begin : startFSM
        if (rst) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin : changeState
        next_state = state;
        prev_state = state;
        count_read = 0;
        count_index = 0;
        count_reg = 0;
        clk_divider = 0;
        out_wire = 0;
        case(state)
            IDLE: begin
                count_read = 0;
                if (read_request) begin
                    next_state = SKIP_ROM;
                end
                else begin
                    next_state = IDLE;
                    count_read = 0;
                end
            end
            SKIP_ROM: begin
                count_reg = 0;
                count_index = 0;
                if (count_reg == 7) begin
                    clk_divider++;
                    next_state = CONVERT_TEMP;
                end
                else begin
                    out_wire = skip_rom[count_index]; 
                    count_index++;
                    count_reg++;
                    next_state = SKIP_ROM;
                end
            end
            CONVERT_TEMP: begin
                count_reg = 0;
                count_index = 0;
                if (count_reg == 7) begin
                    next_state = SKIP_ROM2;
                end
                else begin
                    out_wire = convert_t[count_index]; 
                    count_reg++;
                    count_index++;
                    next_state = CONVERT_TEMP;
                end
            end
            SKIP_ROM2: begin 
                count_reg = 0;
                if (count_reg == 7) begin
                    clk_divider++;
                    next_state = READ_SCRATCH;
                end
                else begin 
                    count_reg++;
                    next_state = SKIP_ROM2;
                end
            end
            READ_SCRATCH: begin
                count_reg = 0;
                count_index = 0;
                if (count_reg == 7) begin
                    next_state = READ;
                end
                else begin 
                    out_wire = read_scratch[count_index]; 
                    count_reg++;
                    count_index++;
                    next_state = READ_SCRATCH;
                end
            end
            READ: begin
                count_read = 0;
                if (count_read == 15) begin
                    next_state = RESET;
                end
                else begin 
                    count_read++;
                    next_state = READ_SCRATCH;
                end
            end
            RESET: begin
                count_read = 0;
                if (count_read == 15) begin
                    next_state = IDLE;
                end
                else begin 
                    count_read++;
                    next_state = RESET;
                end
            end
            default: begin
                next_state = IDLE;
                count_read = 0;
                count_index = 0;
                count_reg = 0; 
            end

        endcase
    end

endmodule 