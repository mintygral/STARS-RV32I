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
    output logic read_command, read_signal, read_clk, out_wire,
    output state_t state
);
    logic [3:0] fcount; // read state
    logic [2:0] tcount;
    logic [2:0] rcount; // every clk cycle
    logic [6:0] clk_divider; // writing state, when rcount == 7
    state_t next_state, prev_state;

    logic [7:0] skip_rom = 8'hCC;
    logic [7:0] convert_t = 8'h44;
    logic [7:0] read_scratch = 8'hBE;

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
        fcount = 0;
        tcount = 0;
        rcount = 0;
        out_wire = 0;
        case(state)
            IDLE: begin
                fcount = 0;
                if (read_request) begin
                    next_state = SKIP_ROM;
                end
                else begin
                    next_state = IDLE;
                    fcount = 0;
                end
            end
            SKIP_ROM: begin
                rcount = 0;
                if (rcount == 7) begin
                    next_state = CONVERT_TEMP;
                end
                else begin
                    out_wire = skip_rom[tcount]; 
                    tcount++;
                    rcount++;
                    next_state = SKIP_ROM;
                end
            end
            CONVERT_TEMP: begin
                rcount = 0; 
                if (rcount == 7) begin
                    next_state = SKIP_ROM2;
                end
                else begin 
                    rcount++;
                    next_state = CONVERT_TEMP;
                end
            end
            SKIP_ROM2: begin 
                rcount = 0;
                if (rcount == 7) begin
                    next_state = READ_SCRATCH;
                end
                else begin 
                    rcount++;
                    next_state = SKIP_ROM2;
                end
            end
            READ_SCRATCH: begin
                rcount = 0;
                if (rcount == 7) begin
                    next_state = READ;
                end
                else begin 
                    rcount++;
                    next_state = READ_SCRATCH;
                end
            end
            READ: begin
                fcount = 0;
                if (fcount == 15) begin
                    next_state = RESET;
                end
                else begin 
                    fcount++;
                    next_state = READ_SCRATCH;
                end
            end
            RESET: begin
                fcount = 0;
                if (fcount == 15) begin
                    next_state = IDLE;
                end
                else begin 
                    fcount++;
                    next_state = RESET;
                end
            end
            default: begin
                next_state = IDLE;
                fcount = 0;
                tcount = 0;
                rcount = 0; 
            end

        endcase
    end

endmodule 