module request_unit (
    // From Bus 
    input logic busy_o,
    input logic [31:0] cpu_dat_o,
    // From CPU
    input logic MemRead,            // = MemRead    from data memory
    input logic MemWrite,           // = MemWrite   from data memory
    input logic [31:0] address_DM,  // = address    from data memory
    input logic [31:0] address_IM,  // = address    from instruction memory
    input logic [31:0] data_DM,     // = write data from data memory
    // To Bus
    output logic write_i,
    output logic read_i,
    output logic [3:0] sel_i,
    output logic [31:0] adr_i,
    output logic [31:0] cpu_dat_i,
    // To CPU
    output logic enable,        // = enable         to Program Counter
    output logic [31:0] instr,  // = Instruction    to Instruction Memory
    output logic [31:0] data    // = Read Data      to Data Memory
);
    typedef enum logic {IDLE, BUSY} StateType;

    StateType state, next_state;
    logic next_write_i, next_read_i, rtype, next_rtype, enable, next_enable, dhit, next_dhit;
    logic [31:0] next_adr_i, next_cpu_dat_i, next_instr, next_data;

    always_ff @(posedge clk, negedge nRST) begin
        if (!nRST) begin
            instr       <= 32'b0;
            data        <= 32'b0;
            write_i     <= 1'b0;
            read_i      <= 1'b0;
            adr_i       <= 32'b0;
            cpu_dat_i   <= 32'b0; 
            state       <= IDLE;
            rtype       <= 1'b0;
            enable      <= 1'b0;
            dhit        <= 1'b0;
        end else begin
            instr       <= next_instr;
            data        <= next_data;
            write_i     <= next_write_i;
            read_i      <= next_read_i;
            adr_i       <= next_adr_i;
            cpu_dat_i   <= next_cpu_dat_i;
            state       <= next_state;
            rtype       <= next_rtype;
            enable      <= next_enable;
            dhit        <= next_dhit;
        end
    end

    assign sel_i = 4'b1111;

    always_comb begin
        next_dhit       = dhit;
        next_enable     = 1'b0;
        next_instr      = instr;
        next_data       = data;
        next_read_i     = read_i;
        next_write_i    = write_i;
        next_adr_i      = adr_i;
        next_cpu_dat_i  = cpu_dat_i;
        next_state      = state;
        next_rtype      = rtype;
        case (state)
            IDLE: begin
                next_state      = BUSY;
                if (MemRead && !dhit) begin
                    next_read_i     = 1'b1; 
                    next_write_i    = 1'b0;
                    next_adr_i      = address_DM;
                    next_rtype      = 1'b1;              
                end else if (MemWrite && !dhit) begin
                    next_read_i     = 1'b0;
                    next_write_i    = 1'b1;
                    next_adr_i      = address_DM;
                    next_cpu_dat_i  = data_DM;
                    next_rtype      = 1'b1;          
                end else begin
                    next_read_i     = 1'b1;
                    next_write_i    = 1'b0;
                    next_adr_i      = address_IM;
                    next_rtype      = 1'b0;
                    next_dhit       = 1'b0;          
                end
            end
            BUSY: begin
                if (!busy_o) begin
                    next_enable     = rtype == 1'b0;
                    next_dhit       = rtype == 1'b1;
                    next_read_i     = 1'b0;
                    next_write_i    = 1'b0;
                    next_adr_i      = 32'b0;
                    next_cpu_dat_i  = 32'b0;
                    next_rtype      = 0;
                    next_state      = IDLE;
                end else begin
                    if (rtype == 0) begin
                        next_instr  = cpu_dat_o;
                        next_data   = 32'b0;
                    end else begin
                        next_instr  = 32'b0;
                        next_data   = cpu_dat_o;
                    end
                end
            end 
            default: begin
                next_dhit       = dhit;
                next_enable     = 1'b0;
                next_instr      = instr;
                next_data       = data;
                next_read_i     = read_i;
                next_write_i    = write_i;
                next_adr_i      = adr_i;
                next_cpu_dat_i  = cpu_dat_i;
                next_state      = state;
            end
        endcase
    end
endmodule