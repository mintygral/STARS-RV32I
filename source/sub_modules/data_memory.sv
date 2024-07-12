module data_memory(
    input logic [31:0] data_read_adr_i, data_write_adr_i, data_bus_i, data_cpu_i,
    input logic clk, data_good, rst,
    output logic data_read, data_write,
    output logic [31:0] data_adr_o, data_bus_o, data_cpu_o
);

    logic next_read, next_write;
    logic [31:0] stored_read_data, stored_write_data, stored_data_adr;

    always_comb begin
        next_read = 1'b0;
        next_write = 1'b0;
        stored_read_data = 32'b0;
        stored_write_data = 32'b0;
        stored_data_adr = 32'b0;
        if(data_read_adr_i != 32'b0) begin
            if(data_good) begin
                stored_read_data = data_bus_i;
                next_read = 1'b0;
            end else begin
                stored_read_data = 32'b0;
                next_read = 1'b1;
                stored_data_adr = data_read_adr_i;
            end
        end else if(data_write_adr_i != 32'b0) begin
            if(data_good) begin
                stored_write_data = 32'b0;
                next_write = 1'b0;
            end else begin
                stored_write_data = data_cpu_i;
                next_write = 1'b1;
                stored_data_adr = data_write_adr_i;
            end
        end
    end

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            data_adr_o <= 32'b0;
            data_bus_o <= 32'b0;
            data_cpu_o <= 32'b0;
            data_read <= 1'b0;
            data_write <= 1'b0;
        end else begin
            data_read <= next_read;
            data_write <= next_write;
            data_adr_o <= stored_data_adr;
            data_cpu_o <= stored_read_data;
            data_bus_o <= stored_write_data;
        end
    end

endmodule