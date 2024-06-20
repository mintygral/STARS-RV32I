`default_nettype none

module register_file (
    input logic [31:0] reg_write, 
    input logic [4:0] rd, rs1, rs2, //not sure about literally any bit amounts
    input logic clk, rst, write, 
    output reg [31:0] reg1, reg2
);
    reg[31:0] register [31:0]; //correct syntax for array

    always @ ( //or should I use initial
        rs1, 
        rs2, 
        register[0], 
        register[1], 
        register[2], 
        register[3],
        register[4],
        register[5],
        register[6], 
        register[7], 
        register[8], 
        register[9],
        register[10],
        register[11],
        register[12],
        register[13],
        register[14],
        register[15],
        register[16],
        register[17],
        register[18],
        register[19],
        register[20],
        register[21],
        register[22],
        register[23],
        register[24],
        register[25],
        register[26],
        register[27],
        register[28],
        register[29],
        register[30],
        register[31]
    ) begin
        reg1 <= register[rs1];
        reg2 <= register[rs2];
    end

    always @ (posedge clk, posedge rst) begin //reset pos or neg or no reset
        if (rst) begin
            for (integer i = 0; i <= 31; i++) begin
                register[i] <= 0;
            end
        end
        else begin
            if (write) begin
                register[rd] <= reg_write;
            end
        end
    end
endmodule