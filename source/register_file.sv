`default_nettype none

module register_file (
    input logic [31:0] in, 
    input logic [4:0] inAddress, outAddress1, outAddress2, //not sure about literally any bit amounts
    input logic clk, rst, write, 
    output reg [31:0] read1, read2
);
    reg[31:0] register [31:0]; //correct syntax for array

    always @ ( //or should I use initial
        outAddress1, 
        outAddress2, 
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
        read1 <= register[outAddress1];
        read2 <= register[outAddress2];
    end

    always @ (posedge clk, negedge rst) begin //reset pos or neg or no reset
        if (rst) begin
            for (integer i = 0; i <= 31; i++) begin
                register[i] <= 0;
            end
        end
        else begin
            if (write) begin
                register[inAddress] <= in;
            end
        end
    end
endmodule