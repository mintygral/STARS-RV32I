module keytranslate (
    input logic [3:0] key_in,

);

endmodule

module bcd2bin
   (
    // input logic [3:0] bcd7, // 10,000,000
    // input logic [3:0] bcd6, // 1,000,000
    // input logic [3:0] bcd5, // 100,000
    // input logic [3:0] bcd4, // 10,000
    input logic [3:0] bcd3, // 1000
    input logic [3:0] bcd2, // 100
    input logic [3:0] bcd1, // 10
    input logic [3:0] bcd0, // 1
    // output logic [31:0] bin
    output logic [15:0] bin
   );

//    assign bin = (bcd7 * 24'd10000000) + (bcd6 * 20'd1000000) + (bcd5 * 17'd100000) + (bcd4 * 14'd10000) + (bcd3 * 10'd1000) + (bcd2*7'd100) + (bcd1*4'd10) + (bcd0 * 1'd1);
        assign bin = (bcd3 * 10'd1000) + (bcd2*7'd100) + (bcd1*4'd10) + (bcd0 * 1'd1);

endmodule

module shift_reg
    ( input logic clk, rst,
      input logic [3:0] in,
      output reg [15:0] q);
 
    always_ff @ (posedge clk, posedge rst)
    begin
        if(rst)
            q <= 0;
        else 
            q <= {q[15:4], in};
    end 
endmodule