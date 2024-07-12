module ssdec(
    input logic [3:0] in,
    input logic enable,
    output logic [6:0] out
);

    assign out[0] = enable & (~in[3] & in[1] | in[3] & ~in[0] | ~in[2] & ~in[0] | in[2] & in[1] | in[3] & ~in[2] & ~in[1] | ~in[3] & in[2] & in[0]);
    assign out[1] = enable & (~in[3] & ~in[2] | ~in[2] & ~in[1] | ~in[2] & ~in[0] | ~in[3] & in[1] & in[0] | ~in[3] & ~in[1] & ~in[0] | in[3] & ~in[1] & in[0]);
    assign out[2] = enable & (in[3] & ~in[2] | ~in[3] & in[2] | ~in[1] & in[0] | ~in[3] & ~in[1] | ~in[3] & in[0]);
    assign out[3] = enable & (~in[2] & in[1] & in[0] | ~in[3] & in[1] & ~in[0] | in[2] & ~in[1] & in[0] | ~in[2] & ~in[1] & ~in[0] | in[3] & in[2] & ~in[0]);
    assign out[4] = enable & (in[3] & in[2] | in[1] & ~in[0] | ~in[2] & ~in[0] | in[3] & in[1]);
    assign out[5] = enable & (in[3] & ~in[2] | ~in[1] & ~in[0] | in[3] & in[1] | in[2] & ~in[0] | ~in[3] & in[2] & ~in[1]);
    assign out[6] = enable & (in[3] & ~in[2] | in[1] & ~in[0] | ~in[2] & in[1] | ~in[3] & in[2] & ~in[1] | in[3] & in[2] & in[0]);

endmodule

    /**always_comb begin
        if(enable) begin
            case(in)
                4'b0000: out = 7'b1111110;
                4'b0001: out = 7'b0110000;
                4'b0010: out = 7'b1101101;
                4'b0011: out = 7'b1111001;
                4'b0100: out = 7'b0110011;
                4'b0101: out = 7'b1011011;
                4'b0110: out = 7'b1011111;
                4'b0111: out = 7'b1110000;
                4'b1000: out = 7'b1111111;
                4'b1001: out = 7'b1110011;
                4'b1010: out = 7'b1110111;
                4'b1011: out = 7'b0011111;
                4'b1100: out = 7'b1001110;
                4'b1101: out = 7'b0111101;
                4'b1110: out = 7'b1001111;
                4'b1111: out = 7'b1000111;
            endcase
        end else begin
            out = 7'b0000000;
        end
    end*/
