module keypad_interface(
    input logic pin1,pin2,pin3,pin4,pin5,pin6,pin7,pin8,
    output logic [3:0] out
);

logic [7:0] code;

assign code = {pin1,pin2,pin3,pin4,pin5,pin6,pin7,pin8};

always_comb begin 
    case(code)
        8'b10000001: out=4'b0011;
        8'b10001000: out=4'b0000;
        8'b10000100: out=4'b0001;
        8'b10000010: out=4'b0010;
        8'b01000001: out=4'b0111;
        8'b01001000: out=4'b0100;
        8'b01000100: out=4'b0101;
        8'b01000010: out=4'b0110;
        8'b00101000: out=4'b1000;
        8'b00100100: out=4'b1001;
        8'b00100010: out=4'b1010;
        8'b00100001: out=4'b1011;
        8'b00011000: out=4'b1100;
        8'b00010100: out=4'b1101;
        8'b00010010: out=4'b1110;
        8'b00010001: out=4'b1111;
        default: out=4'b0;
    endcase
end

        // out=4'b0000; == 1
        // out=4'b0001; == 4
        // out=4'b0010; == 7
        // out=4'b0011; == .
        // out=4'b0111; == 2
        // out=4'b0100; == 5
        // out=4'b0101; == 8
        // out=4'b0110; == 0
        // out=4'b1000; == 3
        // out=4'b1001; == 6
        // out=4'b1010; == 9
        // out=4'b1011; == #
        // out=4'b1100; == A
        // out=4'b1101; == B
        // out=4'b1110; == C
        // out=4'b1111; == D

endmodule