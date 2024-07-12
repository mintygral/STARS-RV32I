module bin_to_LCD(
    input logic [31:0] binary_in,
    output logic [31:0] LCD_out
);

    logic [15:0] BCD_interim;
    integer i;

    always_comb begin
        BCD_interim = 32'b0;
        if(binary_in[31:16] == 16'h0000) begin
            for(i = 0; i < 14; i = i + 1) begin
                if(BCD_interim[3:0] >= 5) BCD_interim[3:0] = BCD_interim[3:0] + 3;
                if(BCD_interim[7:4] >= 5) BCD_interim[7:4] = BCD_interim[7:4] + 3;
                if(BCD_interim[11:8] >= 5) BCD_interim[11:8] = BCD_interim[11:8] + 3;
                if(BCD_interim[15:12] >= 5) BCD_interim[15:12] = BCD_interim[15:12] + 3;
                BCD_interim = {BCD_interim[14:0], binary_in[15-i]};
            end

            case(BCD_interim[15:12])
                4'b0000: LCD_out[31:24] = 8'b00110000;
                4'b0001: LCD_out[31:24] = 8'b00110001;
                4'b0010: LCD_out[31:24] = 8'b00110010;
                4'b0011: LCD_out[31:24] = 8'b00110011;
                4'b0100: LCD_out[31:24] = 8'b00110100;
                4'b0101: LCD_out[31:24] = 8'b00110101;
                4'b0110: LCD_out[31:24] = 8'b00110110;
                4'b0111: LCD_out[31:24] = 8'b00110111;
                4'b1000: LCD_out[31:24] = 8'b00111000;
                4'b1001: LCD_out[31:24] = 8'b00111001;
                4'b1010: LCD_out[31:24] = 8'b00101011;
                4'b1011: LCD_out[31:24] = 8'b00101101;
                4'b1100: LCD_out[31:24] = 8'b00101010;
                4'b1101: LCD_out[31:24] = 8'b11111101;
                default: LCD_out[31:24] = 8'b01011111; //underscore - default/blank value
            endcase
            case(BCD_interim[11:8])
                4'b0000: LCD_out[23:16] = 8'b00110000;
                4'b0001: LCD_out[23:16] = 8'b00110001;
                4'b0010: LCD_out[23:16] = 8'b00110010;
                4'b0011: LCD_out[23:16] = 8'b00110011;
                4'b0100: LCD_out[23:16] = 8'b00110100;
                4'b0101: LCD_out[23:16] = 8'b00110101;
                4'b0110: LCD_out[23:16] = 8'b00110110;
                4'b0111: LCD_out[23:16] = 8'b00110111;
                4'b1000: LCD_out[23:16] = 8'b00111000;
                4'b1001: LCD_out[23:16] = 8'b00111001;
                4'b1010: LCD_out[23:16] = 8'b00101011;
                4'b1011: LCD_out[23:16] = 8'b00101101;
                4'b1100: LCD_out[23:16] = 8'b00101010;
                4'b1101: LCD_out[23:16] = 8'b11111101;
                default: LCD_out[23:16] = 8'b01011111; //underscore - default/blank value
            endcase
            case(BCD_interim[7:4])
                4'b0000: LCD_out[15:8] = 8'b00110000;
                4'b0001: LCD_out[15:8] = 8'b00110001;
                4'b0010: LCD_out[15:8] = 8'b00110010;
                4'b0011: LCD_out[15:8] = 8'b00110011;
                4'b0100: LCD_out[15:8] = 8'b00110100;
                4'b0101: LCD_out[15:8] = 8'b00110101;
                4'b0110: LCD_out[15:8] = 8'b00110110;
                4'b0111: LCD_out[15:8] = 8'b00110111;
                4'b1000: LCD_out[15:8] = 8'b00111000;
                4'b1001: LCD_out[15:8] = 8'b00111001;
                4'b1010: LCD_out[15:8] = 8'b00101011;
                4'b1011: LCD_out[15:8] = 8'b00101101;
                4'b1100: LCD_out[15:8] = 8'b00101010;
                4'b1101: LCD_out[15:8] = 8'b11111101;
                default: LCD_out[15:8] = 8'b01011111; //underscore - default/blank value
            endcase
            case(BCD_interim[3:0])
                4'b0000: LCD_out[7:0] = 8'b00110000;
                4'b0001: LCD_out[7:0] = 8'b00110001;
                4'b0010: LCD_out[7:0] = 8'b00110010;
                4'b0011: LCD_out[7:0] = 8'b00110011;
                4'b0100: LCD_out[7:0] = 8'b00110100;
                4'b0101: LCD_out[7:0] = 8'b00110101;
                4'b0110: LCD_out[7:0] = 8'b00110110;
                4'b0111: LCD_out[7:0] = 8'b00110111;
                4'b1000: LCD_out[7:0] = 8'b00111000;
                4'b1001: LCD_out[7:0] = 8'b00111001;
                4'b1010: LCD_out[7:0] = 8'b00101011;
                4'b1011: LCD_out[7:0] = 8'b00101101;
                4'b1100: LCD_out[7:0] = 8'b00101010;
                4'b1101: LCD_out[7:0] = 8'b11111101;
                default: LCD_out[7:0] = 8'b01011111; //underscore - default/blank value
            endcase
        end else begin
            LCD_out = binary_in;
        end
    end

endmodule