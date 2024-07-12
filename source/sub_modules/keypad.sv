module keypad (
    input clk, nRST,
    input [3:0] rows,
    output [3:0] cols,
    output [7:0] code
);

    logic [1:0] count;
    logic [7:0] keyCode;

    always @(count) begin
        case (count)
            2'b00:  cols <= 4'b0001;
            2'b01:  cols <= 4'b0010;
            2'b10:  cols <= 4'b0100;
            2'b11:  cols <= 4'b1000;
            default: cols <= 4'b0001;
        endcase
    end

    always @(posedge clk, negedge nRST) begin
		if (!nRST) begin
			keyCode <= 8'b0000_0000;
			count <= 2'b00;
		end
		else begin
			if (rows != 4'b0000) begin
				keyCode <= {scan_col, read_row};
			end
			else begin
				count <= count + 2'b01;     
			end
		end
	end

    always_ff @(posedge clk, negedge nRST) begin
        if (!nRST) begin
            code <= 8'b0;
        end else begin
            case (keyCode)
                8'b0000_0000: code <= 8'b0;
				8'b0001_0001: code <= "1";
				8'b0001_0010: code <= "2";
				8'b0001_0100: code <= "3";
				8'b0001_1000: code <= "A";
				8'b0010_0001: code <= "4";
				8'b0010_0010: code <= "5";
				8'b0010_0100: code <= "6";
				8'b0010_1000: code <= "B";
				8'b0100_0001: code <= "7";
				8'b0100_0010: code <= "8";
				8'b0100_0100: code <= "9";
				8'b0100_1000: code <= "C";
				8'b1000_0001: code <= "*";
				8'b1000_0010: code <= "0";
				8'b1000_0100: code <= "#";
				8'b1000_1000: code <= "D";
				default:      code <= 8'b0;
            endcase
        end
    end
endmodule