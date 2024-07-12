module display(
    input logic [31:0] seq,
    output logic [63:0] ssds
);

    //logic [63:0] ssdec_data = ssds;
    logic [7:0] enable = 8'hff;

    ssdec sdd7(.in(seq[31:28]), .enable(enable[7]), .out({ssds[62:56]}));
    ssdec sdd6(.in(seq[27:24]), .enable(enable[6]), .out({ssds[54:48]}));
    ssdec sdd5(.in(seq[23:20]), .enable(enable[5]), .out({ssds[46:40]}));
    ssdec sdd4(.in(seq[19:16]), .enable(enable[4]), .out({ssds[38:32]}));
    ssdec sdd3(.in(seq[15:12]), .enable(enable[3]), .out({ssds[30:24]}));
    ssdec sdd2(.in(seq[11:8]), .enable(enable[2]), .out({ssds[22:16]}));
    ssdec sdd1(.in(seq[7:4]), .enable(enable[1]), .out({ssds[14:8]}));
    ssdec sdd0(.in(seq[3:0]), .enable(enable[0]), .out({ssds[6:0]}));

endmodule
