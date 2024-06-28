module synckey(
    input logic [19:0] in,
    input logic clock, reset,
    output logic [4:0] out,
    output logic strobe
);

    //logic [5:0] i;
    logic [1:0] flipflops;
    logic button_pressed;
    
    always_ff @(posedge clock, posedge reset) begin
        if(reset) begin
            flipflops <= 2'b00;
        end else begin  
            flipflops[1] <= flipflops[0];
            flipflops[0] <= button_pressed;
        end
    end
    
    always_comb begin
        button_pressed = |in;
        out = 0;
        for (integer i = 0; i < 20; i++) begin
            if(in[i])
                out = i[4:0];
        end
        strobe = flipflops[1];
    end

    


endmodule
