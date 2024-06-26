module edge_detector(
    input logic button_sync, clk, nrst,
    output logic posedge_button
);

    logic [1:0] flipflops;

    always_ff @(posedge clk, negedge nrst) begin
        if(!nrst) begin
            posedge_button <= 1'b0;
            flipflops <= 2'b0;
        end else begin
            flipflops[1] <= flipflops[0];
            flipflops[0] <= button_sync;
            if(flipflops[0] & ~flipflops[1]) posedge_button <= 1'b1;
            else posedge_button <= 1'b0;
        end
    end

endmodule