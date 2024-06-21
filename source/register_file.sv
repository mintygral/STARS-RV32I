`default_nettype none

module register_file (
    input logic [31:0] reg_write, 
    input logic [4:0] rd, rs1, rs2, 
    input logic clk, rst, write,
    output logic [31:0] reg1, reg2 //array????
);
    reg[31:0][31:0] register; 
    reg[31:0][31:0] next_register; 


    //assign register = '{default:'0};

    always_comb begin
        next_register = register;
        if (~rd == 0) begin
            if (write) begin
                next_register[rd] = reg_write;
            end
        end
        reg1 = register[rs1];
        reg2 = register[rs2];
    end

    always_ff @ (posedge clk, negedge rst) begin //reset pos or neg or no reset
        if (~rst) begin
            register <= '0;
        end
        else begin
            register <= next_register;
           
        end
    end
endmodule