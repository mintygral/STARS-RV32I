module tb_control_unit;

    logic [31:0] instruction;

    control_unit cont(.instruction(instruction));

    

endmodule