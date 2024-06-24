module datamemory_ALU_integration (
    
);
    
    ALU alu_grab(
        //inpur
        .ALU_source(ALU_source),
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .reg1(reg1),
        .reg2(reg2),
        .immediate(immediate),
        //output
        .read_address(read_address),
        .write_address(write_address),
        .result(result),
        .branch(branch)
        );    

    data_memory init(
        //input
        .data_read_adr_i(read_adress),
        .data_write_adr_i(write_address),
        .data_bus_i(),
        .data_cpu_i(result),
        .clk(),
        .data_good(),
        .rst(),
        //output
        .data_read(),
        .data_write(),
        .data_adr_o(),
        .data_bus_o(),
        .data_cpu_o()
    );

endmodule