.data

# Allocate the numbers array
numbers: .skip 32

.text
find_max:
    la a5, numbers              # la rd, symbol == pseudosintruction for auipc rd, symbol [31:12]; 
                                #                                        addi rd, rd, symbol [11:0]
    lw a0, (a5)                 # load word
    li a1, 1                    # load immediate
    li t4, 10                   # load immediate
    for:
        bge a1, t4, end:        # branch >=
        slli t1, a1, 2          # shift left logical imm
        add t2, a5, t1,         # add
        lw t3, (t2)             # load word
        blt t3, a0              # branch <
        mv a0, t3               # mv rd, rs == pseudoinstruction for addi rd, rs, 0
        skip:
        addi a1, a1, 1          # i++
        j for
    end:
    ret                         # return
