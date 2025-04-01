0 loop: load R2,(R0)    # R2 <- mem(R0)     10 00 0000 = 0x80
1       load R3,(R1)    # R3 <- mem(R1)     11 01 0000 = 0xd0
2       sub R2, R3      # R2 <- R2 - R3     10 11 0110 = 0xb6
3       bpz loop        #                   1101 1001 = 0xd9