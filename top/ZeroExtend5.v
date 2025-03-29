module ZeroExtend2 (input [4:0] in, output out [7:0]);
    assign out = {3'b000, in};
endmodule