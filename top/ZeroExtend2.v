module ZeroExtend2 (input [1:0] in, output out [7:0]);
    assign out = {6'b000000, in};
endmodule