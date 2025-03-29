module SignExtend4 (input [3:0] in, output out [7:0]);
    assign out = {{4{in[3]}}, in};
endmodule