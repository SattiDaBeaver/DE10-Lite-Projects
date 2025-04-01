module PC (
    input CLK,
    input [7:0] PCin,
    input PCwrite,
    output reg [7:0] PCout
    );

    initial begin
        PCout = 0;
    end

    always @(posedge CLK) begin
        if (PCwrite) begin
            PCout <= PCin;
        end
    end
endmodule