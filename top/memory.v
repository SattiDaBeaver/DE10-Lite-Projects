module memory(
    input CLK,
    input MemRead,
    input MemWrite,
    input [7:0] ADDR,
    input [7:0] Data_in,
    output reg [7:0] Data_out
    );

    reg [7:0] mem [0:255];      // Internal Memory

    initial begin
        // $readmemh("mem_init.hex", mem);
    end

    always @ (posedge CLK) begin
        if (MemWrite) begin
            mem[ADDR] <= Data_in;
        end
        else if (MemRead) begin
            Data_out <= mem[ADDR];
        end
    end

endmodule