module memory # (
    // Parameters
    parameter INIT_FILE = ""
    )(
    input CLK,
    input MemRead,
    input MemWrite,
    input [7:0] ADDR,
    input [7:0] Data_in,
    output reg [7:0] Data_out
    );

    reg [7:0] mem [0:255];      // Internal Memory

    // Initialization
    initial if (INIT_FILE) begin
        $readmemb(INIT_FILE, mem);
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