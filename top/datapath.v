module datapath(
    input CLOCK_50,         // CLOCK
    input PCwrite,          // Program Counter
    input AddrSel, input MemRead, input MemWrite,   // Memory
    input IRload, input MDRload,     // Instruction Register and Memory Data Register
    input RASel, input RFWrite, input RegIn,      // Register File and Register Address
    input ABLD, input ALU_A, input [2:0] ALU_B,   // ALU and AB load registers/mux
    input [2:0] ALUop, input FlagWrite, input ALUoutLD,  // ALU and NZ Flag

    output reg [7:0] ALUoutReg,    // ALU output
    output reg [7:0] Aout, output reg [7:0] Bout,    // Register File A and B
    output reg [7:0] OpCode     // Instruction
    );

    wire [7:0] PCout;
    wire [7:0] ALUout;

    // Memory Wires
    wire [7:0] ADDR, Data_in, Data_out;

    // Register File Wires
    reg [7:0] dataAreg, dataBreg;
    reg [7:0] dataW, dataA, dataB;

    // Instruction Register and Memory Data Register
    reg [7:0] IRout;
    reg [7:0] MDRout;

    // RA Select wires
    reg [1:0] RASelOut;

    // A and B Muxes
    reg [7:0] muxA, muxB;


    // PC
    PC progCount(.CLK(CLOCK_50), .PCin(ALUout), .PCwrite(PCwrite), .PCout(PCout));

    // AddrSel Mux
    always @ (*) begin
        if (AddrSel) begin
            ADDR = PCout;
        end
        else begin
            ADDR = dataBreg;
        end
    end

    // Memory
    memory Memory (.CLK(CLOCK_50), .MemRead(MemRead), .MemWrite(MemWrite), .ADDR(ADDR), .Data_in(Data_in), .Data_out(Data_out));

    // Instruction Register and Memory Data Register
    always @ (posedge CLOCK_50) begin
        if (IRLoad) begin
            IRout <= Data_out;
        end
        if (MDRload) begin
            MDRout <= Data_out;
        end
    end

    // RA Select 
    always @ (*) begin
        if (RASel) begin
            RASelOut = 2'b01;
        end
        else begin
            RASelOut = IRout[7:6];
        end
    end

    // Register File 
    register_file RF (.CLOCK_50(CLOCK_50), .RFWrite(RFWrite), .regA(RASelOut), .regB(IRout[5:4]),
							.regW(RASelOut), .dataW(dataW), .dataA(dataA), .dataB(dataB));

    always @ (posedge CLOCK_50) begin
        if (ABLD) begin
            dataAreg <= dataA;
            dataBreg <= dataB;
        end
    end

    // A and B muxes
    always @ (*) begin
        case(ALU_A) 
            1'b0:       muxA = PCout;
            1'b1:       muxA = dataAreg;
        endcase

        case(ALU_B)
            3'b000:     muxB = dataBreg;
            3'b001:     muxB = 8'b00000001;
            3'b010:     muxB = {{4{IRout[7]}}, IRout[7:4]};
            3'b011:     muxB = assign out = {3'b000, IRout[7:3]};
            3'b100:     muxB = {6'b000000, IRout[7:6]};
            default:    muxB = 8'b00000000;
        endcase
    end

endmodule