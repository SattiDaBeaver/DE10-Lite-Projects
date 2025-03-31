module processor (
	input [9:0] SW,
	input [1:0] KEY,
	input CLOCK_50,

	output [6:0] HEX5,
	output [6:0] HEX4,
	output [6:0] HEX3,
	output [6:0] HEX2,
	output [6:0] HEX1,
	output [6:0] HEX0,
	output [9:0] LEDR 
	);
	
	wire [9:0] LED_OUT;

	// Datapath and FSM Wires
	// Memory
	wire PCwrite, AddrSel, MemRead, MemWrite, IRload, MDRloadm, RASel, RFWrite;
	wire RegIn, ABLD, ALU_A, FlagWrite, ALUoutLD;
	wire [2:0] ALU_B, ALUop;

	wire [7:0] ALUregOut, Aout, Bout, OpCode;
	wire N, Z;

	// Register File
	wire [7:0] dataA, dataB, dataW;
	wire [3:0] regA, regB, regW;

	// Processor Modules

	datapath DATAPATH(.CLOCK_50(CLOCK_50), .PCwrite(PCwrite), .AddrSel(AddrSel), .MemRead(MemRead), .MemWrite(MemWrite), .IRload(IRload), .MDRload(MDRload),
			 .RASel(RASel), .RFWrite(RFWrite), .RegIn(RegIn), .ABLD(ABLD), .ALU_A(ALU_A), .ALU_B(ALU_B), .ALUop(ALUop), .FlagWrite(FlagWrite), .ALUoutLD(ALUoutLD),
			 .ALUregOut(ALUregOut), .Aout(Aout), .Bout(Bout), .OpCode(OpCode), .N(N), .Z(Z)
    );

	// I/O Modules
	
	assign LED_OUT = {8'b0, N, Z};

	reg_LED REGLED (.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .Q(LED_OUT), .LEDR(LEDR[9:0]));
	
	reg_HEX H5(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(ALUregOut[7:4]), .display(HEX5));
	reg_HEX H4(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(ALUregOut[3:0]), .display(HEX4));
	reg_HEX H3(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(Aout[7:4]), .display(HEX3));
	reg_HEX H2(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(Aout[3:0]), .display(HEX2));
	reg_HEX H1(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(Bout[7:4]), .display(HEX1));
	reg_HEX H0(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(Bout[3:0]), .display(HEX0));
endmodule


module reg_LED(input CLOCK_50, input EN, input [9:0] Q, output reg [9:0] LEDR);
	always @ (posedge CLOCK_50) begin
		if (EN)
			LEDR <= Q;
		else
			LEDR <= LEDR;
	end
endmodule

module reg_HEX(input CLOCK_50, input EN, input [3:0] hex, output reg [6:0] display);
	wire [6:0] data;
	hex7seg SEG(.hex(hex), .display(data));
	always @ (posedge CLOCK_50) begin
		if (EN)
			display <= data;
		else
			display <= display;
	end
endmodule	
	
	

	