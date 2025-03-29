module top (
	input [9:0] SW,
	input [1:0] KEY,
	output [6:0] HEX3,
	output [6:0] HEX2,
	output [6:0] HEX1,
	output [6:0] HEX0,
	input CLOCK_50,
	output [9:0] LEDR 
	);
	
	wire [9:0] LED_OUT;
	wire [7:0] dataA, dataB, dataW;
	wire [3:0] regA, regB, regW;
	wire RFWrite;
	
	assign RFWrite = ~KEY[0];
	assign regA = SW[9:8];
	assign regB = SW[7:6];
	assign regW = SW[5:4];
	assign dataW = {4'b0000, SW[3:0]};
	
	register_file RF (.CLOCK_50(CLOCK_50), .RFWrite(RFWrite), .regA(regA), .regB(regB),
							.regW(regW), .dataW(dataW), .dataA(dataA), .dataB(dataB));
							
	assign LED_OUT = {dataA[3:0], 2'b00, dataW[3:0]};
							
	reg_LED REGLED (.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .Q(LED_OUT), .LEDR(LEDR[9:0]));
	
	reg_HEX H3(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(dataA[7:4]), .display(HEX3));
	reg_HEX H2(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(dataA[3:0]), .display(HEX2));
	reg_HEX H1(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(dataB[7:4]), .display(HEX1));
	reg_HEX H0(.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .hex(dataB[3:0]), .display(HEX0));
endmodule


module reg_LED(input CLOCK_50, input EN, input [9:0] Q, output reg [9:0] LEDR);
	always@ (posedge CLOCK_50) begin
		if (EN)
			LEDR <= Q;
		else
			LEDR <= LEDR;
	end
endmodule

module reg_HEX(input CLOCK_50, input EN, input [3:0] hex, output reg [6:0] display);
	wire [6:0] data;
	hex7seg SEG(.hex(hex), .display(data));
	always@ (posedge CLOCK_50) begin
		if (EN)
			display <= data;
		else
			display <= display;
	end
endmodule	
	
	

	