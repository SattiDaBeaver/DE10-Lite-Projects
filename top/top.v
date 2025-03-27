module top (
	input [9:0] SW,
	input [1:0] KEY,
	input CLOCK_50,
	output [9:0] LEDR 
	);
	
	reg [9:0] LED_OUT;
	
	register_file RF (.CLOCK_50(CLOCK_50), .RFWrite(~KEY[0]), .regA(SW[9:8]), .regB(SW[7:6]),
							.regW(SW[5:4]), .dataW({4'b0, SW[3:0]}), .dataA(LED_OUT[9:6]), .dataB(LED_OUT[3:0]));
							
	reg_LED REGLED (.CLOCK_50(CLOCK_50), .EN(~KEY[1]), .Q(LED_OUT), .LEDR(LEDR[9:0]));

endmodule


module reg_LED(input CLOCK_50, input EN, input [9:0] Q, output reg [9:0] LEDR);
	always@ (posedge CLOCK_50) begin
		if (EN)
			LEDR <= Q;
		else
			LEDR <= LEDR;
	end
endmodule
	
	

	