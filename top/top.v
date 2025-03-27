module top (
	input [9:0] SW,
	input [1:0] KEY,
	input CLOCK_50,
	output [9:0] LEDR 
	);
	

	reg_LED REGLED (.CLOCK_50(CLOCK_50), .EN(~KEY[0]), .Q(SW[9:0]), .LEDR(LEDR[9:0]));

endmodule


module reg_LED(input CLOCK_50, input EN, input [9:0] Q, output reg [9:0] LEDR);
	always@ (posedge CLOCK_50) begin
		if (EN)
			LEDR <= Q;
		else
			LEDR <= LEDR;
	end
endmodule
	
	

	