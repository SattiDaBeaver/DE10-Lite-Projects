module register_file (
	input CLOCK_50,
	input RFWrite,
	input [1:0] regA,
	input [1:0] regB,
	input [1:0] regW,
	input [7:0] dataW,

	output reg [7:0] dataA,
	output reg [7:0] dataB
);
	reg [7:0] r0, r1, r2, r3;
	parameter R0 = 2'b00, R1 = 2'b01, R2 = 2'b10, R3 = 2'b11;

	initial begin
		r0 = 0;
		r1 = 0;
		r2 = 0;
		r3 = 0;
	end	

	always @ (posedge CLOCK_50) begin
		if (RFWrite) begin
			case(regW[1:0])
				R0: r0 <= dataW;
				R1: r1 <= dataW;
				R2: r2 <= dataW;
				R3: r3 <= dataW;
			endcase
		end
	end

	always @ (*) begin
			case(regA[1:0])
				R0: dataA = r0;
				R1: dataA = r1;
				R2: dataA = r2;
				R3: dataA = r3;
			endcase

			case(regB[1:0])
				R0: dataB = r0;
				R1: dataB = r1;
				R2: dataB = r2;
				R3: dataB = r3;
			endcase
	end


endmodule