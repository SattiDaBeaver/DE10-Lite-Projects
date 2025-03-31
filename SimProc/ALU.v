module ALU(
    input [2:0] ALUop,
    input [7:0] A,
    input [7:0] B,
    output N,
    output Z,
    output reg [7:0] ALUout
    );

    parameter ADD = 3'b000, SUB = 3'b001, OR = 3'b010, NAND = 3'b011, ShiftLeft = 3'b100, ShiftRight = 3'b101;
    always @ (*) begin
        case(ALUop) 
            ADD:        ALUout = A + B;
            SUB:        ALUout = A - B;
            OR:         ALUout = A | B;
            NAND:       ALUout = ~(A & B);
            ShiftLeft:  ALUout = A << B;
            ShiftRight: ALUout = A >> B;
            default:    ALUout = 8'b0;
        endcase
    end

    assign N = ALUout[7];
    assign Z = ~(|ALUout);
endmodule