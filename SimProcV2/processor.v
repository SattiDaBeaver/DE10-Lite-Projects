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


	// wire MemRead, MemWrite;
	// wire enable;
    // wire [7:0] ADDR, Data_in, Data_out;

    // memory # (.INIT_FILE("machine_code.txt")) MEMORY (
    //     .CLK(CLOCK_50),
    //     .MemRead(MemRead),
    //     .MemWrite(MemWrite),
    //     .ADDR(ADDR),
    //     .Data_in(Data_in),
    //     .Data_out(Data_out)
    // );

    // assign ADDR = SW[7:0];
    // assign MemRead = ~KEY[0];
    // assign MemWrite = ~KEY[1];
    // assign Data_in = {6'b0, SW[9:8]};


    // wire [7:0] IRin, Imm4, Imm5, Imm2, OpCode;
    // wire [3:0] instruction;
    // wire [1:0] RA, RB;
    // wire IRload;

    // IR InstructionRegister (
    //     .CLK(CLOCK_50),
    //     .IRin(IRin),
    //     .IRload(IRload),

    //     .RA(RA),
    //     .RB(RB),
    //     .instruction(instruction),
    //     .Imm4SE(Imm4),
    //     .Imm5ZE(Imm5),
    //     .Imm2ZE(Imm2),
    //     .IRout(OpCode)  
    // );


    assign IRin = SW[7:0];
    assign IRload = ~KEY[0];
    
    assign enable = 1;

	reg_LED REGLED (.CLOCK_50(CLOCK_50), .EN(enable), .Q(SW[9:0]), .LEDR(LEDR[9:0]));
	
	reg_HEX H5(.CLOCK_50(CLOCK_50), .EN(enable), .hex(OpCode[7:4]), .display(HEX5));
	reg_HEX H4(.CLOCK_50(CLOCK_50), .EN(enable), .hex(OpCode[3:0]), .display(HEX4));
	reg_HEX H3(.CLOCK_50(CLOCK_50), .EN(enable), .hex(Imm4[7:4]), .display(HEX3));
	reg_HEX H2(.CLOCK_50(CLOCK_50), .EN(enable), .hex(Imm4[3:0]), .display(HEX2));
	reg_HEX H1(.CLOCK_50(CLOCK_50), .EN(enable), .hex(Imm5[3:0]), .display(HEX1));
	reg_HEX H0(.CLOCK_50(CLOCK_50), .EN(enable), .hex(Imm2[3:0]), .display(HEX0));
endmodule



module datapath(
    input CLOCK_50,         // CLOCK
    input PCwrite,          // Program Counter
    input AddrSel, input MemRead, input MemWrite,   // Memory
    input IRload, input MDRload,     // Instruction Register and Memory Data Register
    input RASel, input RFWrite, input RegIn,      // Register File and Register Address
    input ABLD, input ALU_A, input [2:0] ALU_B,   // ALU and AB load registers/mux
    input [2:0] ALUop, input FlagWrite, input ALUoutLD,  // ALU and NZ Flag

    output [7:0] ALUregOut,    // ALU output
    output [7:0] Aout, output [7:0] Bout,    // Register File A and B
    output [7:0] OpCode,     // Instruction
    output reg N, output reg Z,
    output reg [7:0] muxA, output reg [7:0] muxB,
    output wire [7:0] PCoutWire,
    output wire [3:0] currCycle
    );

    // PC Wires
    wire [7:0] PCout;

    // Memory Wires
    reg [7:0] ADDR;
    wire [7:0] Data_out;

    // Register File Wires
    reg [7:0] dataAreg, dataBreg;
    reg [7:0] dataW;
    wire [7:0] dataA, dataB;

    // Instruction Register and Memory Data Register
    reg [7:0] IRout;
    reg [7:0] MDRout;

    // RA Select wires
    reg [1:0] RASelOut;

    // A and B Muxes
    

    // ALU wires
    wire [7:0] ALUout;
    reg [7:0] registerALU;
    wire Nint, Zint;

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
    memory #(.INIT_FILE("machine_code.txt")) Memory 
    (.CLK(CLOCK_50), .MemRead(MemRead), .MemWrite(MemWrite), .ADDR(ADDR), .Data_in(dataAreg), .Data_out(Data_out));

    // Instruction Register and Memory Data Register
    always @ (posedge CLOCK_50) begin
        if (IRload) begin
            IRout <= Data_out;
        end
        if (MDRload) begin
            MDRout <= Data_out;
        end
    end

    // RA Select 
    always @ (*) begin
        if (RASel == 1'b1) begin
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
            3'b011:     muxB = {3'b000, IRout[7:3]};
            3'b100:     muxB = {6'b000000, IRout[4:3]};
            default:    muxB = dataBreg;
        endcase
    end

    // ALU

    ALU ALUx(.ALUop(ALUop), .A(muxA), .B(muxB), .N(Nint), .Z(Zint), .ALUout(ALUout));

    initial begin
        N = 0;
        Z = 0;
    end

    always @ (posedge CLOCK_50) begin
        if (ALUoutLD) begin
            registerALU <= ALUout;
        end

        if (FlagWrite) begin
            N <= Nint;
            Z <= Zint;
        end
    end 

    // RegIn Mux
    always @ (*) begin
        if (RegIn) begin
            dataW = MDRout;
        end
        else begin
            dataW = registerALU;
        end
    end

    assign Aout = muxA;
    assign Bout = muxB;
    assign OpCode = IRout;
    assign ALUregOut = registerALU;
    assign PCoutWire = PCout;
endmodule

module FSM(
    input CLOCK_50,         // CLOCK

    input [7:0] ALUregOut,    // ALU input
    input [7:0] Aout, input [7:0] Bout,    // Register File A and B
    input [7:0] OpCode,     // Instruction
    input N, input Z,

    output reg PCwrite,          // Program Counter
    output reg AddrSel, output reg MemRead, output reg MemWrite,   // Memory
    output reg IRload, output reg MDRload,     // Instruction Register and Memory Data Register
    output reg RASel, output reg RFWrite, output reg RegIn,      // Register File and Register Address
    output reg ABLD, output reg ALU_A, output reg [2:0] ALU_B,   // ALU and AB load registers/mux
    output reg [2:0] ALUop, output reg FlagWrite, output reg ALUoutLD,  // ALU and NZ Flag
    output [3:0] currCycle
	);

    reg [3:0] currState, nextState;
    reg done;
    wire [3:0] instruction;

    assign instruction = OpCode[3:0];

    parameter IDLE = 4'b0000, CYCLE1 = 4'b0001, CYCLE2 = 4'b0010, CYCLE3 = 4'b0011, CYCLE4 = 4'b0100, CYCLE5 = 4'b0101;
    
    // Instruction OpCodes (ORi, SHIFT are 3 bit)
    parameter ADD = 4'b0100, SUB = 4'b0110, NAND = 4'b1000, ORi = 3'b111, LOAD = 4'b0000;
    parameter STORE = 4'b0010, BNZ = 4'b0101, BPZ = 4'b1001, BZ = 4'b1010, SHIFT = 3'b011;
    parameter SLEFT = 1'b1, SRIGHT = 1'b0, J = 4'b0001;

    initial begin
        currState = IDLE;
    end

    // FSM State Codes
    always @ (*) begin
        case(currState)
            IDLE:       nextState = CYCLE1;
            CYCLE1:     nextState = CYCLE2;
            CYCLE2:     if (done) nextState = CYCLE1;
                        else nextState = CYCLE3;
            CYCLE3:     if (done) nextState = CYCLE1;
                        else nextState = CYCLE4;
            CYCLE4:     if (done) nextState = CYCLE1;
                        else nextState = CYCLE5;
            CYCLE5:     nextState = CYCLE1;
            default:    nextState = CYCLE1;
        endcase
    end

    // FSM Flip Flops

    always @ (posedge CLOCK_50) begin
        currState <= nextState;
    end

    // Output Logic
    always @ (*) begin
        PCwrite = 0;  AddrSel = 0;  MemRead = 0;  
        MemWrite = 0;  IRload = 0;  MDRload = 0;     
        RASel = 0;  RFWrite = 0;  RegIn = 0;
        ABLD = 0;  ALU_A = 0;  ALU_B = 0; 
        ALUop = 0;  FlagWrite = 0;  ALUoutLD = 0;
        done = 0;
        case(currState)
            CYCLE1: begin
                    // IR <- Mem(PC)
                    AddrSel = 1;
                    MemRead = 1;
                    IRload = 1;

                    // PC <- PC + 1
                    ALU_A = 0;
                    ALU_B = 3'b001;
                    ALUop = 0;
                    PCwrite = 1;
            end

            CYCLE2: begin
                    // regA <- IR[7:6], A/B <- RF DataA/B 
                    RASel = 0;
                    ABLD = 1;
            end

            CYCLE3: begin
                if (instruction == ADD | instruction == SUB | instruction == NAND) begin
                    ALU_A = 1;       // Select reg A
                    ALU_B = 3'b000;  // Select reg B
                    ALUoutLD = 1;
                    FlagWrite = 1;
                    case (instruction) 
                        ADD: ALUop = 3'b000;
                        SUB: ALUop = 3'b001;
                        NAND: ALUop = 3'b011;
                        default: ALUop = 3'b000;
                    endcase
                end 

                if (instruction[2:0] == SHIFT) begin
                    ALU_A = 1;       // Select reg A
                    ALU_B = 3'b100;  // ALU B <- ZE(Imm2)
                    ALUoutLD = 1;
                    FlagWrite = 1;
                    case (OpCode[5]) 
                        SLEFT: ALUop = 3'b100;
                        SRIGHT: ALUop = 3'b101;
                        default: ALUop = 3'b100;
                    endcase
                end 

                if (instruction == LOAD) begin
                    AddrSel = 0;    // Select B to be address
                    MDRload = 1;    // Output of memory to MDR
                    MemRead = 1;    // Read memory
                end

                if (instruction == STORE) begin
                    AddrSel = 0;    // Select B to be address
                    MemWrite = 1;    // Write to memory
                    done = 1;
                end

                if (instruction == BNZ | instruction == BPZ | instruction == BZ | instruction == J) begin
                    ALU_A = 0; // ALU A <- PC
                    ALU_B = 3'b010; // ALU B <- SE(Imm4)
                    ALUop = 3'b000;
                    case (instruction) 
                        BPZ: if (!N) PCwrite = 1;
                        BZ: if (Z) PCwrite = 1;
                        BNZ: if (!Z) PCwrite = 1;
                        J: PCwrite = 1;
                    endcase
                    done = 1;
                end

                if (instruction[2:0] == ORi) begin
                    RASel = 1; // RA <- 2'b01
                end

            end

            CYCLE4: begin
                if (instruction == ADD | instruction == SUB | instruction == NAND | instruction[2:0] == SHIFT) begin
                    RegIn = 0;      // dataW <- ALU
                    RFWrite = 1;
                    done = 1;
                end 

                if (instruction == LOAD) begin
                    RegIn = 1;      // dataW <- MDR
                    RFWrite = 1;
                    done = 1;
                end

                if (instruction[2:0] == ORi) begin
                    ALU_A = 1;       // Select reg A
                    ALU_B = 3'b011;  // ALU B <- ZE(Imm5)
                    ALUoutLD = 1;
                    FlagWrite = 1;
                    ALUop = 3'b010;
                end
            end 

            CYCLE5: begin // only for ORi
                RegIn = 0;      // dataW <- ALU
                RFWrite = 1;
                done = 1;
            end
                
        endcase
    end

    assign currCycle = currState;
endmodule

module memory # (
    // Parameters
    parameter   INIT_FILE = ""
    )(
    input                   CLK,
    input                   MemRead,
    input                   MemWrite,
    input       [7:0]       ADDR,
    input       [7:0]       Data_in,
    output reg  [7:0]       Data_out
    );

    reg [7:0] mem [0:255];      // Internal Memory

    // Initialization
    integer i;
    initial begin
        if (INIT_FILE) begin
        $readmemb(INIT_FILE, mem);
        for (i = 0; i < 4; i = i + 1) 
            $display("mem[%0d] = %b", i, mem[i]);
        end
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

module register_file (
	input                   CLOCK_50,
	input                   RFWrite,
	input       [1:0]       regA,
	input       [1:0]       regB,
	input       [1:0]       regW,
	input       [7:0]       dataW,

	output reg  [7:0]       dataA,
	output reg  [7:0]       dataB
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

module PC (
    input                   CLK,
    input       [7:0]       PCin,
    input                   PCwrite,
    output reg  [7:0]       PCout
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

module IR (
    input                   CLK,
    input       [7:0]       IRin,
    input                   IRload,

    output      [1:0]       RA,
    output      [1:0]       RB,
    output      [3:0]       instruction,
    output      [7:0]       Imm4SE,
    output      [7:0]       Imm5ZE,
    output      [7:0]       Imm2ZE,
    output      [7:0]       IRout  
    );

    reg         [7:0]       IRreg;

    initial begin
        IRreg = 0;
    end
    
    always @ (posedge CLK) begin
        if (IRload) begin
            IRreg <= IRin;
        end
    end

    assign IRout = IRreg;
    assign RA = IRreg[7:6];
    assign RB = IRreg[5:4];
    assign instruction = IR[3:0];
    assign Imm4SE = {{4{IRreg[3]}}, IRreg[7:4]};
    assign Imm5ZE = {3'b000, IRreg[7:3]};
    assign Imm2ZE = {6'b000000, IRreg[4:3]};
endmodule

module ALU(
    input       [2:0]       ALUop,
    input       [7:0]       A,
    input       [7:0]       B,
    output                  N,
    output                  Z,
    output reg  [7:0]       ALUout
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

module hex7seg (hex, display);
    input [3:0] hex;
    output [6:0] display;

    reg [6:0] display;

    /*
     *       0  
     *      ---  
     *     |   |
     *    5|   |1
     *     | 6 |
     *      ---  
     *     |   |
     *    4|   |2
     *     |   |
     *      ---  
     *       3  
     */
    always @ (hex)
        case (hex)
            4'h0: display = 7'b1000000;
            4'h1: display = 7'b1111001;
            4'h2: display = 7'b0100100;
            4'h3: display = 7'b0110000;
            4'h4: display = 7'b0011001;
            4'h5: display = 7'b0010010;
            4'h6: display = 7'b0000010;
            4'h7: display = 7'b1111000;
            4'h8: display = 7'b0000000;
            4'h9: display = 7'b0011000;
            4'hA: display = 7'b0001000;
            4'hB: display = 7'b0000011;
            4'hC: display = 7'b1000110;
            4'hD: display = 7'b0100001;
            4'hE: display = 7'b0000110;
            4'hF: display = 7'b0001110;
        endcase
endmodule
	
	

	