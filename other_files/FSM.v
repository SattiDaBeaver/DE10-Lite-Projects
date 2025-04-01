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
    output reg [2:0] ALUop, output reg FlagWrite, output reg ALUoutLD  // ALU and NZ Flag
);

    reg [3:0] currState, nextState;
    reg done;
    wire [3:0] instruction;

    assign instruction = OpCode[3:0];

    parameter CYCLE1 = 4'b0000, CYCLE2 = 4'b0001, CYCLE3 = 4'b0010, CYCLE4 = 4'b0011, CYCLE5 = 4'b0100;
    
    // Instruction OpCodes (ORi, SHIFT are 3 bit)
    parameter ADD = 4'b0100, SUB = 4'b0110, NAND = 4'b1000, ORi = 3'b111, LOAD = 4'b0000;
    parameter STORE = 4'b0010, BNZ = 4'b0101, BPZ = 4'b1001, BZ = 4'b1010, SHIFT = 3'b011;
    parameter SLEFT = 1'b1, SRIGHT = 1'b0;

    initial begin
        currState = 0;
    end

    // FSM State Codes
    always @ (*) begin
        case(currState)
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

                if (instruction == BNZ | instruction == BPZ | instruction == BZ) begin
                    ALU_A = 0; // ALU A <- PC
                    ALU_B = 3'b010; // ALU B <- SE(Imm4)
                    ALUop = 3'b000;
                    case (instruction) 
                        BPZ: if (!N) PCwrite = 1;
                        BZ: if (Z) PCwrite = 1;
                        BNZ: if (!Z) PCwrite = 1;
                        default: PCwrite = 0;
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

endmodule