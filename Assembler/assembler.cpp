#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <bitset>

using namespace std;

string getOpCode(fstream& line);
string getRegAB(fstream& line);
string getImm4(fstream& line);
string getImm5(fstream& line);
string getImm2(fstream& line);
string getLR(fstream& line);
string getRegA(fstream& line);

int main(void) {
  // Opcodes
  /*
  ADD = 0100
  SUB = 0110
  NAND = 1000
  ORi = 111
  LOAD = 0000
  STORE = 0010
  BNZ = 1001
  BPZ = 0101
  BZ = 1010
  J = 0001
  SHIFT = 011 (left = 1, right = 0)
  */

  fstream inFile("main.s", ios::in);
  if (!inFile.is_open()) {
    cout << "\"main.s\" Not Found!" << endl;
    return -1;
  }
  fstream outFile("machine_code.txt", ios::out);

  int line = 1;

  while (!inFile.eof()) {
    string instruction = "";
    string OpCode = getOpCode(inFile);

    if (OpCode == ""){
      cout << "Invalid Instruction on line " << line << endl;
      return -1;
    }

    // Add Sub NAND Load Store
    // INSTR Ra, Rb
    if (OpCode == "0100" || OpCode == "0110" || OpCode == "1000" || OpCode == "0010" || OpCode == "0000"){
      string regAB = getRegAB(inFile);
      if (regAB == ""){
        cout << "Invalid Register Reference on line " << line << endl;
        return -1;
      }
      instruction = regAB + OpCode;
    }

    // Branch and Jumps
    // INSTR Imm4
    else if (OpCode == "0101" || OpCode == "1001" || OpCode == "1010" || OpCode == "0001"){
      string Imm4 = getImm4(inFile);
      if (Imm4 == ""){
        cout << "Invalid Integer on line " << line << endl;
        return -1;
      }
      instruction = Imm4 + OpCode;
    }

    // OR Immediate
    // ORi Imm5
    else if (OpCode == "111"){
      string Imm5 = getImm5(inFile);
      if (Imm5 == ""){
        cout << "Invalid Integer on line " << line << endl;
        return -1;
      }
      instruction = Imm5 + OpCode;
    }

    // Shift Left/Right
    // Shift L/R 
    else if (OpCode == "011"){
      string shift = getLR(inFile);
      string regA = getRegA(inFile);
      string Imm2 = getImm2(inFile);
      if (shift == ""){
        cout << "Invalid Character on line " << line << endl;
        return -1;
      }
      if (Imm2 == ""){
        cout << "Invalid Integer on line " << line << endl;
        return -1;
      }

      instruction = regA + shift + Imm2 + OpCode;
    }

    outFile << instruction << endl;
    cout << instruction << endl;
    line++;
  }

  return 0;
}

string getOpCode(fstream& line) {
  string OpCode;
  line >> OpCode;

  if (line.fail()){
    return "";
  }

  transform(OpCode.begin(), OpCode.end(),OpCode.begin(), ::tolower);

  if (OpCode == "add"){
    return "0100";
  } else if (OpCode == "sub"){
    return "0110";
  } else if (OpCode == "nand"){
    return "1000";
  } else if (OpCode == "ori"){
    return "111";
  } else if (OpCode == "load"){
    return "0000";
  } else if (OpCode == "store"){
    return "0010";
  } else if (OpCode == "bpz"){
    return "0101";
  } else if (OpCode == "bnz"){
    return "1001";
  } else if (OpCode == "bz"){
    return "1010";
  } else if (OpCode == "shift"){
    return "011";
  } else if (OpCode == "j"){
  return "0001";
  }
  else {
    return "";
  }
}

string getRegAB(fstream& line) {
  string RA, RB;
  string registers = "";


  line >> RA;
  if (line.fail()){
    return "";
  }

  line >> RB;
  if (line.fail()){
    return "";
  }

  if (RA == "r0,"){
    registers.append("00");
  } else if (RA == "r1,"){
    registers.append("01");
  } else if (RA == "r2,"){
    registers.append("10");
  } else if (RA == "r3,"){
    registers.append("11");
  }
  else {
    return "";
  }

  if (RB == "r0"){
    registers.append("00");
  } else if (RB == "r1"){
    registers.append("01");
  } else if (RB == "r2"){
    registers.append("10");
  } else if (RB == "r3"){
    registers.append("11");
  }
  else {
    return "";
  }
  return registers;
}

string getImm4(fstream& line) {
  int Imm4;
  line >> Imm4;
  if (line.fail()){
    return "";
  }
  if (Imm4 > 7 || Imm4 < -8){
    return "";
  }
  return bitset<4>(Imm4).to_string();
}

string getImm5(fstream& line) {
  unsigned int Imm5;
  line >> Imm5;
  if (line.fail()){
    return "";
  }
  if (Imm5 > 31){
    return "";
  }
  return bitset<5>(Imm5).to_string();
}

string getImm2(fstream& line){
  unsigned int Imm2;
  line >> Imm2;
  if (line.fail()){
    return "";
  }
  if (Imm2 > 3){
    return "";
  }
  return bitset<2>(Imm2).to_string();
}

string getLR(fstream& line){
  char shift;
  line >> shift;
  if (line.fail()){
    return "";
  }

  shift = tolower(shift);

  if (shift == 'l'){
    return "1";
  }
  else if (shift == 'r'){
    return "0";
  }
  return "";
}

string getRegA(fstream& line) {
  string RA;
  string registers = "";

  line >> RA;
  if (line.fail()){
    return "";
  }

  if (RA == "r0,"){
    registers.append("00");
  } else if (RA == "r1,"){
    registers.append("01");
  } else if (RA == "r2,"){
    registers.append("10");
  } else if (RA == "r3,"){
    registers.append("11");
  }
  else {
    return "";
  }
  return registers;
}


