`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Callenes
//           Dylan Sandall
// 
// Create Date: 01/27/2019 09:22:55 AM
// Design Name: 
// Module Name: CU_Decoder
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module OTTER_CU_Decoder(
//  Edited to accept all IR as input rather than seperate chunks
//    input [6:0] CU_OPCODE, //changed to CU_IR[6:0]
//    input [2:0] CU_FUNC3,  //changed to CU_IR[14:12]
//    input [6:0] CU_FUNC7,  //changed to CU_IR[31:25]

    input [31:0] CU_IR,
    output logic CU_REGWRITE,
    output logic CU_MEMWRITE,
    output logic CU_MEMREAD2,
    output logic CU_ALU_SRCA,
    output logic [1:0] CU_ALU_SRCB,
    output logic [3:0] CU_ALU_FUN,
    output logic [1:0] CU_RF_WR_SEL,
    output logic RS1_USED,
    output logic RS2_USED,
    output logic RD_USED,
    output logic [4:0] RD_ADDR,
    output logic [4:0] RS1_ADDR, 
    output logic [4:0] RS2_ADDR,
    output logic [2:0] MEM_TYPE
   );

        // turns IR into opcode object, made more readable
        typedef enum logic [6:0] {
            LUI      = 7'b0110111, //U type, writes U immediate (alu output) to RD 
            AUIPC    = 7'b0010111, //U type, writes U + PC (alu output) to RD
            JAL      = 7'b1101111, //J type, PC+4 to RD
            JALR     = 7'b1100111, //I type, writes PC+4 to RD
            BRANCH   = 7'b1100011, //B type, does not write to RD
            LOAD     = 7'b0000011, //I type, writes memory to RD
            STORE    = 7'b0100011, //S type, does not write to RD
            OP_IMM   = 7'b0010011, //I type, writes ALU to RD
            OP       = 7'b0110011, //R type, writes ALU to RD
            SYSTEM   = 7'b1110011
        } opcode_t;
        opcode_t OPCODE;    //create object OPCODE
        //assign OPCODE = opcode_t'(CU_IR[6:0]);  //assign object actual value
        assign OPCODE = CU_IR[6:0];  //assign object actual value

        //  FOR ALU source select
        //  ALU src is 0 (rs1), unless U type
        assign CU_ALU_SRCA = (OPCODE==LUI || OPCODE==AUIPC) ? 1 : 0;

        // FOR MEM
        assign CU_MEMWRITE = OPCODE == STORE;
        assign CU_MEMREAD2 = OPCODE == LOAD;
                
        // FOR WB
        assign CU_REGWRITE = (OPCODE!=BRANCH) && (OPCODE!=STORE);

        always_comb begin
            //  FOR ALU operation control
            case(OPCODE)
                // if opcode is type immediate, and func3 code is 101, 
                // set alufun to IR[30]+func3 else 0+func3
                    OP_IMM: CU_ALU_FUN= (CU_IR[14:12]==3'b101)?{CU_IR[30],CU_IR[14:12]}:{1'b0,CU_IR[14:12]};
                // if opcode is LUI or SYS, set alufun to 1001 
                    LUI,SYSTEM: CU_ALU_FUN = 4'b1001;
                // if opcode is OP, 
                    OP: CU_ALU_FUN = {CU_IR[30],CU_IR[14:12]};
                // else, set to 0
                default: CU_ALU_FUN = 4'b0;
            endcase

            //  FOR ALU source select
            case(OPCODE)
                STORE:  CU_ALU_SRCB=2;  //S-type
                LOAD:   CU_ALU_SRCB=1;  //I-type
                JAL:    CU_ALU_SRCB=1;  //I-type
                OP_IMM: CU_ALU_SRCB=1;  //I-type
                AUIPC:  CU_ALU_SRCB=3;  //U-type (special) LUI does not use B
                default:CU_ALU_SRCB=0;  //R-type    //OP  BRANCH-does not use
            endcase
            
            // FOR WB source select
            case(OPCODE)
                JAL:    CU_RF_WR_SEL=0;
                JALR:    CU_RF_WR_SEL=0;
                LOAD:    CU_RF_WR_SEL=2;
                SYSTEM:  CU_RF_WR_SEL=1;
                default: CU_RF_WR_SEL=3; 
            endcase
        end

        assign RD_ADDR   = CU_IR[11:7];
        assign RS1_ADDR  = CU_IR[19:15]; 
        assign RS2_ADDR  = CU_IR[24:20];
        assign MEM_TYPE  = CU_IR[14:12];
        assign RS1_USED        = RS1_ADDR != 0
                            && OPCODE != LUI
                            && OPCODE != AUIPC
                            && OPCODE != JAL;
        assign RS2_USED        = RS2_ADDR != 0 
                         &&(OPCODE == BRANCH
                            | OPCODE == STORE
                            | OPCODE == OP);
        assign RD_USED        = RD_ADDR != 0
                            && OPCODE != BRANCH
                            && OPCODE != STORE;                
endmodule
