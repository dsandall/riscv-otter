`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/09/2022 11:08:07 AM
// Design Name: 
// Module Name: branch_gen
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


module branch_cond(
    input  [31:0] RS1,        //IR [19:15]
    input  [31:0] RS2,        //IR [24:20]
    input  [2:0] FUNC3,      //IR [14:12]
    input  [6:0] OPCODE,           //IR [6:0]
    output logic [1:0] PCSOURCE
);

    typedef enum logic [6:0] {
            LUI      = 7'b0110111,
            AUIPC    = 7'b0010111,
            JAL      = 7'b1101111,
            JALR     = 7'b1100111,
            BRANCH   = 7'b1100011,
            LOAD     = 7'b0000011,
            STORE    = 7'b0100011,
            OP_IMM   = 7'b0010011,
            OP       = 7'b0110011,
            SYSTEM   = 7'b1110011
        } opcode_t;

    reg brn_cond;
    reg br_lt;
    reg br_ltu;
    reg br_eq;

    always_comb begin
        br_lt=0; br_eq=0; br_ltu=0;
        if($signed(RS1) < $signed(RS2)) br_lt=1;
        if(RS1==RS2) br_eq=1;
        if(RS1<RS2) br_ltu=1;
        
        case(FUNC3)
                3'b000: brn_cond = br_eq;     //BEQ 
                3'b001: brn_cond = ~br_eq;    //BNE
                3'b100: brn_cond = br_lt;     //BLT
                3'b101: brn_cond = ~br_lt;    //BGE
                3'b110: brn_cond = br_ltu;    //BLTU
                3'b111: brn_cond = ~br_ltu;   //BGEU
                default: brn_cond =0;
        endcase

        case(OPCODE)
            JALR: PCSOURCE =2'b01;
            BRANCH: PCSOURCE=(brn_cond)?2'b10:2'b00;
            JAL: PCSOURCE=2'b11;                    
            default: PCSOURCE=2'b00; 
        endcase
    end

endmodule
