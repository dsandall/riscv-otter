`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/18/2022 07:19:41 PM
// Design Name: 
// Module Name: hazard_control
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


module hazard_control(
    input [31:0] IR,
    input [4:0] RS1, RS2, RD_EX, RD_ME, RD_WB,
    input RS1_USED, RS2_USED, RD_USED_EX, RD_USED_ME, RD_USED_WB, 
    input [6:0] OP_EX, OP_ME, OP_WB,
    output logic STALL, 
    output logic [1:0] DeRS1_FWDSEL, DeRS2_FWDSEL, ALUA_FWDSEL, ALUB_FWDSEL

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
    //assign OPCODE = opcode_t'(IR[6:0]);  //assign object actual value
    assign OPCODE = IR[6:0];  //assign object actual value
    opcode_t OPCODE_EX, OPCODE_ME, OPCODE_WB;    //create object OPCODE
    assign OPCODE_EX = OP_EX;  //assign object actual value
    assign OPCODE_ME = OP_ME;  //assign object actual value
    assign OPCODE_WB = OP_WB;  //assign object actual value







    logic rs1_stall, rs2_stall, rs1_haz, rs2_haz;


always_comb begin
    // defaults
    rs1_stall      = 0;
    rs2_stall      = 0;
    ALUA_FWDSEL    = 0;
    DeRS1_FWDSEL   = 0;
    ALUB_FWDSEL    = 0;
    DeRS2_FWDSEL   = 0;
    STALL          = 0;

    // RS1 HAZARD
    if ((RS1 == RD_EX) & RD_USED_EX & RS1_USED) begin
        if (OPCODE_EX == LOAD) begin
             rs1_stall = 1;
        end else begin
            case(OPCODE)
                OP,OP_IMM,STORE: ALUA_FWDSEL   = 1;
                BRANCH,JALR:     DeRS1_FWDSEL  = 1;
                LOAD:            ALUA_FWDSEL   = 1;
                default:         rs1_stall     = 1;
            endcase
        end
    end else if ((RS1 == RD_ME) & RD_USED_ME & RS1_USED) begin
        if (OPCODE_ME == LOAD) rs1_stall = 1;
        else begin
            case(OPCODE)
                OP,OP_IMM,STORE: ALUA_FWDSEL   = 2;
                BRANCH,JALR:     DeRS1_FWDSEL  = 2;
                LOAD:            ALUA_FWDSEL   = 2;
                default:         rs1_stall     = 1;
            endcase
        end
    end

    // RS2 HAZARD
    if ((RS2 == RD_EX) & RD_USED_EX & RS2_USED) begin
        if (OPCODE_EX == LOAD) rs2_stall = 1;
        else begin
            case(OPCODE)
                OP:     ALUB_FWDSEL   = 1;
                STORE:  DeRS2_FWDSEL  = 1;
                BRANCH: DeRS2_FWDSEL  = 1;
                default: rs2_stall    = 1;
            endcase
        end
    end else if ((RS2 == RD_ME) & RD_USED_ME & RS2_USED) begin
        if (OPCODE_ME == LOAD) rs2_stall = 1;
        else begin
            case(OPCODE)
                OP:     ALUB_FWDSEL   = 2;
                STORE:  DeRS2_FWDSEL  = 2;
                BRANCH: DeRS2_FWDSEL  = 2;
                default: rs2_stall    = 1;
            endcase
        end
    end

    STALL = rs1_stall || rs2_stall;
end

/*  PSEUDOCODE, TO ORGANIZE BRAINWAVES ON DIGITAL PAPER  
    always_comb begin

        if (OPCODE == R, I, S and rs1_haz) begin
            if rs1addr == rdaddr_ex then send pr_me.RDdata (one cycle after) to ALUA and rs1_fwd =1
            if rs1addr == rdaddr_me then send pr_wb.RDdata (one cycle after) to ALUA and rs1_fwd =1
        end
        if (opcode == R and rs2_haz)
            if rs2addr == rdaddr_ex then send pr_me.RDdata (one cycle after) to ALUB and rs2_fwd =1
            if rs2addr == rdaddr_me then send pr_wb.RDdata (one cycle after) to ALUB and rs2_fwd =1
        if (opcode == S and rs2_haz)
            if rs2addr == rdaddr_ex rs2_fwd =1
            if rs2addr == rdaddr_me rs2_fwd =1
            // rs2 apparrently not an issue now that i think abt it, can remove mux entirely

        if (opcode == Branch and rs1_haz)
            if rs1addr == rdaddr_ex, you gotta wait 
            if rs1addr == rdaddr_me, then send fwd_data (not pr_Me data, because it updates too late) and rs1_fwd =1
            (and if rs1_addr == rdaddr_wb, it will work, disable stall)

        if (opcode == branch and rs2_haz)
            if rs2addr == rdaddr_ex, you gotta wait 
            if rs2addr == rdaddr_me, then send fwd_data (not pr_Me data, because it updates too late) and rs2_fwd =1
            (and if rs2_addr == rdaddr_wb, it will work, disable stall)

        case(OPCODE)
                STORE: ;
            default: ;
        endcase
    end
*/
endmodule
