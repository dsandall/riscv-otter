`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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

// allows opcodes to be stored as words rather than codes
// IR[6:2]  opcode for 32-bit instructions   
// first two bits are always 11
// wire CU_OPCODE =CU_IR[6:0];
// wire func3 = CU_IR[14:12];


module OTTER_MCU(
    input CLK,
    input RESET,
    input [31:0] IOBUS_IN,
    output [31:0] IOBUS_OUT,
    output [31:0] IOBUS_ADDR,
    output logic IOBUS_WR 
);
    wire [31:0] pc, pc_in, next_pc, jalr_pc, branch_pc, jump_pc,rs1,rs2,
        immI,immS,immU,immB,immJ,muxOut_aluB,muxOut_aluA,aluOut,rfIn,csr_reg, mem_Dout, Iaddr, IR, aluOPA, aluOPB, DeRS1, DeRS2;
    wire [4:0] rd_addr, rs1_addr, rs2_addr;
    wire [3:0] alu_fun;
    wire [2:0] mem_type;
    wire [1:0] alu_srcB, rf_sel, rf_wr_sel, mSize, pcSource, aluA_fwdSel, aluB_fwdSel, DeRS1_fwdSel, DeRS2_fwdSel;
    wire memRead1,memRead2,pcWrite,regWrite,memWrite,op1_sel,mem_op,IorD,pcWriteCond,memRead,alu_srcA, rs1_used, rs2_used, rd_used, stall;   

    logic [31:0] pr_Ex_opA, pr_Ex_opB, pr_Me_aluRes, pr_WB_aluRes, pr_WB_memData;

    logic [31:0] fwd_data;

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

    typedef struct packed {
        opcode_t opcode;        //IR[6:0]
        logic [31:0] IR;
        logic [4:0] rs1_addr;   //IR[19:15]
        logic [4:0] rs2_addr;   //IR[24:20]
        logic [4:0] rd_addr;    //IR[11:7]
        logic rs1_used;         //boolean
        logic rs2_used;         //boolean
        logic rd_used;          //boolean
        logic [3:0] alu_fun;    //dcdr
        logic memWrite;         //dcdr
        logic memRead2;         //dcdr
        logic regWrite;         //dcdr
        logic [1:0] rf_wr_sel;  //dcdr
        logic [2:0] mem_type;   //sign 14, size 13-12 IR[14:12]
        logic [31:0] pc;        //from pc
        logic [31:0] rs1;
        logic [31:0] rs2;
        //  fwding registers
        logic [31:0] fwd_data;
        logic [1:0] aluA_fwdSel, aluB_fwdSel;

    } instr_t;

    opcode_t OPCODE;    //create object OPCODE

    instr_t pr_De, pr_Ex, pr_Me, pr_WB, pr_CLR; 



//==== Instruction Fetch ===========================================
    // ==== pipeline logic ==== 
    always_ff @(posedge CLK) begin
        if (!stall) begin
            pr_De.pc        <= Iaddr;   
        end
    end
     
    // ==== fetch stage components ==== 
    assign pcWrite = !stall; 	//increment unless stalling
    //assign memRead1 = !stall; 	//replaced with !stall in field
    
    //PC and Branch Mux swapped to allow quicker branch times, and simplify branch hazard logic
    assign pc_in = Iaddr + 4;    
    ProgCount PC (CLK,RESET,pcWrite,pc_in,pc); 

    Mult4to1 BranchMux (pc, jalr_pc, branch_pc, jump_pc, pcSource, Iaddr);
    
    OTTER_mem_byte #(14) memory  (
        //inputs
        .MEM_CLK(CLK),
        .MEM_ADDR1(Iaddr),
        .MEM_ADDR2(pr_Me_aluRes),
        .MEM_DIN2(pr_Me.rs2),
        .MEM_WRITE2(pr_Me.memWrite),
        .MEM_READ1(!stall),
        .MEM_READ2(pr_Me.memRead2),
        .MEM_SIZE(pr_Me.IR[13:12]), 
        .MEM_SIGN(pr_Me.IR[14]),
        .IO_IN(IOBUS_IN),
        //outputs
        //.ERR(),
        .MEM_DOUT1(IR),
        .MEM_DOUT2(mem_Dout),
        .IO_WR(IOBUS_WR)
    );

//==== Instruction Decode ===========================================
    // ==== pipeline logic ==== 
    
    assign OPCODE = IR[6:0];  //assign object actual value     
    
    always_ff @(posedge CLK) begin
        if (!stall) begin
            //nonstruct vars (from curr stage)
            pr_Ex_opA       <= muxOut_aluA;
            pr_Ex_opB       <= muxOut_aluB;
            //struct vars
            pr_Ex           <= pr_De;   
            pr_Ex.rs1       <= DeRS1;
            pr_Ex.rs2       <= DeRS2;
            pr_Ex.IR        <= IR;
            pr_Ex.opcode    <= OPCODE;  
            pr_Ex.rs1_addr  <= rs1_addr;
            pr_Ex.rs2_addr  <= rs2_addr;
            pr_Ex.rd_addr   <= rd_addr;
            pr_Ex.rs1_used  <= rs1_used;
            pr_Ex.rs2_used  <= rs2_used;
            pr_Ex.rd_used   <= rd_used;
            pr_Ex.alu_fun   <= alu_fun;
            pr_Ex.memWrite  <= memWrite;
            pr_Ex.memRead2  <= memRead2;
            pr_Ex.regWrite  <= regWrite;
            pr_Ex.rf_wr_sel <= rf_wr_sel;
            pr_Ex.mem_type  <= mem_type;
            pr_Ex.aluA_fwdSel<=aluA_fwdSel;
            pr_Ex.aluB_fwdSel<=aluB_fwdSel;
            end
        else begin
            pr_Ex           <= pr_CLR;   
            pr_Ex_opA       <= 0;
            pr_Ex_opB       <= 0;
            end
    end


    // ==== DE stage components ==== 

    OTTER_CU_Decoder CU_DECODER(
        .CU_IR(IR),
        .CU_REGWRITE(regWrite),
        .CU_MEMWRITE(memWrite),
        .CU_MEMREAD2(memRead2),
        .CU_ALU_SRCA(alu_srcA),
        .CU_ALU_SRCB(alu_srcB),
        .CU_ALU_FUN(alu_fun),
        .CU_RF_WR_SEL(rf_wr_sel),
        .RS1_USED(rs1_used),
        .RS2_USED(rs2_used),
        .RD_USED(rd_used),
        .RD_ADDR(rd_addr),
        .RS1_ADDR(rs1_addr),
        .RS2_ADDR(rs2_addr),
        .MEM_TYPE(mem_type)
        );

    OTTER_registerFile RF (
        //inputs
        .Read1(rs1_addr),
        .Read2(rs2_addr),
        .WriteReg(pr_WB.rd_addr),   //WB
        .WriteData(rfIn),           //WB
        .RegWrite(pr_WB.regWrite),  //WB
        .clock(CLK),
        //outputs
        .Data1(rs1),
        .Data2(rs2)
        ); 

    // immed gen (p474 otter TB)
    assign immS = {{21{IR[31]}},IR[30:25],IR[11:7]};
    assign immI = {{21{IR[31]}},IR[30:20]}; 
    assign immU = {IR[31:12],12'b0};
    assign immB = {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};
    assign immJ = {{12{IR[31]}}, IR[19:12], IR[20],IR[30:21],1'b0};

    //Target Generator
    assign jalr_pc      = DeRS1 + immI;   //rs1 + I imm
    assign branch_pc    = pr_De.pc + immB;    //PC + B imm
    assign jump_pc      = pr_De.pc + immJ;    //PC + J imm  

    Mult4to1 DeRS1fwd (rs1, fwd_data, pr_Me.fwd_data, pr_WB.fwd_data, DeRS1_fwdSel, DeRS1);
    Mult4to1 DeRS2fwd (rs2, fwd_data, pr_Me.fwd_data, pr_WB.fwd_data, DeRS2_fwdSel, DeRS2);

    branch_cond BRANCH_COND (DeRS1,DeRS2,mem_type,IR[6:0],pcSource);    

    Mult2to1 ALUAinput (rs1, immU, alu_srcA, muxOut_aluA);  
    Mult4to1 ALUBinput (rs2, immI, immS, pr_De.pc, alu_srcB, muxOut_aluB);

    hazard_control HazardCTRL (
        .IR(IR),
        .RS1(rs1_addr),
        .RS2(rs2_addr), 
        .RD_EX(pr_Ex.rd_addr), 
        .RD_ME(pr_Me.rd_addr), 
        .RD_WB(pr_WB.rd_addr), 
        .RS1_USED(rs1_used), 
        .RS2_USED(rs2_used), 
        .RD_USED_EX(pr_Ex.rd_used), 
        .RD_USED_ME(pr_Me.rd_used),  
        .RD_USED_WB(pr_WB.rd_used),  
        .STALL(stall), 
        .DeRS1_FWDSEL(DeRS1_fwdSel), 
        .DeRS2_FWDSEL(DeRS2_fwdSel), 
        .ALUA_FWDSEL(aluA_fwdSel), 
        .ALUB_FWDSEL(aluB_fwdSel), 
        .OP_EX(pr_Ex.IR[6:0]),
        .OP_ME(pr_Me.IR[6:0]),
        .OP_WB(pr_WB.IR[6:0])
        );
   

    




//==== Execute ======================================================
    // ==== pipeline logic ==== 

    always_ff @(posedge CLK) begin
        pr_Me_aluRes        <= aluOut;
        pr_Me               <= pr_Ex;

        pr_Me.fwd_data       <= fwd_data;
    end
    


    // ==== EX stage components ==== 
    //assign aluA_fwdSel =0, aluB_fwdSel =0;

    Mult4to1 ALUAfwd (pr_Ex_opA, pr_Me.fwd_data, pr_WB.fwd_data, 32'b0, pr_Ex.aluA_fwdSel, aluOPA);
    Mult4to1 ALUBfwd (pr_Ex_opB, pr_Me.fwd_data, pr_WB.fwd_data, 32'b0, pr_Ex.aluB_fwdSel, aluOPB);

    OTTER_ALU ALU (pr_Ex.alu_fun, aluOPA, aluOPB, aluOut); // the ALU

    //fwd_store
    always_comb begin
        if (pr_Ex.rd_used) begin
            case(pr_Ex.opcode)
                LUI:     fwd_data = aluOut;
                AUIPC:   fwd_data = aluOut;
                OP_IMM:  fwd_data = aluOut;
                OP:      fwd_data = aluOut;
                JAL:     fwd_data = pr_Ex.pc + 4;
                JALR:    fwd_data = pr_Ex.pc + 4;
                default: fwd_data = 0;
            endcase
        end
        else fwd_data = 0;
    end

//==== Memory ======================================================
    // ==== pipeline logic ==== 
    //  link IO outputs to mem stage data
    assign IOBUS_ADDR = pr_Me_aluRes;
    assign IOBUS_OUT = pr_Me.rs2;

    always_ff @(posedge CLK) begin
        pr_WB_aluRes    <= pr_Me_aluRes;
        //pr_WB_memData   <= mem_Dout;
        pr_WB           <= pr_Me;
    end


    
    // ==== MEM stage components ==== 
    //  MEM module already included in fetch, wiring done there

 
     
//==== Write Back ==================================================
    // ==== pipeline logic ==== 
        //no pipeline logic, final stage     

    // ==== WB stage components ==== 
    assign csr_reg = 0; //no csr in current version
    Mult4to1 regWriteback ((pr_WB.pc+4),csr_reg,mem_Dout,pr_WB_aluRes,pr_WB.rf_wr_sel,rfIn);

 
 

       
            
endmodule
