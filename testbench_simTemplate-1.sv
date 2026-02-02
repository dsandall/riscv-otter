`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/24/2018 08:37:20 AM
// Design Name: 
// Module Name: simTemplate
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
// b asdfasdfasdfasdf  asas    
// b asdfasdfasdfasdf  asas  as           
//////////////////////////////////////////////////////////////////////////////////
module simTemplate(
     );
    
     reg CLK=0,BTNL=0,BTNC=0/*,PS2Clk,PS2Data,VGA_HS,VGA_VS,Tx*/;
     reg [15:0] SWITCHES,LEDS;
     reg [7:0] CATHODES/*,VGA_RGB*/;
     reg [3:0] ANODES;
   
     OTTER_Wrapper ottr(.sclk(CLK), .BTNL(BTNL), .BTNC(BTNC),
      .SWITCHES(SWITCHES), .LEDS(LEDS), .CATHODES(CATHODES), .ANODES(ANODES));
      
    initial begin
        #10 
        BTNC=0;
        SWITCHES=15'd0;

      //$finish;
    end
    
    initial forever  #10  CLK =  ! CLK; 

    
       
  /*  initial begin
         if(ld_use_hazard)
            $display("%t -------> Stall ",$time);
        if(branch_taken)
            $display("%t -------> branch taken",$time); 
      end*/
endmodule
