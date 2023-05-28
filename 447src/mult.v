/**
 * mult.v
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is a multiplier module that can be used by the main processor.
 *
 * The output O is the unsigned half-product of A*B, matching the mul 
 * instruction in RISC-V.
 * 
 * mult() can be combinational (~5ns in 18447 synthesis) or 
 * PIPELINED (~4ns stage 1 and ~1ns stage 2).
 * 
 * When pipelined, multf() has a shorter (less than 3ns) 1st stage 
 * but uses more pipeline flip-flops. Combinational mode is also faster.
 * 
 * PRECISION is 32 by default and can be any multiple of 2 (or 4) for mult (or multf).  
 *
 * Authors:
 *  - 2022: James Hoe
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

module mult(A, B, O, CLK);
   parameter PIPELINED=1, PRECISION=32; // divisible by 2
   
   input [PRECISION-1:0] A, B;
   output [PRECISION-1:0] O;
   input CLK;
   
   // intermidate values; no need for aubu 
   wire [PRECISION-1:0] albl;
   wire [(PRECISION/2)-1:0] aubl, albu;
   
   assign albl=A[(PRECISION/2)-1:0]*B[(PRECISION/2)-1:0];
   assign aubl=A[PRECISION-1:(PRECISION/2)]*B[(PRECISION/2)-1:0];
   assign albu=A[(PRECISION/2)-1:0]*B[PRECISION-1:(PRECISION/2)];

   // latched versions will be deadcode eliminated if not PIPELINED   
   reg [PRECISION-1:0] albl_ff;
   reg [(PRECISION/2)-1:0] aubl_ff, albu_ff;
   always @(posedge CLK) begin
      albl_ff<=albl;
      aubl_ff<=aubl;
      albu_ff<=albu;
   end
   
   // one of the two will be deadcode eliminated during synthesis
   wire [PRECISION-1:0]   O_comb, O_ff;
   assign O_comb=albl+(aubl<<(PRECISION/2))+(albu<<(PRECISION/2));
   assign O_ff=albl_ff+(aubl_ff<<(PRECISION/2))+(albu_ff<<(PRECISION/2));

   assign O=PIPELINED?O_ff:O_comb;
   
endmodule // mult

module multf(A, B, O, CLK);
   parameter PIPELINED=1, PRECISION=32; // devisible by 4
   
   input [PRECISION-1:0] A, B;
   output [PRECISION-1:0] O;
   input CLK;
   
   wire [(PRECISION/2)-1:0] albl_ll, albl_ul, albl_lu, albl_uu;

   wire [(PRECISION/2)-1:0] aubl, albu;
   
   assign albl_ll=A[(1*(PRECISION/4))-1:(0*(PRECISION/4))]*
		  B[(1*(PRECISION/4))-1:(0*(PRECISION/4))];
   assign albl_ul=A[(2*(PRECISION/4))-1:(1*(PRECISION/4))]*
		  B[(1*(PRECISION/4))-1:(0*(PRECISION/4))];
   assign albl_lu=A[(1*(PRECISION/4))-1:(0*(PRECISION/4))]*
		  B[(2*(PRECISION/4))-1:(1*(PRECISION/4))];
   assign albl_uu=A[(2*(PRECISION/4))-1:(1*(PRECISION/4))]*
		  B[(2*(PRECISION/4))-1:(1*(PRECISION/4))];

   assign aubl=A[PRECISION-1:(PRECISION/2)]*B[(PRECISION/2)-1:0];
   assign albu=A[(PRECISION/2)-1:0]*B[PRECISION-1:(PRECISION/2)];

   // latched versions will be deadcode eliminated if not PIPELINED   
   reg [(PRECISION/2)-1:0]  albl_ll_ff, albl_ul_ff, albl_lu_ff, albl_uu_ff;

   reg [(PRECISION/2)-1:0] aubl_ff, albu_ff;

   always @(posedge CLK) begin
      albl_ll_ff<=albl_ll;
      albl_ul_ff<=albl_ul;
      albl_lu_ff<=albl_lu;
      albl_uu_ff<=albl_uu;

      aubl_ff<=aubl;
      albu_ff<=albu;
   end
   
   // one of the two will be deadcode eliminated during synthesis
   wire [PRECISION-1:0]   O_comb, O_ff;
   assign O_comb=albl_ll+
		 (albl_ul<<(PRECISION/4))+
		 (albl_lu<<(PRECISION/4))+
		 (albl_uu<<(PRECISION/2))+
		 (aubl<<(PRECISION/2))+
		 (albu<<(PRECISION/2));
   assign O_ff=albl_ll_ff+
	       (albl_ul_ff<<(PRECISION/4))+
	       (albl_lu_ff<<(PRECISION/4))+
	       (albl_uu_ff<<(PRECISION/2))+
	       (aubl_ff<<(PRECISION/2))+
	       (albu_ff<<(PRECISION/2));

   assign O=PIPELINED?O_ff:O_comb;
   
endmodule // multf

/* 
 * half-length multiply using carry-save for 3->2 reduction
 *
 * combiantional, ~3.0ns A/B -> O
 *
 * pipelined (at summand5)
 * 
 * less than 2ns A/B -> latch
 * less than 2ns CLK -> O
 * 
 */


module multcsa(A, B, O, CLK);
   parameter PIPELINED=1, PRECISION=32; 
   
   input [PRECISION-1:0] A, B;
   output [PRECISION-1:0] O;
   input CLK;

   reg [PRECISION-1:0] 	 O;
   
   function [PRECISION-1:0] carry;
      input [PRECISION-1:0] a, b, c;
      begin
	 carry=((a&b)|(b&c)|(a&c));
      end
   endfunction 

   function [PRECISION-1:0] sum;
      input [PRECISION-1:0] a, b, c;
      begin
	 sum=(a^b^c);
      end
   endfunction 

   reg [PRECISION-1:0] summand0[0:PRECISION-1];
   reg [PRECISION-1:0] summand1[0:21];
   reg [PRECISION-1:0] summand2[0:14];
   reg [PRECISION-1:0] summand3[0:9];
   reg [PRECISION-1:0] summand4[0:6];
   reg [PRECISION-1:0] summand5_comb[0:4];
   reg [PRECISION-1:0] summand5_ff[0:4];
   wire [PRECISION-1:0] summand5[0:4];
   reg [PRECISION-1:0] summand6[0:3];
   reg [PRECISION-1:0] summand7[0:2];
   reg [PRECISION-1:0] summand8[0:1];

   integer    i;
   
   always@(*) begin
      for(i=0;i<32;i++) begin
	 summand0[i]=B[i]?(A<<i):0;
      end
   end

   always@(*) begin
      for(i=0;i<10;i++) begin  // 30+2 -> 20+2
	 summand1[2*i]=sum(summand0[3*i],summand0[3*i+1],summand0[3*i+2]);
	 summand1[2*i+1]={carry(summand0[3*i],summand0[3*i+1],summand0[3*i+2]),1'b0};
      end
      summand1[20]=summand0[30];
      summand1[21]=summand0[PRECISION-1];
   end 
   
   always@(*) begin
      for(i=0;i<7;i++) begin // 21+1 -> 14+1
	 summand2[2*i]=sum(summand1[3*i],summand1[3*i+1],summand1[3*i+2]);
	 summand2[2*i+1]={carry(summand1[3*i],summand1[3*i+1],summand1[3*i+2]),1'b0};
      end
      summand2[14]=summand1[21];
   end 
   
   always@(*) begin
      for(i=0;i<5;i++) begin  // 15 -> 10
	 summand3[2*i]=sum(summand2[3*i],summand2[3*i+1],summand2[3*i+2]);
	 summand3[2*i+1]={carry(summand2[3*i],summand2[3*i+1],summand2[3*i+2]),1'b0};
      end
   end 
   
   always@(*) begin
      for(i=0;i<3;i++) begin // 9+1 -> 6+1
	 summand4[2*i]=sum(summand3[3*i],summand3[3*i+1],summand3[3*i+2]);
	 summand4[2*i+1]={carry(summand3[3*i],summand3[3*i+1],summand3[3*i+2]),1'b0};
      end
      summand4[6]=summand3[9];
   end 
   
   always@(*) begin
      for(i=0;i<2;i++) begin // 6+1 -> 4+1
	 summand5_comb[2*i]=sum(summand4[3*i],summand4[3*i+1],summand4[3*i+2]);
	 summand5_comb[2*i+1]={carry(summand4[3*i],summand4[3*i+1],summand4[3*i+2]),1'b0};
      end
      summand5_comb[4]=summand4[6];
   end 

   always@(posedge CLK) begin
      summand5_ff<=summand5_comb;
   end 

   assign summand5=PIPELINED?summand5_ff:summand5_comb;
   
   always@(*) begin
      for(i=0;i<1;i++) begin // 3+2 -> 2+2
	 summand6[2*i]=sum(summand5[3*i],summand5[3*i+1],summand5[3*i+2]);
	 summand6[2*i+1]={carry(summand5[3*i],summand5[3*i+1],summand5[3*i+2]),1'b0};
      end
      summand6[2]=summand5[3];
      summand6[3]=summand5[4];
   end 
   
   always@(*) begin
      for(i=0;i<1;i++) begin // 3+1 -> 2+1
	 summand7[2*i]=sum(summand6[3*i],summand6[3*i+1],summand6[3*i+2]);
	 summand7[2*i+1]={carry(summand6[3*i],summand6[3*i+1],summand6[3*i+2]),1'b0};
      end
      summand7[2]=summand6[3];
   end 
   
   always@(*) begin
      for(i=0;i<1;i++) begin // 3 -> 2
	 summand8[2*i]=sum(summand7[3*i],summand7[3*i+1],summand7[3*i+2]);
	 summand8[2*i+1]={carry(summand7[3*i],summand7[3*i+1],summand7[3*i+2]),1'b0};
      end
   end

   always@(*) begin
      O=summand8[0]+summand8[1];
   end 
   
endmodule


