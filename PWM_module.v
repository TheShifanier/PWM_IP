	/*TODO^
	+ 1. increment signal with RC: done on OS1 and OS2
	+ 2. decrement signal with RC: done on OS1 and OS2 
	+ 3. increment signal w/o RC: done on OS1 and OS2
	+ 4. decrement signal w/o RC: done on OS1 and OS2
	+ 5. Prescaler: done, not checked all possible states 
	+ 6. Rebuilt logic with CHCRxx reg 
	+ 7. Rebuilt main logic with CNTR
	+ 8. MORE CHANNELS( СH1-8 )

	*/
	`ifndef _simple_vh_
	`define _simple_vh_
	`timescale 1ns/1ns
	`define DATAWIDTH 32
	`define ADDR 16
	`define PSELx_size 1
	`define IC_bus_size 8
	`define RC_size		7 	//SIZE OF RC bits in decimal
	///////////////////////////////////APB defines///////////////////////////////
	`define IDLE     2'b00
	`define W_ENABLE  2'b01
	`define R_ENABLE  2'b10
	`define PSELx_value 1'b1
	//////////////////////////////////REGISTERS////////////////////////////////
		//PSG//
	`define PSG_val		5	//PSG[5:0] value
	`define PSG_EN 		6	//PSG[6]
	`define PSG_R_EN	7	//PSG[7]
		//CNTR//
	`define RC      	6   //CNTR[6:0] 	+
	`define ARR_R_EN   	7   //CNTR[7]		+
	`define CNT_R_EN 	8	//CNTR[8]		+
	`define CDIR    	9   //CNTR[9]    	+
	`define CCR_R_EN 	10  //CNTR[10]      +  save or only CCRE_x??
	`define CNT_EN  	11  //CNTR[11]    	+  
	`define OS_EN   	12  //CNTR[12]		+
	`define RC_SR     	14  //CNTR[14:13] 	+
	`define TIM_SR    	15  //CNTR[15]     	+ -
		
		//CHCR14 - Channels 1-4 Control Register//
	`define CCRE_1		0 	//CCR register reload CH1
	`define CCRE_2		1 	//CCR register reload CH2
	`define CCRE_3		2 	//CCR register reload CH3
	`define CCRE_4		3 	//CCR register reload CH4
	`define OSE_1      	4   //Output signal enable CH1
	`define OSE_2      	5   //Output signal enable CH2
	`define OSE_3      	6   //Output signal enable CH3
	`define OSE_4      	7   //Output signal enable CH4
	`define CHINTR_1   	8   //Channel 1 generate interruption
	`define CHINTR_2   	9   //Channel 2 generate interruption
	`define CHINTR_3   	10   //Channel 3 generate interruption
	`define CHINTR_4   	11   //Channel 4 generate interruption 
	`define CHSR_1 		12	//Channel 1 state register
	`define CHSR_2 		13	//Channel 2 state register
	`define CHSR_3 		14	//Channel 3 state register
	`define CHSR_4 		15	//Channel 4 state register

		//CHCR58 - Channels 5-8 Control Register//
	`define CCRE_5		0 	//CCR register reload CH1
	`define CCRE_6		1 	//CCR register reload CH2
	`define CCRE_7		2 	//CCR register reload CH3
	`define CCRE_8		3 	//CCR register reload CH4
	`define OSE_5      	4   //Output signal enable CH1
	`define OSE_6      	5   //Output signal enable CH2
	`define OSE_7      	6   //Output signal enable CH3
	`define OSE_8      	7   //Output signal enable CH4
	`define CHINTR_5   	8   //Channel 1 generate interruption 
	`define CHINTR_6   	9   //Channel 2 generate interruption
	`define CHINTR_7   	10   //Channel 3 generate interruption
	`define CHINTR_8   	11   //Channel 4 generate interruption
	`define CHSR_5 		12	//Channel 1 state register
	`define CHSR_6 		13	//Channel 2 state register
	`define CHSR_7 		14	//Channel 3 state register
	`define CHSR_8 		15	//Channel 4 state register

	module PWM_module(i_clk,i_clr,o_PWM, o_INTR,i_CH_IC, 
					PADDR, PWRITE, PSEL, PENABLE, PWDATA, PRDATA, PREADY, PSLVERR);
		
	input   	[`DATAWIDTH-1:0]  	PWDATA;
	input   	[`ADDR-1:0]   		PADDR;
	input		[`IC_bus_size-1:0]  i_CH_IC;
	input   	[`PSELx_size-1:0]  	PSEL;
	input   						i_clk, i_clr, PWRITE, PENABLE;

	output wire [`DATAWIDTH-1:0]   	PRDATA;
	output wire [7:0]   			o_PWM, o_INTR;
	output wire 					PREADY, PSLVERR;
	reg         [`DATAWIDTH-1:0]   	IC_CH1_SHIFT, IC_CH1_PERIOD, IC_CH2_SHIFT, IC_CH2_PERIOD, IC_CH3_SHIFT, IC_CH3_PERIOD, IC_CH4_SHIFT, IC_CH4_PERIOD;
	reg         [`DATAWIDTH-1:0]   	IC_CH5_SHIFT, IC_CH5_PERIOD, IC_CH6_SHIFT, IC_CH6_PERIOD, IC_CH7_SHIFT, IC_CH7_PERIOD, IC_CH8_SHIFT, IC_CH8_PERIOD;
	reg         [`DATAWIDTH-1:0]   	CNT, CNTR, ARR, CHCR14, CHCR58, PRDATA_reg, CCR11, CCR12, CCR21, CCR22;
	reg         [`DATAWIDTH-1:0]   	CCR31, CCR32, CCR41, CCR42, CCR51, CCR52, CCR61, CCR62, CCR71, CCR72, CCR81, CCR82;
	reg			[7:0]				ERR_HNDLR, state_of_output, PSG, o_INTR_reg, cnt_for_PSG;
	reg			[1:0]				State, CH1_IC_reg, CH2_IC_reg, CH3_IC_reg, CH4_IC_reg, CH5_IC_reg;
	reg			[1:0]				CH6_IC_reg, CH7_IC_reg, CH8_IC_reg;
	reg								PREADY_reg, CHCR14_reg, CHCR58_reg;
	wire  prev_CNTR;
	wire [7:0]  fall_IC_CH, rise_IC_CH;
	/////////////////MAIN SECTION/////////

	always @(posedge i_clk, negedge i_clr)
		if (i_clr == 0)
		begin
			CNTR[`DATAWIDTH-1:0] <= 32'b0;
			ARR[`DATAWIDTH-1:0] <= 32'b0;
			PSG <= 8'b0;
			CHCR58_reg <= 1'b0;
			{CHCR14,CHCR58} <= `DATAWIDTH'b0;
			State <= `IDLE;
			cnt_for_PSG <= 8'b1;
			PRDATA_reg <= `DATAWIDTH'b0;
			PREADY_reg <= 1'b0;
			o_INTR_reg <= 8'b0;
			CHCR14_reg <= 1'b0;
			{CCR11, CCR21, CCR31, CCR41, CCR51, CCR61, CCR71, CCR81} <= `DATAWIDTH'b0;
			{CCR12, CCR22, CCR32, CCR42, CCR52, CCR62, CCR72, CCR82} <= `DATAWIDTH'b0;
			{CH1_IC_reg, CH2_IC_reg, CH3_IC_reg, CH4_IC_reg, CH5_IC_reg, CH6_IC_reg, CH7_IC_reg, CH8_IC_reg} <= 2'b0;
			{IC_CH1_SHIFT, IC_CH1_PERIOD, IC_CH2_SHIFT, IC_CH2_PERIOD, IC_CH3_SHIFT, IC_CH3_PERIOD, IC_CH4_SHIFT, IC_CH4_PERIOD} <= `DATAWIDTH'b0;
			{IC_CH5_SHIFT, IC_CH5_PERIOD, IC_CH6_SHIFT, IC_CH6_PERIOD, IC_CH7_SHIFT, IC_CH7_PERIOD, IC_CH8_SHIFT, IC_CH8_PERIOD} <= `DATAWIDTH'b0;
		end
		else if (i_clr == 1'b1)
		begin
			if ((PSG[`PSG_EN] == 1'b1) && (PSG[`PSG_R_EN] == 1'b1))
			begin
				if (PSG[`PSG_val:0] == 1)
				begin
					cnt_for_PSG <= 8'b0;
					PSG[`PSG_R_EN] <= 0;
				end
				else if ((PSG[`PSG_val:0] != 1) && (PSG[`PSG_val:0] == cnt_for_PSG + 1'b1))
				begin
					cnt_for_PSG <= 8'b1;
					PSG[`PSG_R_EN] <= 0;
				end
				else  cnt_for_PSG <= cnt_for_PSG + 1'b1;
			end
			else if ((PSG[`PSG_EN] == 1) && (PSG[`PSG_R_EN] == 0) && (PSG[`PSG_val:0] != 1'b1)) 
			begin
				PSG[`PSG_R_EN] <= 1;
				//cnt_for_PSG <= cnt_for_PSG + 1;
			end
			else if ((PSG[`PSG_EN] == 1) && (PSG[`PSG_R_EN] == 0) && (PSG[`PSG_val:0] == 0)) 
			begin
				PSG[`PSG_val:0] <= 1; // PSG[`PSG_val:0] == 0 reserved, not used by MPS
				PSG[`PSG_R_EN] <= 0;
			end
			////////////APB section begin////////////
			case (State)
				`IDLE : begin
					PRDATA_reg <= `DATAWIDTH'b0;
					if ((PSEL == `PSELx_value) && (!PENABLE)) 
					begin
						if (PWRITE) begin
							State <= `W_ENABLE;
							PREADY_reg <= 1;
						end
						else
						begin
							State <= `R_ENABLE;
							PREADY_reg <= 1;
						end
					end
				end
				`W_ENABLE : begin
					if ((PSEL == `PSELx_value) && (PWRITE) && (PENABLE))
					begin
						case(PADDR)
						//16'h0000: CNT <= PWDATA;
						16'h0004:
						begin
							CNTR <= PWDATA; //write all data to CNTR register
							$strobe("TEST TIM_SR %b, RC_SR %b, OS_EN %b, CNT_EN %b, CCR_R_EN %b, CDIR %b, CNT_R_EN%b, ARR_R_EN %b, RC %b", CNTR[`TIM_SR], CNTR[`RC_SR:`RC_SR - 1], CNTR[`OS_EN], CNTR[`CNT_EN], CNTR[`CCR_R_EN], CNTR[`CDIR], CNTR[`CNT_R_EN], CNTR[`ARR_R_EN], CNTR[`RC:0]);			
						end
						16'h0008: CHCR14 <= PWDATA;
						16'h000C: CHCR58 <= PWDATA;
						16'h0010: PSG <= PWDATA; 
						16'h0014:
							if (CNTR[`ARR_R_EN] != 0) 
							begin 
								ARR <= PWDATA;
								CNTR[`CNT_R_EN] <= 1;
							end
						16'h0018: if ((CNTR[`CCR_R_EN] != 0) && (CHCR14[`CCRE_1] != 0)) CCR11 <= PWDATA;
						16'h001C: if ((CNTR[`CCR_R_EN] != 0) && (CHCR14[`CCRE_2] != 0)) CCR21 <= PWDATA;
						16'h0020: if ((CNTR[`CCR_R_EN] != 0) && (CHCR14[`CCRE_3] != 0)) CCR31 <= PWDATA;
						16'h0024: if ((CNTR[`CCR_R_EN] != 0) && (CHCR14[`CCRE_4] != 0)) CCR41 <= PWDATA;
						16'h0028: if ((CNTR[`CCR_R_EN] != 0) && (CHCR58[`CCRE_5] != 0)) CCR51 <= PWDATA;
						16'h002C: if ((CNTR[`CCR_R_EN] != 0) && (CHCR58[`CCRE_6] != 0)) CCR61 <= PWDATA;
						16'h0030: if ((CNTR[`CCR_R_EN] != 0) && (CHCR58[`CCRE_7] != 0)) CCR71 <= PWDATA;
						16'h0034: if ((CNTR[`CCR_R_EN] != 0) && (CHCR58[`CCRE_8] != 0)) CCR81 <= PWDATA;

						16'h0038: if ((CNTR[`CCR_R_EN] != 0) && (CHCR14[`CCRE_1] != 0)) CCR12 <= PWDATA;
						16'h003C: if ((CNTR[`CCR_R_EN] != 0) && (CHCR14[`CCRE_2] != 0)) CCR22 <= PWDATA;
						16'h0040: if ((CNTR[`CCR_R_EN] != 0) && (CHCR14[`CCRE_3] != 0)) CCR32 <= PWDATA;
						16'h0044: if ((CNTR[`CCR_R_EN] != 0) && (CHCR14[`CCRE_4] != 0)) CCR42 <= PWDATA;
						16'h0048: if ((CNTR[`CCR_R_EN] != 0) && (CHCR58[`CCRE_5] != 0)) CCR52 <= PWDATA;
						16'h004C: if ((CNTR[`CCR_R_EN] != 0) && (CHCR58[`CCRE_6] != 0)) CCR62 <= PWDATA;
						16'h0050: if ((CNTR[`CCR_R_EN] != 0) && (CHCR58[`CCRE_7] != 0)) CCR72 <= PWDATA;
						16'h0054: if ((CNTR[`CCR_R_EN] != 0) && (CHCR58[`CCRE_8] != 0)) CCR82 <= PWDATA;
						default:
						begin
							$display("ERROR: invalid input data value");
						end
						endcase
					end
					State <= `IDLE;
				end
				`R_ENABLE : begin
					if ((PSEL  == `PSELx_value) && (!PWRITE) && (PENABLE)) begin
					PREADY_reg <= 1;
					case (PADDR)
					16'h0000: PRDATA_reg <= CNT;
					16'h0004: PRDATA_reg <= CNTR;
					16'h0008: PRDATA_reg <= CHCR14;
					16'h000C: PRDATA_reg <= CHCR58;
					16'h0010: PRDATA_reg[7:0] <= PSG;
					16'h0014: PRDATA_reg <= ARR;
					16'h0018: PRDATA_reg <= CCR11;
					16'h001C: PRDATA_reg <= CCR21;
					16'h0020: PRDATA_reg <= CCR31;
					16'h0024: PRDATA_reg <= CCR41;
					16'h0028: PRDATA_reg <= CCR51;
					16'h002C: PRDATA_reg <= CCR61;
					16'h0030: PRDATA_reg <= CCR71;
					16'h0034: PRDATA_reg <= CCR81;

					16'h0038: PRDATA_reg <= CCR12;
					16'h003C: PRDATA_reg <= CCR22;
					16'h0040: PRDATA_reg <= CCR32;
					16'h0044: PRDATA_reg <= CCR42;
					16'h0048: PRDATA_reg <= CCR52;
					16'h004C: PRDATA_reg <= CCR62;
					16'h0050: PRDATA_reg <= CCR72;
					16'h0054: PRDATA_reg <= CCR82;
					endcase
					State <= `IDLE;
					end
				end
				default: State <= `IDLE;
			endcase
			////////////APB section end////////////
			///////////////////////////////////////
			////CNT and RC control logic (PWM mode) begin/////
			if (CNTR[`CDIR] == 1'b1)
			begin
				if (CNT + `DATAWIDTH'b1>= ARR ) CNTR[`CNT_R_EN] <= 1;
			end
			if (CNTR[`CDIR] == 1'b0)
			begin
				if (CNT == 1'b1) CNTR[`CNT_R_EN] <= 1;
			end
			if ((CNTR != 0) && (CNTR[`CNT_EN])&& (PSG[`PSG_R_EN] == 0))
			begin	
				///If RC != 0///
				if (ERR_HNDLR != 0)
				begin
					CNTR[`CNT_R_EN] <= 1;
					//ERR_HNDLR <= 0;
					$display("Error due to counter overflow or state undefined");
				end
				else if (CNTR[`CNT_R_EN] == 1) CNTR[`CNT_R_EN] <= 0;		
				else if ( (CNTR[`RC_SR:`RC_SR - 1] == 2'b10))
				begin			
					if (CNTR[`CDIR] == 1'b1)
					begin
							if (CNT + `DATAWIDTH'b1>= ARR)
							begin
								if (CNTR[`RC:0] == `RC_size'b0)	CNTR[`RC_SR:`RC_SR-1] <= 2'b00;
								else if  (CNTR[`RC:0] != `RC_size'b0) CNTR[`RC:0] <= CNTR[`RC:0] - `RC'b1;						
							end
					end
					else if (CNTR[`CDIR] == 1'b0)
					begin   
							if (CNT <= 1'b1) 
							begin
								if (CNTR[`RC:0] == `RC_size'b0)	CNTR[`RC_SR:`RC_SR-1] <= 2'b00;
								else if  (CNTR[`RC:0] != `RC_size'b0) CNTR[`RC:0] <= CNTR[`RC:0] - `RC'b1;  
							end
					end 
				end
				///If RC == 0//
				////CNT and RC control logic (PWM mode) end/////
				/////////////////////////////////////
				////CNT and RC control logic (IC mode) begin /////
			else if (PSG[`PSG_R_EN] == 0)
			begin
				if (ERR_HNDLR != 0)
				begin
					ERR_HNDLR <= 8'b0;
				end	
				if (CHCR14[`CHSR_4:`CHSR_1] != 0)
				begin
					//if (CNTR[`CDIR] == 1'b1)
					CHCR14_reg <= CHCR14[`CHSR_1];
					if (CHCR14[`CHSR_1] == 1)
					begin
						CH1_IC_reg <= {CH1_IC_reg[0], i_CH_IC[0]};
						if (prev_CNTR == 1) {CCR11, CCR12} <= 0;
						else if ( (CNT == ARR) ) {CCR11, CCR12} <= 0;
						//if (CNTR[`RC_SR:`RC_SR-1] != 2'b00) CNTR[`RC_SR:`RC_SR-1] <= 2'b00; //CHECK A POSSIBLE ERROR DUE TO INVALID VALUE IN CNTR_reg
						if ((CH1_IC_reg[0] == 1) && (rise_IC_CH[0] == 1))
						begin
							IC_CH1_SHIFT <= CNT - CCR11;
							$strobe("sSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR11, IC_CH1_SHIFT, $time);
							$display("dSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR11, IC_CH1_PERIOD, $time);
							CCR11 <= CNT;
							o_INTR_reg[0] <= 1'b1;
						end
						//if ((i_CH_IC[0] == 1) && (rise_IC_CH[0] == 0)) 
						if ((CH1_IC_reg[0] == 0) && (fall_IC_CH[0] == 1))
						begin
							IC_CH1_PERIOD <= CNT - CCR12;
							CCR12 <= CNT;
							CCR11 <= CNT;
							o_INTR_reg[0] <= 1'b0;
							$strobe("BEGIN2 CNT %h, Signal captured at: %h, Period = %h, Signal duration = %h, time %d", CNT, CCR12, CCR11, IC_CH1_PERIOD, $time);
						end
					end
					if (CHCR14[`CHSR_2] == 1)
					begin
						CH2_IC_reg <= {CH2_IC_reg[0], i_CH_IC[1]};
						if (prev_CNTR == 1) {CCR21, CCR22} <= 0;
						else if ( (CNT == ARR) ) {CCR21, CCR22} <= 0;
						//if (CNTR[`RC_SR:`RC_SR-1] != 2'b00) CNTR[`RC_SR:`RC_SR-1] <= 2'b00; //CHECK A POSSIBLE ERROR DUE TO INVALID VALUE IN CNTR_reg
						if ((CH2_IC_reg[0] == 1) && (rise_IC_CH[1] == 1))
						begin
							IC_CH2_SHIFT <= CNT - CCR21;
							$strobe("sSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR21, IC_CH2_SHIFT, $time);
							$display("dSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR21, IC_CH2_PERIOD, $time);
							CCR21 <= CNT;
							o_INTR_reg[1] <= 1'b1;
						end
						//if ((i_CH_IC[1] == 1) && (rise_IC_CH[1] == 0)) o_INTR_reg[1] <= 1'b0;
						if ((CH2_IC_reg[0] == 0)&&(fall_IC_CH[1] == 1))
						begin
							IC_CH2_PERIOD <= CNT - CCR22;
							CCR22 <= CNT;
							CCR21 <= CNT;
							o_INTR_reg[1] <= 1'b0;
							$strobe("BEGIN2 CNT %h, Signal captured at: %h, Period = %h, Signal duration = %h, time %d", CNT, CCR22, CCR21, IC_CH2_PERIOD, $time);
						end
					end
					if (CHCR14[`CHSR_3] == 1)
					begin
						CH3_IC_reg <= {CH3_IC_reg[0], i_CH_IC[2]};
						if (prev_CNTR == 1) {CCR31, CCR32} <= 0;
						else if ( (CNT == ARR) ) {CCR31, CCR32} <= 0;
						//if (CNTR[`RC_SR:`RC_SR-1] != 2'b00) CNTR[`RC_SR:`RC_SR-1] <= 2'b00; //CHECK A POSSIBLE ERROR DUE TO INVALID VALUE IN CNTR_reg
						if ((CH3_IC_reg[0] == 1) && (rise_IC_CH[2] == 1))
						begin
							IC_CH3_SHIFT <= CNT - CCR31;
							$strobe("sSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR31, IC_CH3_SHIFT, $time);
							$display("dSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR31, IC_CH3_PERIOD, $time);
							CCR31 <= CNT;
							o_INTR_reg[2] <= 1'b1;
						end
						//if ((i_CH_IC[2] == 1) && (rise_IC_CH[2] == 0)) o_INTR_reg[2] <= 1'b0;
						if ((CH3_IC_reg[0] == 0)&&(fall_IC_CH[2] == 1))
						begin
							IC_CH3_PERIOD <= CNT - CCR32;
							CCR32 <= CNT;
							CCR31 <= CNT;
							o_INTR_reg[2] <= 1'b0;
							$strobe("BEGIN2 CNT %h, Signal captured at: %h, Period = %h, Signal duration = %h, time %d", CNT, CCR32, CCR31,IC_CH3_PERIOD, $time);
						end
					end
					if (CHCR14[`CHSR_4] == 1)
					begin
						CH4_IC_reg <= {CH4_IC_reg[0], i_CH_IC[3]};
						if (prev_CNTR == 1) {CCR41, CCR42} <= 0;
						else if ( (CNT == ARR) ) {CCR41, CCR42} <= 0;
						//if (CNTR[`RC_SR:`RC_SR-1] != 2'b00) CNTR[`RC_SR:`RC_SR-1] <= 2'b00; //CHECK A POSSIBLE ERROR DUE TO INVALID VALUE IN CNTR_reg
						if ((CH4_IC_reg[0] == 1) && (rise_IC_CH[3] == 1))
						begin
							IC_CH4_SHIFT <= CNT - CCR41;
							$strobe("sSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR41, IC_CH4_SHIFT, $time);
							$display("dSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR41, IC_CH4_PERIOD, $time);
							CCR41 <= CNT;
							o_INTR_reg[3] <= 1'b1;
						end
						//if ((i_CH_IC[3] == 1) && (rise_IC_CH[3] == 0)) o_INTR_reg[3] <= 1'b0;
						if ((CH4_IC_reg[0] == 0) && (fall_IC_CH[3] == 1))
						begin
							IC_CH4_PERIOD <= CNT - CCR42;
							CCR42 <= CNT;
							CCR41 <= CNT;
							o_INTR_reg[3] <= 1'b0;
							$strobe("BEGIN2 CNT %h, Signal captured at: %h, Period = %h, Signal duration = %h, time %d", CNT, CCR42, CCR41, IC_CH4_PERIOD, $time);
						end
					end
				end
				if (CHCR58[`CHSR_8:`CHSR_5] != 0)
				begin
					CHCR58_reg <= CHCR14[`CHSR_1];
					if (CHCR58[`CHSR_5] == 1)
					begin
						CH5_IC_reg <= {CH5_IC_reg[0], i_CH_IC[4]};
						if (prev_CNTR == 1) {CCR51, CCR52} <= 0;
						else if ( (CNT == ARR) ) {CCR51, CCR52} <= 0;
						//if (CNTR[`RC_SR:`RC_SR-1] != 2'b00) CNTR[`RC_SR:`RC_SR-1] <= 2'b00; //CHECK A POSSIBLE ERROR DUE TO INVALID VALUE IN CNTR_reg
						if ((CH5_IC_reg[0] == 1) && (rise_IC_CH[4] == 1))
						begin
							IC_CH5_SHIFT <= CNT - CCR51;
							$strobe("sSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR51, IC_CH5_SHIFT, $time);
							$display("dSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR51, IC_CH5_PERIOD, $time);
							CCR51 <= CNT;
							o_INTR_reg[4] <= 1'b1;
						end
						//if ((i_CH_IC[4] == 1) && (rise_IC_CH[4] == 0)) o_INTR_reg[4] <= 1'b0;
						if ((CH5_IC_reg[0] == 0) && (fall_IC_CH[4] == 1))
						begin
							IC_CH5_PERIOD <= CNT - CCR52;
							CCR52 <= CNT;
							CCR51 <= CNT;
							o_INTR_reg[4] <= 1'b0;
							$strobe("BEGIN2 CNT %h, Signal captured at: %h, Period = %h, Signal duration = %h, time %d", CNT, CCR52, CCR51, IC_CH5_PERIOD, $time);
						end
					end
					if (CHCR58[`CHSR_6] == 1)
					begin
						CH6_IC_reg <= {CH6_IC_reg[0], i_CH_IC[5]};
						if (prev_CNTR == 1) {CCR61, CCR62} <= 0;
						else if ( (CNT == ARR) ) {CCR61, CCR62} <= 0;
						//if (CNTR[`RC_SR:`RC_SR-1] != 2'b00) CNTR[`RC_SR:`RC_SR-1] <= 2'b00; //CHECK A POSSIBLE ERROR DUE TO INVALID VALUE IN CNTR_reg
						if ((CH6_IC_reg[0] == 1) && (rise_IC_CH[5] == 1))
						begin
							IC_CH6_SHIFT <= CNT - CCR61;
							$strobe("sSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR61, IC_CH6_SHIFT, $time);
							$display("dSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR61, IC_CH6_PERIOD, $time);
							CCR61 <= CNT;
							o_INTR_reg[5] <= 1'b1;
						end
						//if ((i_CH_IC[5] == 1) && (rise_IC_CH[5] == 0)) o_INTR_reg[5] <= 1'b0;
						if ((CH6_IC_reg[0] == 0)&&(fall_IC_CH[5] == 1))
						begin
							IC_CH6_PERIOD <= CNT - CCR62;
							CCR62 <= CNT;
							CCR61 <= CNT;
							o_INTR_reg[5] <= 1'b0;
							$strobe("BEGIN2 CNT %h, Signal captured at: %h, Period = %h, Signal duration = %h, time %d", CNT, CCR62, CCR61, IC_CH6_PERIOD, $time);
						end
					end
					if (CHCR58[`CHSR_7] == 1)
					begin
						CH7_IC_reg <= {CH7_IC_reg[0], i_CH_IC[6]};
						if (prev_CNTR == 1) {CCR71, CCR72} <= 0;
						else if ( (CNT == ARR) ) {CCR71, CCR72} <= 0;
						//if (CNTR[`RC_SR:`RC_SR-1] != 2'b00) CNTR[`RC_SR:`RC_SR-1] <= 2'b00; //CHECK A POSSIBLE ERROR DUE TO INVALID VALUE IN CNTR_reg
						if ((CH7_IC_reg[0] == 1) && (rise_IC_CH[6] == 1))
						begin
							IC_CH7_SHIFT <= CNT - CCR71;
							$strobe("sSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR71, IC_CH7_SHIFT, $time);
							$display("dSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR71, IC_CH7_PERIOD, $time);
							CCR71 <= CNT;
							o_INTR_reg[6] <= 1'b1;
						end
						//if ((i_CH_IC[6] == 1) && (rise_IC_CH[6] == 0)) o_INTR_reg[6] <= 1'b0;
						if ((CH7_IC_reg[0] == 0)&&(fall_IC_CH[6] == 1))
						begin
							IC_CH7_PERIOD <= CNT - CCR72;
							CCR72 <= CNT;
							CCR71 <= CNT;
							o_INTR_reg[6] <= 1'b0;
							$strobe("BEGIN2 CNT %h, Signal captured at: %h, Period = %h, Signal duration = %h, time %d", CNT, CCR72, CCR71,IC_CH7_PERIOD, $time);
						end
					end
					if (CHCR58[`CHSR_8] == 1)
					begin
						CH8_IC_reg <= {CH8_IC_reg[0], i_CH_IC[7]};
						if (prev_CNTR == 1) {CCR81, CCR82} <= 0;
						else if ( (CNT == ARR) ) {CCR81, CCR82} <= 0;
						//if (CNTR[`RC_SR:`RC_SR-1] != 2'b00) CNTR[`RC_SR:`RC_SR-1] <= 2'b00; //CHECK A POSSIBLE ERROR DUE TO INVALID VALUE IN CNTR_reg
						if ((CH8_IC_reg[0] == 1) && (rise_IC_CH[7] == 1))
						begin
							IC_CH8_SHIFT <= CNT - CCR81;
							$strobe("sSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR81, IC_CH8_SHIFT, $time);
							$display("dSHIFT CNT %h, Signal captured at: %h, Shift = %h, time %d", CNT, CCR81, IC_CH8_PERIOD, $time);
							CCR81 <= CNT;
							o_INTR_reg[7] <= 1'b1;
						end
						//if ((i_CH_IC[7] == 1) && (rise_IC_CH[7] == 0)) o_INTR_reg[7] <= 1'b0;
						if ((CH8_IC_reg[0] == 0) && (fall_IC_CH[7] == 1))
						begin
							IC_CH8_PERIOD <= CNT - CCR82;
							CCR82 <= CNT;
							CCR81 <= CNT;
							o_INTR_reg[7] <= 1'b0;
							$strobe("BEGIN2 CNT %h, Signal captured at: %h, Period = %h, Signal duration = %h, time %d", CNT, CCR82, CCR81, IC_CH8_PERIOD, $time);
						end
					end
				end
			end
			end
		end

	always @(posedge i_clk, negedge i_clr)
		if (i_clr == 0)
		begin
			state_of_output <= 8'b0;
			ERR_HNDLR <= 8'b0;
		end
		else if (PSG[`PSG_R_EN] == 0)
		begin
			if (ERR_HNDLR != 0)
			begin
				ERR_HNDLR <= 8'b0;
			end
		//////////////////////////////////////////////////////////////////////////////
		///////////////////////////////// PWM MODE LOGIC///////////////////////////////
		//////////////////////////////////////////////////////////////////////////////
			if ((CNTR != 0) && (CNTR[`CNT_EN]!= 0) && (ARR != 0)) 
			begin
				if (CHCR14 != 0)
				begin
					if (CHCR14[`CHSR_1] == 0)
					begin
						if (CHCR14[`OSE_1] == 1'b1)
						begin
							//$display("OSE_1 = 1");
							if ( (CNTR[`RC_SR:`RC_SR-1] == 2'b10) && (CCR11 != 0))
							begin
							// increment signal - set	
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[0] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[0] == 1) && (CNT  < CCR11)) || ((state_of_output[0] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[0] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[0] <= 1'b1;
									$display("Task ERROR1 state_of_output[0] %b", state_of_output[0]);
									state_of_output[0] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[0] %b", CNT, state_of_output[0]);
									end
									else if (state_of_output[0] == 1)
									begin
										if (CNT == ARR)
										begin 
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[0] <= 1'b0;	////	!!!!!!!!!!!!!!!!!!	////end 
											end
											default:
											begin
												state_of_output[0] <= 1'b0;  ////	!!!!!!!!!!!!!!!!!!	////end
											end
											endcase
										end
									end
									else  if ((state_of_output[0] == 1'b0) && (CNTR[`RC:0] != `RC_size'b0))
									begin
										if ((CNT <= ARR) && (CNT >= CCR11) )
										begin 
											$display("CNT %b,  CCR11 %b", CNT, CCR11);
											state_of_output[0] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin
									$display("DECR CNT %d,  CCR11 %d, state %d", CNT, CCR11, state_of_output[0]);
									if (CNT == ARR) state_of_output[0] <= 1'b1; 
									else if ((CNTR[`CNT_EN]) && ((state_of_output[0] == 0) && (CNT  > CCR11 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[0] <= 1'b1;
									$display("ERROR2 state_of_output[0] %b", state_of_output[0]);
									//state_of_output[0] <= 1'b1;
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[0] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[0] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[0] <= 1'b1;
									$display("ERROR3 state_of_output[0] %b", state_of_output[0]);
									//state_of_output[0] <= 1'b0;
									end 
									else if ( (state_of_output[0] == 0) && (CNTR[`CNT_EN]) )
									begin
										if (CNT == 0)
										begin
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[0] <= 1'b0;
											end
											default:
											begin
												state_of_output[0] <= 1'b1;
											end
											endcase       
										end 
									end
									else  if ( (state_of_output[0] == 1'b1) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT <= ARR) && (CNT <= CCR11 + `DATAWIDTH'b1) )
										begin 
											$display("decrement OS 1 CNT %b,  CCR11 %b", CNT, CCR11);
											state_of_output[0] <= 1'b0;   
										end
									end
								end 
							end
								// W/o RC //
							else if ( (CNTR[`RC:0] == `RC_size'b0) && (CNTR[`RC_SR:`RC_SR-1] == 2'b01) && (CCR11 != 0))
							begin
								//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[0] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[0] == 1) && (CNT  < CCR11)) || ((state_of_output[0] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[0] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[0] <= 1'b1;
									$display("Task ERROR1 state_of_output[0] %b", state_of_output[0]);
									state_of_output[0] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[0] %b", CNT, state_of_output[0]);
									end 
									else if (state_of_output[0] == 1)
									begin
										if ( CNT== ARR )
										begin
											state_of_output[0] <= 1'b0;       
										end
									end
									else  if ( state_of_output[0] == 1'b0 )
									begin
										if ((CNT <= ARR) && (CNT >= CCR11) )
										begin 
											state_of_output[0] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin
									if (CNT == ARR) state_of_output[0] <= 1'b1; 
									else if ((CNTR[`CNT_EN]) && ((state_of_output[0] == 0) && (CNT  > CCR11 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[0] <= 1'b1;
									$display("ERROR2 state_of_output[0] %b", state_of_output[0]);
									//state_of_output[0] <= 1'b1;
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[0] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[0] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[0] <= 1'b1;
									$display("ERROR3 state_of_output[0] %b", state_of_output[0]);
									state_of_output[0] <= 1'b0;
									end      
									else if (state_of_output[0] == 0)
									begin
										if ((CNT == 0) )
										begin
											state_of_output[0] <= 1'b1;       
										end
									end
									else  if (state_of_output[0] == 1)
									begin
										if ((CNT <= ARR) && (CNT <= CCR11 + `DATAWIDTH'b1) )
										begin 
											state_of_output[0] <= 1'b0;   
										end
									end
								end 
							end
						end
					end
					if (CHCR14[`CHSR_2] == 0)
					begin
						if (CHCR14[`OSE_2] == 1'b1)
						begin
							if ((CNTR[`RC_SR:`RC_SR-1] == 2'b10) && (CCR21 != 0))
							begin
							//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[1] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[1] == 1) && (CNT  < CCR21)) || ((state_of_output[1] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[1] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[1] <= 1'b1;
									$display("Task ERROR4 state_of_output[1] %b", state_of_output[1]);
									state_of_output[1] <= 1'b0;
									$display("Task ERROR4 with CNT %d, state_of_output[1] %b", CNT, state_of_output[1]);
									end

									else if (state_of_output[1] == 1)
									begin
										if (CNT == ARR)
										begin 
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[1] <= 1'b0;
											end 
											default:
											begin
												$display("CNT %b,  ARR %b", CNT, ARR);
												state_of_output[1] <= 1'b0;
											end
											endcase
										end
									end
									else  if ((CNTR[`RC:0] != `RC_size'b0) &&(state_of_output[1] == 1'b0))
									begin
										if ((CNT <= ARR) && (CNT >= CCR21) )
										begin 
											$display("CNT %b,  CCR21 %b", CNT, CCR21);
											state_of_output[1] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin
									$display("DECR oS2 CNT %d,  CCR21 %d, state %d", CNT, CCR21, state_of_output[1]);
									if (CNT == ARR) state_of_output[1] <= 1'b1;   
									else if ((CNTR[`CNT_EN]) && ((state_of_output[1] == 0) && (CNT  > CCR21 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[1] <= 1'b1;
									$display("ERROR4 state_of_output[1] %b", state_of_output[1]);
									//state_of_output[1] <= 1'b1;
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[1] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[1] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[1] <= 1'b1;
									$display("ERROR5 state_of_output[1] %b", state_of_output[1]);
									state_of_output[1] <= 1'b0;
									end     
									else if ( (state_of_output[1] == 0) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT  == 0) )
										begin
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[1] <= 1'b0;
											end
											default:
											begin
												state_of_output[1] <= 1'b1;
											end
											endcase       
										end 
									end
									else  if ( (state_of_output[1] == 1'b1) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT <= ARR) && (CNT <= CCR21 + `DATAWIDTH'b1) )
										begin 
											$display("decrement OS 2 CNT %b,  CCR21 %b", CNT, CCR21);
											state_of_output[1] <= 1'b0;   
										end
									end
								end 
							end
							// W/o RC //
							else if ( (CNTR[`RC:0] == `RC_size'b0) && (CNTR[`RC_SR:`RC_SR-1] == 2'b01) && (CCR21 != 0) )
							begin
								//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[1] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[1] == 1) && (CNT  < CCR21)) || ((state_of_output[1] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[1] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[1] <= 1'b1;
									$display("Task ERROR1 state_of_output[1] %b", state_of_output[1]);
									state_of_output[1] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[1] %b", CNT, state_of_output[1]);
									end
									else if ( state_of_output[1] == 1)
									begin
										if ( CNT == ARR )
										begin
											state_of_output[1] <= 1'b0;       
										end
									end
									else  if ( state_of_output[1] == 0 )
									begin
										if ((CNT <= ARR) && (CNT >= CCR21) )
										begin
											state_of_output[1] <= 1'b1;        
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin  
									if (CNT == ARR) state_of_output[1] <= 1'b1;   
									else if ((CNTR[`CNT_EN]) && ((state_of_output[1] == 0) && (CNT  > CCR21 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[1] <= 1'b1;
									$display("ERROR4 state_of_output[1] %b", state_of_output[1]);
									//state_of_output[1] <= 1'b1;
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[1] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[1] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[1] <= 1'b1;
									$display("ERROR5 state_of_output[1] %b", state_of_output[1]);
									state_of_output[1] <= 1'b0;
									end    
									else if (state_of_output[1] == 0)
									begin
										if ((CNT == 0) )
										begin
											state_of_output[1] <= 1'b1;       
										end
									end
									else  if (state_of_output[1] == 1)
									begin
										if ((CNT <= ARR) && (CNT <= CCR21 + `DATAWIDTH'b1) )
										begin 
											state_of_output[1] <= 1'b0;       
										end
									end
								end 
							end
						end
					end
					if (CHCR14[`CHSR_3] == 0)
					begin
						if (CHCR14[`OSE_3] == 1'b1)
						begin
							//$display("OSE_1 = 1");
							if ((CNTR[`RC_SR:`RC_SR-1] == 2'b10) && (CCR31 != 0))
							begin
							//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[2] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[2] == 1) && (CNT  < CCR31)) || ((state_of_output[2] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[2] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[2] <= 1'b1;
									$display("Task ERROR1 state_of_output[2] %b", state_of_output[2]);
									state_of_output[2] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[2] %b", CNT, state_of_output[2]);
									end
									else if (state_of_output[2] == 1)
									begin
										if (CNT == ARR)
										begin 
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[2] <= 1'b0;	////	!!!!!!!!!!!!!!!!!!	////
											end 
											default:
											begin
												state_of_output[2] <= 1'b0;  ////	!!!!!!!!!!!!!!!!!!	////
											end
											endcase
										end
									end
									else  if ((CNTR[`RC:0] != `RC_size'b0) &&(state_of_output[2] == 1'b0))
									begin
										if ((CNT <= ARR) && (CNT >= CCR31) )
										begin 
											$display("CNT %b,  CCR31 %b", CNT, CCR31);
											state_of_output[2] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin
									$display("DECR CNT %d,  CCR31 %d, state %d", CNT, CCR31, state_of_output[2]);
									if (CNT == ARR) state_of_output[2] <= 1'b1; 
									else if ((CNTR[`CNT_EN]) && ((state_of_output[2] == 0) && (CNT  > CCR31 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[2] <= 1'b1;
									$display("ERROR2 state_of_output[2] %b", state_of_output[2]);
									//state_of_output[2] <= 1'b1;
									//$display("ERROR2 with CNT %d, state_of_output[2] %b", CNT, state_of_output[2]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[2] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[2] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[2] <= 1'b1;
									$display("ERROR3 state_of_output[2] %b", state_of_output[2]);
									//state_of_output[2] <= 1'b0;
									//$display("ERROR3 with CNT %d, state_of_output[2] %b", CNT, state_of_output[2]);
									end 
									else if ( (state_of_output[2] == 0) && (CNTR[`CNT_EN]) )
									begin
										if (CNT  == 0)
										begin
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[2] <= 1'b0;
											end
											default:
											begin
												state_of_output[2] <= 1'b1;
											end
											endcase       
										end 
									end
									else  if ( (state_of_output[2] == 1'b1) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT <= ARR) && (CNT <= CCR31 + `DATAWIDTH'b1) )
										begin 
											$display("decrement OS 1 CNT %b,  CCR31 %b", CNT, CCR31);
											state_of_output[2] <= 1'b0;   
										end
									end
								end 
							end
								// W/o RC //
							else if ( (CNTR[`RC:0] == `RC_size'b0) && (CNTR[`RC_SR:`RC_SR-1] == 2'b01) && (CCR31 != 0))
							begin
								//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[2] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[2] == 1) && (CNT  < CCR31)) || ((state_of_output[2] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[2] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[2] <= 1'b1;
									$display("Task ERROR1 state_of_output[2] %b", state_of_output[2]);
									state_of_output[2] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[2] %b", CNT, state_of_output[2]);
									end 
									else if (state_of_output[2] == 1)
									begin
										if ( CNT== ARR )
										begin
											state_of_output[2] <= 1'b0;       
										end
									end
									else  if ( state_of_output[2] == 1'b0 )
									begin
										if ((CNT <= ARR) && (CNT >= CCR31) )
										begin
											state_of_output[2] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin     
									if (CNT == ARR) state_of_output[2] <= 1'b1; 
									else if ((CNTR[`CNT_EN]) && ((state_of_output[2] == 0) && (CNT  > CCR31 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[2] <= 1'b1;
									$display("ERROR2 state_of_output[2] %b", state_of_output[2]);
									//state_of_output[2] <= 1'b1;
									//$display("ERROR2 with CNT %d, state_of_output[2] %b", CNT, state_of_output[2]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[2] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[2] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[2] <= 1'b1;
									$display("ERROR3 state_of_output[2] %b", state_of_output[2]);
									state_of_output[2] <= 1'b0;
									//$display("ERROR3 with CNT %d, state_of_output[2] %b", CNT, state_of_output[2]);
									end 
									else if (state_of_output[2] == 0)
									begin
										if ((CNT == 0) )
										begin
											state_of_output[2] <= 1'b1;       
										end
									end
									else  if (state_of_output[2] == 1)
									begin
										if ((CNT <= ARR) && (CNT <= CCR31 + `DATAWIDTH'b1) )
										begin
											state_of_output[2] <= 1'b0;   
										end
									end
								end 
							end
						end
					end
					if (CHCR14[`CHSR_4] == 0)
					begin
						if (CHCR14[`OSE_4] == 1'b1)
						begin
							if ((CNTR[`RC_SR:`RC_SR-1] == 2'b10) && (CCR41 != 0))
							begin
							//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[3] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[3] == 1) && (CNT  < CCR41)) || ((state_of_output[3] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[3] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[3] <= 1'b1;
									$display("Task ERROR4 state_of_output[3] %b", state_of_output[3]);
									state_of_output[3] <= 1'b0;
									$display("Task ERROR4 with CNT %d, state_of_output[3] %b", CNT, state_of_output[3]);
									end

									else if (state_of_output[3] == 1)
									begin
										if (CNT == ARR)
										begin 
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[3] <= 1'b0;
											end 
											default:
											begin
												$display("CNT %b,  ARR %b", CNT, ARR);
												state_of_output[3] <= 1'b0;
											end
											endcase
										end
									end
									else  if ((CNTR[`RC:0] != `RC_size'b0) &&(state_of_output[3] == 1'b0))
									begin
										if ((CNT <= ARR) && (CNT >= CCR41) )
										begin 
											$display("CNT %b,  CCR41 %b", CNT, CCR41);
											state_of_output[3] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin
									$display("DECR oS2 CNT %d,  CCR41 %d, state %d", CNT, CCR41, state_of_output[3]);
									if (CNT == ARR) state_of_output[3] <= 1'b1;   
									else if ((CNTR[`CNT_EN]) && ((state_of_output[3] == 0) && (CNT  > CCR41 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[3] <= 1'b1;
									$display("ERROR4 state_of_output[3] %b", state_of_output[3]);
									//state_of_output[3] <= 1'b1;
									//$display("ERROR4 with CNT %d, state_of_output[3] %b", CNT, state_of_output[3]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[3] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[3] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[3] <= 1'b1;
									$display("ERROR5 state_of_output[3] %b", state_of_output[3]);
									//state_of_output[3] <= 1'b0;
									//$display("ERROR5 with CNT %d, state_of_output[3] %b", CNT, state_of_output[3]);
									end     
									else if ( (state_of_output[3] == 0) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT  == 0) )
										begin
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[3] <= 1'b0;
											end
											default:
											begin
												state_of_output[3] <= 1'b1;
											end
											endcase       
										end 
									end
									else  if ( (state_of_output[3] == 1'b1) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT <= ARR) && (CNT <= CCR41 + `DATAWIDTH'b1) )
										begin 
											$display("decrement OS 4 CNT %b,  CCR41 %b", CNT, CCR41);
											state_of_output[3] <= 1'b0;   
										end
									end
								end 
							end
							// W/o RC //
							else if ( (CNTR[`RC:0] == `RC_size'b0) && (CNTR[`RC_SR:`RC_SR-1] == 2'b01) && (CCR41 != 0) )
							begin
								//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[3] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[3] == 1) && (CNT  < CCR41)) || ((state_of_output[3] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[3] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[3] <= 1'b1;
									$display("Task ERROR1 state_of_output[3] %b", state_of_output[3]);
									state_of_output[3] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[3] %b", CNT, state_of_output[3]);
									end
									else if ( state_of_output[3] == 1)
									begin
										if ( CNT == ARR )
										begin
											state_of_output[3] <= 1'b0;       
										end
									end
									else  if ( state_of_output[3] == 0 )
									begin
										if ((CNT <= ARR) && (CNT >= CCR41) )
										begin
											state_of_output[3] <= 1'b1;        
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin     
									if (CNT == ARR) state_of_output[3] <= 1'b1;   
									else if ((CNTR[`CNT_EN]) && ((state_of_output[3] == 0) && (CNT  > CCR41 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[3] <= 1'b1;
									$display("ERROR4 state_of_output[3] %b", state_of_output[3]);
									//state_of_output[3] <= 1'b1;
									//$display("ERROR4 with CNT %d, state_of_output[3] %b", CNT, state_of_output[3]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[3] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[3] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[3] <= 1'b1;
									$display("ERROR5 state_of_output[3] %b", state_of_output[3]);
									state_of_output[3] <= 1'b0;
									//$display("ERROR5 with CNT %d, state_of_output[3] %b", CNT, state_of_output[3]);
									end     
									else if (state_of_output[3] == 0)
									begin
										if ((CNT == 0) )
										begin
											state_of_output[3] <= 1'b1;       
										end
									end
									else  if (state_of_output[3] == 1)
									begin
										if ((CNT <= ARR) && (CNT <= CCR41 + `DATAWIDTH'b1) )
										begin 
											state_of_output[3] <= 1'b0;       
										end
									end
								end 
							end
						end
					end
				end
				else if (CHCR14 == 0) state_of_output[3:0] <= 4'b0;
				if (CHCR58 != 0)
				begin
					if (CHCR58[`CHSR_5] == 0)
					begin
						if (CHCR58[`OSE_5] == 1'b1)
						begin
							//$display("OSE_1 = 1");
							if ((CNTR[`RC_SR:`RC_SR-1] == 2'b10) && (CCR51 != 0))
							begin
							//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[4] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[4] == 1) && (CNT  < CCR51)) || ((state_of_output[4] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[4] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[4] <= 1'b1;
									$display("Task ERROR1 state_of_output[4] %b", state_of_output[4]);
									state_of_output[4] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[4] %b", CNT, state_of_output[4]);
									end
									else if (state_of_output[4] == 1)
									begin
										if (CNT == ARR)
										begin 
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[4] <= 1'b0;	////	!!!!!!!!!!!!!!!!!!	////
											end 
											default:
											begin
												state_of_output[4] <= 1'b0;  ////	!!!!!!!!!!!!!!!!!!	////
											end
											endcase
										end
									end
									else  if ((CNTR[`RC:0] != `RC_size'b0) &&(state_of_output[4] == 1'b0))
									begin
										if ((CNT <= ARR) && (CNT >= CCR51) )
										begin 
											$display("CNT %b,  CCR51 %b", CNT, CCR51);
											state_of_output[4] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin
									$display("DECR CNT %d,  CCR51 %d, state %d", CNT, CCR51, state_of_output[4]);
									if (CNT == ARR) state_of_output[4] <= 1'b1; 
									else if ((CNTR[`CNT_EN]) && ((state_of_output[4] == 0) && (CNT  > CCR51 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[4] <= 1'b1;
									$display("ERROR2 state_of_output[4] %b", state_of_output[4]);
									//state_of_output[4] <= 1'b1;
									//$display("ERROR2 with CNT %d, state_of_output[4] %b", CNT, state_of_output[4]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[4] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[4] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[4] <= 1'b1;
									$display("ERROR3 state_of_output[4] %b", state_of_output[4]);
									//state_of_output[4] <= 1'b0;
									//$display("ERROR3 with CNT %d, state_of_output[4] %b", CNT, state_of_output[4]);
									end 
									else if ( (state_of_output[4] == 0) && (CNTR[`CNT_EN]) )
									begin
										if (CNT  == 0)
										begin
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[4] <= 1'b0;
											end
											default:
											begin
												state_of_output[4] <= 1'b1;
											end
											endcase       
										end 
									end
									else  if ((state_of_output[4] == 1'b1) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT <= ARR) && (CNT <= CCR51 + `DATAWIDTH'b1) )
										begin 
											$display("decrement OS 1 CNT %b,  CCR51 %b", CNT, CCR51);
											state_of_output[4] <= 1'b0;   
										end
									end
								end 
							end
								// W/o RC //
							else if ( (CNTR[`RC:0] == `RC_size'b0) && (CNTR[`RC_SR:`RC_SR-1] == 2'b01) && (CCR51 != 0))
							begin
								//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[4] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[4] == 1) && (CNT  < CCR51)) || ((state_of_output[4] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[4] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[4] <= 1'b1;
									$display("Task ERROR1 state_of_output[4] %b", state_of_output[4]);
									state_of_output[4] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[4] %b", CNT, state_of_output[4]);
									end 
									else if (state_of_output[4] == 1)
									begin
										if ( CNT== ARR )
										begin
											state_of_output[4] <= 1'b0;       
										end
									end
									else  if ( state_of_output[4] == 1'b0 )
									begin
										if ((CNT <= ARR) && (CNT >= CCR51) )
										begin
											state_of_output[4] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin     
									if (CNT == ARR) state_of_output[4] <= 1'b1; 
									else if ((CNTR[`CNT_EN]) && ((state_of_output[4] == 0) && (CNT  > CCR51 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[4] <= 1'b1;
									$display("ERROR2 state_of_output[4] %b", state_of_output[4]);
									//state_of_output[4] <= 1'b1;
									//$display("ERROR2 with CNT %d, state_of_output[4] %b", CNT, state_of_output[4]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[4] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[4] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[4] <= 1'b1;
									$display("ERROR3 state_of_output[4] %b", state_of_output[4]);
									state_of_output[4] <= 1'b0;
									//$display("ERROR3 with CNT %d, state_of_output[4] %b", CNT, state_of_output[4]);
									end 
									else if (state_of_output[4] == 0)
									begin
										if ((CNT == 0) )
										begin
											state_of_output[4] <= 1'b1;       
										end
									end
									else  if (state_of_output[4] == 1)
									begin
										if ((CNT <= ARR) && (CNT <= CCR51 + `DATAWIDTH'b1) )
										begin
											state_of_output[4] <= 1'b0;   
										end
									end
								end 
							end
						end
					end
					if (CHCR58[`CHSR_6] == 0)
					begin
						if (CHCR58[`OSE_6] == 1'b1)
						begin
							if ( (CNTR[`RC_SR:`RC_SR-1] == 2'b10) && (CCR61 != 0))
							begin
							//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[5] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[5] == 1) && (CNT  < CCR61)) || ((state_of_output[5] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[5] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[5] <= 1'b1;
									$display("Task ERROR4 state_of_output[5] %b", state_of_output[5]);
									state_of_output[5] <= 1'b0;
									$display("Task ERROR4 with CNT %d, state_of_output[5] %b", CNT, state_of_output[5]);
									end

									else if (state_of_output[5] == 1)
									begin
										if (CNT == ARR)
										begin 
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[5] <= 1'b0;
											end 
											default:
											begin
												$display("CNT %b,  ARR %b", CNT, ARR);
												state_of_output[5] <= 1'b0;
											end
											endcase
										end
									end
									else  if ((CNTR[`RC:0] != `RC_size'b0) &&(state_of_output[5] == 1'b0))
									begin
										if ((CNT <= ARR) && (CNT >= CCR61) )
										begin 
											$display("CNT %b,  CCR61 %b", CNT, CCR61);
											state_of_output[5] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin
									$display("DECR oS2 CNT %d,  CCR61 %d, state %d", CNT, CCR61, state_of_output[5]);
									if (CNT == ARR) state_of_output[5] <= 1'b1;   
									else if ((CNTR[`CNT_EN]) && ((state_of_output[5] == 0) && (CNT  > CCR61 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[5] <= 1'b1;
									$display("ERROR4 state_of_output[5] %b", state_of_output[5]);
									//state_of_output[5] <= 1'b1;
									//$display("ERROR4 with CNT %d, state_of_output[5] %b", CNT, state_of_output[5]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[5] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[5] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[5] <= 1'b1;
									$display("ERROR5 state_of_output[5] %b", state_of_output[5]);
									//state_of_output[5] <= 1'b0;
									//$display("ERROR5 with CNT %d, state_of_output[5] %b", CNT, state_of_output[5]);
									end     
									else if ( (state_of_output[5] == 0) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT  == 0) )
										begin
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[5] <= 1'b0;
											end
											default:
											begin
												state_of_output[5] <= 1'b1;
											end
											endcase       
										end 
									end
									else  if ( (state_of_output[5] == 1'b1) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT <= ARR) && (CNT <= CCR61 + `DATAWIDTH'b1) )
										begin 
											$display("decrement OS 2 CNT %b,  CCR61 %b", CNT, CCR61);
											state_of_output[5] <= 1'b0;   
										end
									end
								end 
							end
							// W/o RC //
							else if ( (CNTR[`RC:0] == `RC_size'b0) && (CNTR[`RC_SR:`RC_SR-1] == 2'b01) && (CCR61 != 0) )
							begin
								//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[5] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[5] == 1) && (CNT  < CCR61)) || ((state_of_output[5] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[5] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[5] <= 1'b1;
									$display("Task ERROR1 state_of_output[5] %b", state_of_output[5]);
									state_of_output[5] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[5] %b", CNT, state_of_output[5]);
									end
									else if ( state_of_output[5] == 1)
									begin
										if ( CNT == ARR )
										begin
											state_of_output[5] <= 1'b0;       
										end
									end
									else  if ( state_of_output[5] == 0 )
									begin
										if ((CNT <= ARR) && (CNT >= CCR61) )
										begin
											state_of_output[5] <= 1'b1;        
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin     
									if (CNT == ARR) state_of_output[5] <= 1'b1;   
									else if ((CNTR[`CNT_EN]) && ((state_of_output[5] == 0) && (CNT  > CCR61 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[5] <= 1'b1;
									$display("ERROR4 state_of_output[5] %b", state_of_output[5]);
									//state_of_output[5] <= 1'b1;
									//$display("ERROR4 with CNT %d, state_of_output[5] %b", CNT, state_of_output[5]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[5] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[5] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[5] <= 1'b1;
									$display("ERROR5 state_of_output[5] %b", state_of_output[5]);
									state_of_output[5] <= 1'b0;
									//$display("ERROR5 with CNT %d, state_of_output[5] %b", CNT, state_of_output[5]);
									end     
									else if (state_of_output[5] == 0)
									begin
										if ((CNT == 0) )
										begin
											state_of_output[5] <= 1'b1;       
										end
									end
									else  if (state_of_output[5] == 1)
									begin
										if ((CNT <= ARR) && (CNT <= CCR61 + `DATAWIDTH'b1) )
										begin 
											state_of_output[5] <= 1'b0;       
										end
									end
								end 
							end
						end
					end
					if (CHCR58[`CHSR_7] == 0)
					begin
						if (CHCR58[`OSE_7] == 1'b1)
						begin
							//$display("OSE_1 = 1");
							if ((CNTR[`RC_SR:`RC_SR-1] == 2'b10) && (CCR71 != 0))
							begin
							//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[6] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[6] == 1) && (CNT  < CCR71)) || ((state_of_output[6] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[6] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[6] <= 1'b1;
									$display("Task ERROR1 state_of_output[6] %b", state_of_output[6]);
									state_of_output[6] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[6] %b", CNT, state_of_output[6]);
									end
									else if (state_of_output[6] == 1)
									begin
										if (CNT == ARR)
										begin 
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[6] <= 1'b0;	////	!!!!!!!!!!!!!!!!!!	////
											end 
											default:
											begin
												state_of_output[6] <= 1'b0;  ////	!!!!!!!!!!!!!!!!!!	////
											end
											endcase
										end
									end
									else  if ((CNTR[`RC:0] != `RC_size'b0) &&(state_of_output[6] == 1'b0))
									begin
										if ((CNT <= ARR) && (CNT >= CCR71) )
										begin 
											$display("CNT %b,  CCR71 %b", CNT, CCR71);
											state_of_output[6] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin
									$display("DECR CNT %d,  CCR71 %d, state %d", CNT, CCR71, state_of_output[6]);
									if (CNT == ARR) state_of_output[6] <= 1'b1; 
									else if ((CNTR[`CNT_EN]) && ((state_of_output[6] == 0) && (CNT  > CCR71 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[6] <= 1'b1;
									$display("ERROR2 state_of_output[6] %b", state_of_output[6]);
									//state_of_output[6] <= 1'b1;
									//$display("ERROR2 with CNT %d, state_of_output[6] %b", CNT, state_of_output[6]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[6] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[6] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[6] <= 1'b1;
									$display("ERROR3 state_of_output[6] %b", state_of_output[6]);
									//state_of_output[6] <= 1'b0;
									//$display("ERROR3 with CNT %d, state_of_output[6] %b", CNT, state_of_output[6]);
									end 
									else if ( (state_of_output[6] == 0) && (CNTR[`CNT_EN]) )
									begin
										if (CNT  == 0)
										begin
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[6] <= 1'b0;
											end
											default:
											begin
												state_of_output[6] <= 1'b1;
											end
											endcase       
										end 
									end
									else  if ( (state_of_output[6] == 1'b1) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT <= ARR) && (CNT <= CCR71 + `DATAWIDTH'b1) )
										begin 
											$display("decrement OS 1 CNT %b,  CCR71 %b", CNT, CCR71);
											state_of_output[6] <= 1'b0;   
										end
									end
								end 
							end
								// W/o RC //
							else if ( (CNTR[`RC:0] == `RC_size'b0) && (CNTR[`RC_SR:`RC_SR-1] == 2'b01) && (CCR71 != 0))
							begin
								//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[6] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[6] == 1) && (CNT  < CCR71)) || ((state_of_output[6] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[6] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[6] <= 1'b1;
									$display("Task ERROR1 state_of_output[6] %b", state_of_output[6]);
									state_of_output[6] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[6] %b", CNT, state_of_output[6]);
									end 
									else if (state_of_output[6] == 1)
									begin
										if ( CNT== ARR )
										begin
											state_of_output[6] <= 1'b0;       
										end
									end
									else  if ( state_of_output[6] == 1'b0 )
									begin
										if ((CNT <= ARR) && (CNT >= CCR71) )
										begin
											state_of_output[6] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin     
									if (CNT == ARR) state_of_output[6] <= 1'b1; 
									else if ((CNTR[`CNT_EN]) && ((state_of_output[6] == 0) && (CNT  > CCR71 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[6] <= 1'b1;
									$display("ERROR2 state_of_output[6] %b", state_of_output[6]);
									//state_of_output[6] <= 1'b1;
									//$display("ERROR2 with CNT %d, state_of_output[6] %b", CNT, state_of_output[6]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[6] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[6] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[6] <= 1'b1;
									$display("ERROR3 state_of_output[6] %b", state_of_output[6]);
									state_of_output[6] <= 1'b0;
									//$display("ERROR3 with CNT %d, state_of_output[6] %b", CNT, state_of_output[6]);
									end 
									else if (state_of_output[6] == 0)
									begin
										if ((CNT == 0) )
										begin
											state_of_output[6] <= 1'b1;       
										end
									end
									else  if (state_of_output[6] == 1)
									begin
										if ((CNT <= ARR) && (CNT <= CCR71 + `DATAWIDTH'b1) )
										begin
											state_of_output[6] <= 1'b0;   
										end
									end
								end 
							end
						end
					end
					if (CHCR58[`CHSR_8] == 0)
					begin
						if (CHCR58[`OSE_8] == 1'b1)
						begin
							if ((CNTR[`RC_SR:`RC_SR-1] == 2'b10) && (CCR81 != 0))
							begin
							//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[7] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[7] == 1) && (CNT  < CCR81)) || ((state_of_output[7] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[7] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[7] <= 1'b1;
									$display("Task ERROR4 state_of_output[7] %b", state_of_output[7]);
									state_of_output[7] <= 1'b0;
									$display("Task ERROR4 with CNT %d, state_of_output[7] %b", CNT, state_of_output[7]);
									end

									else if (state_of_output[7] == 1)
									begin
										if (CNT == ARR)
										begin 
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[7] <= 1'b0;
											end 
											default:
											begin
												$display("CNT %b,  ARR %b", CNT, ARR);
												state_of_output[7] <= 1'b0;
											end
											endcase
										end
									end
									else  if ((CNTR[`RC:0] != `RC_size'b0) &&(state_of_output[7] == 1'b0))
									begin
										if ((CNT <= ARR) && (CNT >= CCR81) )
										begin 
											$display("CNT %b,  CCR81 %b", CNT, CCR81);
											state_of_output[7] <= 1'b1;   
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin
									$display("DECR oS8 CNT %d,  CCR81 %d, state %d", CNT, CCR81, state_of_output[7]);
									if (CNT == ARR) state_of_output[7] <= 1'b1;   
									else if ((CNTR[`CNT_EN]) && ((state_of_output[7] == 0) && (CNT  > CCR81 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[7] <= 1'b1;
									$display("ERROR4 state_of_output[7] %b", state_of_output[7]);
									//state_of_output[7] <= 1'b1;
									//$display("ERROR4 with CNT %d, state_of_output[7] %b", CNT, state_of_output[7]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[7] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[7] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[7] <= 1'b1;
									$display("ERROR5 state_of_output[7] %b", state_of_output[7]);
									//state_of_output[7] <= 1'b0;
									//$display("ERROR5 with CNT %d, state_of_output[7] %b", CNT, state_of_output[7]);
									end     
									else if ( (state_of_output[7] == 0) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT  == 0) )
										begin
											case  (CNTR[`RC:0])
											`RC_size'b0: 
											begin
												state_of_output[7] <= 1'b0;
											end
											default:
											begin
												state_of_output[7] <= 1'b1;
											end
											endcase       
										end 
									end
									else  if ((state_of_output[7] == 1'b1) && (CNTR[`CNT_EN]) )
									begin
										if ((CNT <= ARR) && (CNT <= CCR81 + `DATAWIDTH'b1) )
										begin 
											$display("decrement OS 2 CNT %b,  CCR81 %b", CNT, CCR81);
											state_of_output[7] <= 1'b0;   
										end
									end
								end 
							end
							// W/o RC //
							else if ( (CNTR[`RC:0] == `RC_size'b0) && (CNTR[`RC_SR:`RC_SR-1] == 2'b01) && (CCR81 != 0) )
							begin
								//increment signal - set
								if (CNTR[`CDIR] == 1'b1)
								begin
									ERR_HNDLR[7] <= 1'b0;
									if ((CNTR[`CNT_EN]) && (((state_of_output[7] == 1) && (CNT  < CCR81)) || ((state_of_output[7] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[7] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[7] <= 1'b1;
									$display("Task ERROR1 state_of_output[7] %b", state_of_output[7]);
									state_of_output[7] <= 1'b0;
									$display("Task ERROR1 with CNT %d, state_of_output[7] %b", CNT, state_of_output[7]);
									end
									else if ( state_of_output[7] == 1)
									begin
										if ( CNT == ARR )
										begin
											state_of_output[7] <= 1'b0;       
										end
									end
									else  if ( state_of_output[7] == 0 )
									begin
										if ((CNT <= ARR) && (CNT >= CCR81) )
										begin
											state_of_output[7] <= 1'b1;        
										end
									end
								end
								//decrement signal - set
								else if (CNTR[`CDIR] == 1'b0)
								begin     
									if (CNT == ARR) state_of_output[7] <= 1'b1;   
									else if ((CNTR[`CNT_EN]) && ((state_of_output[7] == 0) && (CNT  > CCR81 + `DATAWIDTH'b1)))
									begin
									ERR_HNDLR[7] <= 1'b1;
									$display("ERROR4 state_of_output[7] %b", state_of_output[7]);
									//state_of_output[7] <= 1'b1;
									//$display("ERROR4 with CNT %d, state_of_output[7] %b", CNT, state_of_output[7]);
									end 
									else if ((CNTR[`CNT_EN]) && (((state_of_output[7] == 0)  && (CNT  >= ARR + `DATAWIDTH'b1)) || ((state_of_output[7] == 1)  && (CNT  >= ARR + `DATAWIDTH'b1))))
									begin
									ERR_HNDLR[7] <= 1'b1;
									$display("ERROR5 state_of_output[7] %b", state_of_output[7]);
									state_of_output[7] <= 1'b0;
									//$display("ERROR5 with CNT %d, state_of_output[7] %b", CNT, state_of_output[7]);
									end     
									else if (state_of_output[7] == 0)
									begin
										if ((CNT == 0) )
										begin
											state_of_output[7] <= 1'b1;       
										end
									end
									else  if (state_of_output[7] == 1)
									begin
										if ((CNT <= ARR) && (CNT <= CCR81 + `DATAWIDTH'b1) )
										begin 
											state_of_output[7] <= 1'b0;       
										end
									end
								end 
							end
						end
					end
				end
				else if (CHCR58 == 0) state_of_output[7:4] <= 4'b0;
			end
		end

	////////////COUNTER/////////////////
	always @(posedge i_clk, negedge i_clr)
		if (i_clr == 0)
		begin
			CNT <= 0;
		end
		else 
		begin
			if ((CNTR[`CNT_EN] == 1) && (PSG[`PSG_val:0] == cnt_for_PSG + 1'b1))
			begin
				if (CNTR[`CNT_R_EN] == 1) 
				begin
					case (CNTR[`CDIR])
					1'b1:
					begin
						CNT <= `IDLE;
					end	
					1'b0:
					begin
						CNT <= ARR;
					end 
					default:
					begin
						CNT <= `IDLE;
					end	
					endcase
				end
				else  begin
					case (CNTR[`CDIR])
					1'b1: CNT <= CNT + 1;
					1'b0: CNT <= CNT - 1;
					endcase
					
				end
			end
		end
	///////////////////////////////////
	////////	PRESCALER	///////////
	///////////////////////////////////
	////////////////////////////////////
	assign PSLVERR = 0;
	assign PRDATA = PRDATA_reg;
	assign PREADY = PREADY_reg;
	assign prev_CNTR = ~CHCR14_reg & CHCR14[`CHSR_1];

	/////////////////////////////////////////////////////////////

	assign fall_IC_CH[0] = ~CH1_IC_reg[0] & CH1_IC_reg[1];
	assign rise_IC_CH[0] = CH1_IC_reg[0] & ~CH1_IC_reg[1];
	assign fall_IC_CH[1] = ~CH2_IC_reg[0] & CH2_IC_reg[1];
	assign rise_IC_CH[1] = CH2_IC_reg[0] & ~CH2_IC_reg[1];
	assign fall_IC_CH[2] = ~CH3_IC_reg[0] & CH3_IC_reg[1];
	assign rise_IC_CH[2] = CH3_IC_reg[0] & ~CH3_IC_reg[1];
	assign fall_IC_CH[3] = ~CH4_IC_reg[0] & CH4_IC_reg[1];
	assign rise_IC_CH[3] = CH4_IC_reg[0] & ~CH4_IC_reg[1];
	assign fall_IC_CH[4] = ~CH5_IC_reg[0] & CH5_IC_reg[1];
	assign rise_IC_CH[4] = CH5_IC_reg[0] & ~CH5_IC_reg[1];
	assign fall_IC_CH[5] = ~CH6_IC_reg[0] & CH6_IC_reg[1];
	assign rise_IC_CH[5] = CH6_IC_reg[0] & ~CH6_IC_reg[1];
	assign fall_IC_CH[6] = ~CH7_IC_reg[0] & CH7_IC_reg[1];
	assign rise_IC_CH[6] = CH7_IC_reg[0] & ~CH7_IC_reg[1];
	assign fall_IC_CH[7] = ~CH8_IC_reg[0] & CH8_IC_reg[1];
	assign rise_IC_CH[7] =CH8_IC_reg[0] & ~CH8_IC_reg[1];

	//////////////////////////////////////////////////////////////

	assign o_INTR[0] = CHCR14[`CHINTR_1]? o_INTR_reg[0]: 1'b0;
	assign o_INTR[1] = CHCR14[`CHINTR_2]? o_INTR_reg[1]: 1'b0;
	assign o_INTR[2] = CHCR14[`CHINTR_3]? o_INTR_reg[2]: 1'b0;
	assign o_INTR[3] = CHCR14[`CHINTR_4]? o_INTR_reg[3]: 1'b0;
	assign o_INTR[4] = CHCR58[`CHINTR_5]? o_INTR_reg[4]: 1'b0;
	assign o_INTR[5] = CHCR58[`CHINTR_6]? o_INTR_reg[5]: 1'b0;
	assign o_INTR[6] = CHCR58[`CHINTR_7]? o_INTR_reg[6]: 1'b0;
	assign o_INTR[7] = CHCR58[`CHINTR_8]? o_INTR_reg[7]: 1'b0;

	///////////////////////////////////////////////////////////

	assign o_PWM[0] = CNTR[`OS_EN]? state_of_output[0]: 1'b0;
	assign o_PWM[1] = CNTR[`OS_EN]? state_of_output[1]: 1'b0;
	assign o_PWM[2] = CNTR[`OS_EN]? state_of_output[2]: 1'b0;
	assign o_PWM[3] = CNTR[`OS_EN]? state_of_output[3]: 1'b0;
	assign o_PWM[4] = CNTR[`OS_EN]? state_of_output[4]: 1'b0;
	assign o_PWM[5] = CNTR[`OS_EN]? state_of_output[5]: 1'b0;
	assign o_PWM[6] = CNTR[`OS_EN]? state_of_output[6]: 1'b0;
	assign o_PWM[7] = CNTR[`OS_EN]? state_of_output[7]: 1'b0;

	endmodule
`endif


