/**
*	@author: Nishad Saraf
*	This verilog program is used to count the high and low pulses of the clock signal as the input goes 
*	from low to high and high to low.
*	input signal has a frequency of 4KHz while the input clock signal has a frequency of 10MHz
*/

module PWM_detect(H_Cntr,L_Cntr,clk,reset,signal);
// port declarations
input  clk;
input  signal;
input  reset;
// temporary 32-bit registers to store high and low count
reg [31:0] highcount	=	0;
reg [31:0] lowcount		=	0;
// final output ports
output reg [31:0] H_Cntr;
output reg [31:0] L_Cntr;
// these registers are used a flag to know the start of next cycle of input signal
reg high	=	1'b0;
reg low		=	1'b0;

always @(posedge clk or negedge reset)
begin
	// when system restarts initialize the counters to zero
	if(!reset)
	begin
		H_Cntr <= 1'b0;
		L_Cntr <= 1'b0;
	end
	else
	begin
		if(signal)
		begin
			// this statement if true ark the start of next high cycle of input signal
			if(high == 1'b1 && low == 1'b1)
			begin
				// output the count values on the output port
				H_Cntr <= highcount;
				L_Cntr <= lowcount;
				// as 1 high count is wasted to detect the start of next cycle initialize the count from 1 instead of zero
				highcount	<= 1'b1;
				// reset the low count and flags
				lowcount	<= 1'b0;
				high <= 1'b0;
				low  <= 1'b0;
			end
			else
			begin
				// set the high flag to record that postive part of input cycle 
				high <= 1'b1;
				// increment the counter
				highcount <= highcount + 1;
			end

		end
		else	// when the input signal goes low
		begin
			if(high==1'b1)
			begin
				// set the flag to record the negative part of input cycle
				low 		<= 1'b1;
				// increment the count value
				lowcount	<= lowcount + 1;
			end
			else
			begin
				low <= 1'b0;
			end
		end
	end
end
endmodule

// test beanch for PWM_detect
module pwmdetect_test;
  reg	        clk,signal,reset;						
  wire			[31:0] 		H_Cntr;	    
  wire			[31:0] 		L_Cntr;	   
 // instantiate the PWM_detect module
 PWM_detect test1(H_Cntr,L_Cntr,clk,reset,signal);		
  initial
   begin 
      clk=1'b0;
      signal= 1'b0;
      forever #5 clk=~clk;	// simulate the clock input signal
     
    end 
  initial 
    begin 
		// after 3time delay assign value to the signals
     #0 reset=1'b1;	
     #0 signal=1'b0;
     #10 signal= 1'b1;
     #20 signal= 1'b0; 
     #30 signal= 1'b1;
     #40 signal = 1'b0;

    end   
endmodule  