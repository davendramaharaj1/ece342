//Top module for game
module part2 
(
    // Clock pins
    input                     clk,
    input                     reset,
    input                     enter,
    input               [7:0] guess,
    output              [7:0] actual,
	 output					[3:0] dp_tries,
    output                    dp_over,
    output                    dp_under,
    output                    dp_equal
);

	logic over;
	logic under;
	logic equal;
	logic done;
	logic enable;

	// Datapath
	logic dp_inc_actual;
	datapath the_datapath
	(
		.clk(clk),
		.reset(reset),
		.i_guess(guess),
		.i_inc_actual(dp_inc_actual),
		.i_decr_enable(enable),
		.o_over(over),
		.o_under(under),
		.o_equal(equal),
		.o_done(done),
        .actual(actual),
		  .attempts(dp_tries)
	);
	
	// State Machine
	logic ctrl_update_leds;
	control the_control
	(
		.clk(clk),
		.reset(reset),
		.i_enter(enter),
		.o_inc_actual(dp_inc_actual),
		.o_decr_enable(enable),
		.i_over(over),
		.i_under(under),
		.i_equal(equal),
		.i_done(done),
		.o_update_leds(ctrl_update_leds)
	);
	
	// LED controllers
	led_ctrl ledc_under(clk, reset, under, ctrl_update_leds, dp_under);
	led_ctrl ledc_over(clk, reset, over, ctrl_update_leds, dp_over);
	led_ctrl ledc_equal(clk, reset, equal, ctrl_update_leds, dp_equal);
	
endmodule

/*******************************************************/
/********************Control module********************/
/*****************************************************/
module control
(
	input clk,
	input reset,
	
	// Button input
	input i_enter,
	
	// Datapath
	output logic o_inc_actual,
	output logic o_decr_enable,
	input i_over,
	input i_under,
	input i_equal,
	input i_done,
	
	// LED Control
	output logic o_update_leds
);

// Declare two objects, 'state' and 'nextstate'
// that are of enum type.
enum int unsigned
{
	S_GEN_RAND,
	S_CHECK,
	S_WAIT_NOENTER,
	S_WAIT_ENTER,
	S_END
} state, nextstate;

// Clocked always block for making state registers
always_ff @ (posedge clk or posedge reset) begin
	if (reset) state <= S_GEN_RAND;
	else state <= nextstate;
end

// always_comb replaces always @* and gives compile-time errors instead of warnings
// if you accidentally infer a latch
always_comb begin
	nextstate = state;
	o_inc_actual = 1'b0;
	o_update_leds = 1'b0;
	o_decr_enable = 1'b0;
	
	case (state)
		// generating a random number 
		S_GEN_RAND: begin
			o_inc_actual = 1'b1;
			if (i_enter) nextstate = S_WAIT_NOENTER;
		end
		
		// waiting on the user to release the enter button
		S_WAIT_NOENTER: begin
			// stop the count!
			o_inc_actual = 1'b0;
			if (!i_enter) begin
				// decrement the attempts 
				o_decr_enable = 1'b1;
				nextstate = S_CHECK;
			end
		end
		
		//update the LEDs and check if the game is over (guess = actual)
		S_CHECK: begin
			// update the LEDS
			o_update_leds = 1'b1;
			if (i_equal || i_done) nextstate = S_END;
			else nextstate = S_WAIT_ENTER;
		end
		
		//wait for the user to enter another number
		S_WAIT_ENTER: begin
			if (i_enter) nextstate = S_WAIT_NOENTER;
		end
		
		// end state, should return user to generate another random number
		S_END: begin
			nextstate = S_END;
		end
	endcase
end
endmodule

/*******************************************************/
/********************Datapath module*******************/
/*****************************************************/
module datapath
(
	input clk,
	input reset,
	
	// Number entry
	input [7:0] i_guess,
	
	// Increment actual and enable
	input i_inc_actual,
	input i_decr_enable,
	
	// Comparison result
	output o_over,
	output o_under,
	output o_equal,
	output o_done,
	output logic [7:0] actual,
	output logic [3:0] attempts
);

// Update the 'actual' register based on control signals
always_ff @ (posedge clk or posedge reset) begin
	if (reset) actual <= '0;
	else begin
		if (i_inc_actual) actual <= actual + 8'd1;
	end
end

// update the attemmpts register based on the control signals to track the tries left
always_ff @ (posedge clk or posedge reset) begin
	if(reset) attempts <= 4'd7;
	else begin
		if (i_decr_enable) attempts <= attempts - 4'd1;
	end
end

// Generate comparisons and done signal
assign o_over = i_guess > actual;
assign o_equal = i_guess == actual;
assign o_under = i_guess < actual;
assign o_done = attempts == 0;
endmodule

/*******************************************************/
/********************LED control module****************/
/*****************************************************/
module led_ctrl
(
	input clk,
	input reset,
	
	input i_val,
	input i_enable,
	output logic o_out
);

always_ff @ (posedge clk or posedge reset) begin
	if (reset) o_out <= '0;
	else if (i_enable) o_out <= i_val;
end

endmodule
