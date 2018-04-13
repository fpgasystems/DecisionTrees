
/*
 * Copyright 2018 - 2019 Systems Group, ETH Zurich
 *
 * This hardware operator is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


module FPAggregator #(parameter FP_ADDER_LATENCY = 2) (

		input   wire 						clk,
		input   wire 						rst_n,

		input   wire [31:0]					fp_in,
		input   wire 						fp_in_valid,
		input   wire 						fp_in_last,
		output  wire                        fp_in_ready,

		output  reg  [31:0]					aggreg_out,
		output  reg  						aggreg_out_valid,
		input   wire                        aggreg_out_ready
	);




wire 									aggreg_in_fifo_full;
wire 									aggreg_in_fifo_valid;
wire 									aggreg_in_fifo_re;
wire 	[32:0]							aggreg_in_fifo_dout;

wire 	[33:0]							input_A;
reg  	[33:0]							prev_aggreg_value;
wire 	[33:0]							aggreg_value;
reg     [3:0] 							fpadder_latency_count;


wire 									fp_in_valid_delayed;
wire 									fp_in_last_delayed;
wire 									aggregator_ready;

////////////////////////////////////////////////////////////////////////////////
assign fp_in_ready = ~aggreg_in_fifo_full;

quick_fifo  #(.FIFO_WIDTH(32+1),        
              .FIFO_DEPTH_BITS(9),
              .FIFO_ALMOSTFULL_THRESHOLD(508)
             ) aggreg_in_fifo (
        	.clk                (clk),
        	.reset_n            (rst_n),
        	.din                ({fp_in_last, fp_in}),
        	.we                 (fp_in_valid),

        	.re                 (aggreg_in_fifo_re),
        	.dout               (aggreg_in_fifo_dout),
        	.empty              (),
        	.valid              (aggreg_in_fifo_valid),
        	.full               (aggreg_in_fifo_full),
        	.count              (),
        	.almostfull         ()
    	);

assign aggreg_in_fifo_re = aggregator_ready;
////////////////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
	if (~rst_n) begin
		// reset
		prev_aggreg_value      <= 0;
		fpadder_latency_count  <= 0;
	end
	else begin
		if(aggregator_ready & aggreg_in_fifo_valid) begin
			fpadder_latency_count  <= FP_ADDER_LATENCY-1;
		end
		else if(~aggregator_ready) begin
			fpadder_latency_count  <= fpadder_latency_count - 1'b1;
		end
		//--------------------- Do aggregation --------------------------//
		if(fp_in_valid_delayed) begin 
			if(~fp_in_last_delayed) begin
				prev_aggreg_value <= aggreg_value;
			end
			else begin
				prev_aggreg_value <= 0;
			end
		end
                
		//--------------------- Tuple Output ----------------------------//
		aggreg_out_valid <= 1'b0;

		if(fp_in_valid_delayed & fp_in_last_delayed) begin 
			if(aggreg_value[33:32] == 2'b00) begin
				aggreg_out       <=  0;
			end
			else begin
				aggreg_out       <= aggreg_value[31:0];
			end
			
			aggreg_out_valid <= 1'b1;
		end
	end
end

assign aggregator_ready = (fpadder_latency_count == 0);

assign input_A = {1'b0, {|(aggreg_in_fifo_dout[31:0])}, aggreg_in_fifo_dout[31:0]};

FPAdder_8_23_uid2_l2 fpadder(
				.clk          (clk),
				.rst          (~rst_n),
				.seq_stall    (1'b0),
				.X            (input_A),
				.Y            (prev_aggreg_value),
				.R            (aggreg_value)
				);

// delay valid, last with FPAdder Latency
delay #(.DATA_WIDTH(1),
	    .DELAY_CYCLES(FP_ADDER_LATENCY) 
	) fpadder_delay(

	    .clk              (clk),
	    .rst_n            (rst_n),
	    .data_in          (aggreg_in_fifo_dout[32]),   // 
	    .data_in_valid    (aggreg_in_fifo_valid & aggregator_ready),
	    .data_out         (fp_in_last_delayed),
	    .data_out_valid   (fp_in_valid_delayed)
	);







endmodule
