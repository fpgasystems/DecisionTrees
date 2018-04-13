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
 

module FPAddersReduceTree #(parameter NUM_FP_POINTS    = 8,
						    parameter FP_ADDER_LATENCY = 2) 
   (
		input   wire 						clk,
		input   wire 						rst_n,

		input   wire [31:0]					fp_in_vector[NUM_FP_POINTS-1:0],
		input   wire 						fp_in_vector_valid[NUM_FP_POINTS-1:0],
		input   wire 						fp_in_vector_last[NUM_FP_POINTS-1:0],

		output  wire [31:0]				    reduce_out,
		output  wire 					    reduce_out_valid,
		input   wire  						reduce_out_ready
	);


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////               Local Parameters              /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

localparam NUM_TREE_LEVELS = (NUM_FP_POINTS == 16)? 4 :
                             (NUM_FP_POINTS ==  8)? 3 :
                             (NUM_FP_POINTS ==  4)? 2 : 1;


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////            Signals Declarations             /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

wire   [33:0]                  tree_data[NUM_TREE_LEVELS:0][(NUM_FP_POINTS>>1)-1:0][1:0];
wire   [31:0] 				   tree_out;

wire 						   fp_in_valid_delayed;
wire  						   fp_in_last_delayed;


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////               FP Adders Tree                /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// first level of tree adders
generate
	genvar i;
	for (i = 0; i < (NUM_FP_POINTS>>1); i = i + 1)
	begin:treeLevel1

	    assign tree_data[0][i][0] = {1'b0, {|(fp_in_vector[i<<1])}, fp_in_vector[i<<1] };
	    assign tree_data[0][i][1] = {1'b0, {|(fp_in_vector[(i<<1)+1])}, fp_in_vector[(i<<1)+1] };
		
		FPAdder_8_23_uid2_l2 fpadder_1_x(
				.clk          (clk),
				.rst          (~rst_n),
				.seq_stall    (1'b0),
				.X            (tree_data[0][i][0]),
				.Y            (tree_data[0][i][1]),
				.R            (tree_data[1][i>>1][i%2])
				);
	end
endgenerate

// the rest of levels
generate
	genvar j;
	for (i = 1; i < NUM_TREE_LEVELS; i = i + 1)
	begin:treeLevels
		for (j = 0; j < (NUM_FP_POINTS >> (i+1)); j = j + 1)
			begin:levelAdders
				FPAdder_8_23_uid2_l2 fpadder_i_x(
					.clk          (clk),
					.rst          (~rst_n),
					.seq_stall    (1'b0),
					.X            (tree_data[i][j][0]),
					.Y            (tree_data[i][j][1]),
					.R            (tree_data[i+1][j>>1][j%2])
				);
			end 
	end
endgenerate

// delay valid and last
delay #(.DATA_WIDTH(1),
	    .DELAY_CYCLES(FP_ADDER_LATENCY*NUM_TREE_LEVELS) 
	) fpadder_delay(

	    .clk              (clk),
	    .rst_n            (rst_n),
	    .data_in          (fp_in_vector_last[0]),   // 
	    .data_in_valid    (fp_in_vector_valid[0]),
	    .data_out         (fp_in_last_delayed),
	    .data_out_valid   (fp_in_valid_delayed)
	);

// assign tree output 
assign tree_out = (tree_data[NUM_TREE_LEVELS][0][0][33:32] == 2'b00)? 0 : tree_data[NUM_TREE_LEVELS][0][0][31:0];

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////               FP Aggregator                 /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

FPAggregator #(.FP_ADDER_LATENCY(FP_ADDER_LATENCY)) 

 tree_aggregator(

		.clk                (clk),
		.rst_n              (rst_n),

		.fp_in              (tree_out),
		.fp_in_valid        (fp_in_valid_delayed),
		.fp_in_last         (fp_in_last_delayed),
		.fp_in_ready        (),

		.aggreg_out         (reduce_out),
		.aggreg_out_valid   (reduce_out_valid),
		.aggreg_out_ready   (reduce_out_ready)
	);






endmodule

