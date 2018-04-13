
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
 
/*
     
*/

`include "DTEngine_defines.vh"
`include "../../framework_defines.vh"

module DTEngine (
	input   wire                                   clk,
    input   wire                                   rst_n,
    //-------------------------------------------------//
	input   wire 					               start_um,
    input   wire [511:0]                           um_params,
    output  reg                                    um_done,

    output  wire [`NUM_USER_STATE_COUNTERS*32-1:0] um_state_counters,
    output  wire                                   um_state_counters_valid,
    // TX RD
    output  reg  [57:0]                            um_tx_rd_addr,
    output  reg  [7:0]                             um_tx_rd_tag,
    output  reg  						           um_tx_rd_valid,
    input   wire                                   um_tx_rd_ready,
    // TX WR
    output  reg  [57:0]                            um_tx_wr_addr,
    output  reg  [7:0]                             um_tx_wr_tag,
    output  reg 					               um_tx_wr_valid,
    output  reg  [511:0]			               um_tx_data,
    input   wire                                   um_tx_wr_ready,
    // RX RD
    input   wire [7:0]                             um_rx_rd_tag,
    input   wire [511:0]                           um_rx_data,
    input   wire                                   um_rx_rd_valid,
    output  wire                                   um_rx_rd_ready,
    // RX WR 
    input   wire                                   um_rx_wr_valid,
    input   wire [7:0]                             um_rx_wr_tag
	);


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////               Local Parameters              /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

localparam DATA_LINE_DISTR_LEVELS = (`NUM_DTPU_CLUSTERS == 8)? 3 : 
                                    (`NUM_DTPU_CLUSTERS == 4)? 2 : 1;


localparam [1:0]  	IDLE         = 2'b00,
					PROG_MODE    = 2'b01,
					PROCESS_MODE = 2'b10,
					ENGINE_DONE  = 2'b11;

localparam  TREE_WEIGHTS_PROG       = 1'b0;
localparam  TREE_FEATURE_INDEX_PROG = 1'b1;

localparam  WAIT_CYCLES_FOR_LAST_TREE = 16;
localparam  FP_ADDER_LATENCY          = 2;
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////            Signals Declarations             /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

reg 								params_valid;

reg 								program_en;     
reg 								process_en;
reg 								partial_mode;

reg 	[3:0]  						num_levels_per_tree_minus_one;   
reg 	[7:0]  						num_trees_per_pu_minus_one;  
reg 	[3:0]  						num_clusters_per_tuple;
reg     [3:0]                       num_clusters_per_tuple_minus_one;  

reg     [11:0]                      num_trees_minus_one;

reg     [15:0]                      tree_weights_numcls;
reg  	[15:0]  					tree_weights_numcls_minus_one;
reg     [15:0]                      tree_feature_index_numcls;
reg 	[15:0]  					tree_feature_index_numcls_minus_one;
reg 	[15:0]  					tuple_numcls_minus_one;
reg 	[15:0]  					tuple_numcls;
reg 	[15:0]  					num_features_per_tuple;

reg     [31:0]                      total_trees_weights_numcls;
reg 	[31:0]  					total_trees_weights_numcls_minus_one;
reg 	[31:0]  					total_trees_feature_index_numcls_minus_one;
reg     [31:0]                      total_trees_feature_index_numcls;
reg 	[31:0]  					total_features_numcls_minus_one;
reg 	[31:0]  					total_output_numcls_minus_one;
reg     [31:0]                      total_output_numcls;

reg     [31:0]                      missing_value;

reg 	[57:0]  					tree_weights_base;
reg  	[57:0]  					tree_feature_index_base;
reg 	[57:0]  					data_features_base;
reg  	[57:0]  					dt_out_base_addr;

reg 	[1:0]  						engine_fsm_state;

reg 	[31:0]  					tree_weights_addr;
reg 		  						weights_mode;
reg 	[31:0]  					tree_feature_index_addr;
reg 		  						program_read_done;

reg 	[31:0]  					features_addr;
 
reg 	[`NUM_DTPU_CLUSTERS-1:0]   	prog_schedule;
reg 	[`NUM_DTPU_CLUSTERS-1:0]   	proc_schedule;

reg 	     					   	init_w;
reg 		  					   	init_idx;
reg 		  						init_p;
reg 	[7:0]  						last_tree_latency_count;
reg 	[31:0]  					num_written_cls;

wire                                receiving_program_done;
wire                                distributing_program_done;
wire                                program_done;
wire                                process_done;
wire                                engine_done;

reg 	[8:0]  						received_cl_count;
reg 	[31:0]  					received_weights_cls;
reg 	[31:0]  					received_findexes_cls;
reg 	[31:0]  					received_tuples_cls;

wire 								single_tree_weights_received;
wire 								single_tree_feature_indexes_received;
wire 								single_tuple_features_received;
wire 								data_last_flag;
wire 								InDataFIFO_re;
wire 								InDataFIFO_empty;
wire 								InDataFIFO_valid_out;
wire 								InDataFIFO_full;
wire    [515:0]						InDataFIFO_dout;

reg   								   data_read_done;
reg 								   shift_enable;
reg     [`NUM_DTPU_CLUSTERS_BITS-1:0]  shift_count;
reg     [`NUM_DTPU_CLUSTERS-1:0]       schedule_to_shift;
wire    [`NUM_DTPU_CLUSTERS-1:0]       shifted_schedule;


reg 								data_line_distr_valid[DATA_LINE_DISTR_LEVELS:0][(2**DATA_LINE_DISTR_LEVELS)-1:0];
reg 								data_line_distr_last[DATA_LINE_DISTR_LEVELS:0][(2**DATA_LINE_DISTR_LEVELS)-1:0];
reg 								data_line_distr_ctrl[DATA_LINE_DISTR_LEVELS:0][(2**DATA_LINE_DISTR_LEVELS)-1:0];
reg 								data_line_part;
reg 	[1:0]						data_line_distr_prog[DATA_LINE_DISTR_LEVELS:0][(2**DATA_LINE_DISTR_LEVELS)-1:0];
reg     [`NUM_DTPU_CLUSTERS-1:0]    data_line_distr_en[DATA_LINE_DISTR_LEVELS:0][(2**DATA_LINE_DISTR_LEVELS)-1:0];
reg 	[2:0]						curr_pu;
reg     [2:0]                       data_line_distr_pu[DATA_LINE_DISTR_LEVELS:0][(2**DATA_LINE_DISTR_LEVELS)-1:0];
reg     [2:0]                       num_trees_sent_to_cluster;
reg     [255:0] 					data_line_distr[DATA_LINE_DISTR_LEVELS:0][(2**DATA_LINE_DISTR_LEVELS)-1:0];

wire								data_line_valid_array[`NUM_DTPU_CLUSTERS-1:0];
wire								data_line_last_array[`NUM_DTPU_CLUSTERS-1:0];
wire								data_line_ctrl_array[`NUM_DTPU_CLUSTERS-1:0];
wire	[1:0]						data_line_prog_array[`NUM_DTPU_CLUSTERS-1:0];
wire    [2:0]                       data_line_pu_array[`NUM_DTPU_CLUSTERS-1:0];
wire    [`NUM_DTPU_CLUSTERS-1:0]    data_line_en_array[`NUM_DTPU_CLUSTERS-1:0];
wire    [255:0] 					data_line_array[`NUM_DTPU_CLUSTERS-1:0];
wire								data_line_ready_array[`NUM_DTPU_CLUSTERS-1:0];

wire  	[16*`NUM_PUS_PER_CLUSTER-1:0] 	pu_tree_node_index_out[`NUM_DTPU_CLUSTERS-1:0];
wire  								  	pu_tree_node_index_out_valid[`NUM_DTPU_CLUSTERS-1:0];

wire    [31:0] 						partial_aggregation_out[`NUM_DTPU_CLUSTERS-1:0];
wire    	 						partial_aggregation_out_valid[`NUM_DTPU_CLUSTERS-1:0];
wire    	 						partial_aggregation_out_ready[`NUM_DTPU_CLUSTERS-1:0];

wire    [511:0] 					partial_tree_node_indexes_CL1;
wire    [511:0] 					partial_tree_node_indexes_CL2;
wire  								partial_tree_node_indexes_CL1_valid;
wire  								partial_tree_node_indexes_CL2_valid;

wire  								tree_index_out_fifo1_re;
wire  								tree_index_out_fifo1_valid;
wire    [511:0] 					tree_index_out_fifo1_dout;

wire  								tree_index_out_fifo2_re;
wire  								tree_index_out_fifo2_valid;
wire    [511:0] 					tree_index_out_fifo2_dout;


wire   	[`NUM_DTPU_CLUSTERS_BITS-1:0]   curr_cluster;
wire 							 	    curr_cluster_valid;

reg    	[31:0]                    		partial_leaf_aggreg_value;
reg    		                     		partial_leaf_aggreg_value_valid;
reg    		                     		partial_leaf_aggreg_value_last;
reg    	[`NUM_DTPU_CLUSTERS_BITS-1:0]   tuple_cluster_offset;
reg    	[`NUM_DTPU_CLUSTERS_BITS-1:0]   tuple_cluster_base;

wire                                aggregator_ready;

reg    	[31:0]                    	tuple_out_data;
reg  							 	tuple_out_data_valid;

reg    	[31:0]                    	tree_leaf_aggreg_dout_array[15:0];
reg  							 	tree_leaf_aggreg_valid;
reg  							 	tree_leaf_aggreg_valid_re;
reg  							 	tree_leaf_aggreg_dout_array_valid; 
reg    	[3:0]                     	tuple_pos;

reg    	[511:0]                   	tree_leaf_aggreg_dout;


reg                                 pick_cl_id;
reg     [31:0] 					    curr_out_addr;

wire    [`NUM_DTPU_CLUSTERS-1:0]    clusters_ready;
wire    							target_clusters_ready;

reg     [31:0] 						programmingCycles;
reg     [31:0] 						processingCycles;
reg     [11:0]                      num_written_tree_cls;
reg     [11:0] 						num_trees_index_out_cls_minus_one;
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////            DT Engine Parameters             /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


always @(posedge clk) begin
	if (~rst_n) begin
		params_valid                               <= 1'b0;

		program_en                                 <= 0;     
		process_en                                 <= 0;
		partial_mode                               <= 0;

		num_levels_per_tree_minus_one              <= 0;   
		num_trees_per_pu_minus_one                 <= 0;  
		num_clusters_per_tuple                     <= 0;  
                
        tree_weights_numcls                        <= 0;
		tree_weights_numcls_minus_one              <= 0;
		tree_feature_index_numcls_minus_one        <= 0;
		tree_feature_index_numcls 			       <= 0;
		tuple_numcls_minus_one                     <= 0;
		num_features_per_tuple                     <= 0;

		total_trees_weights_numcls_minus_one       <= 0;
		total_trees_feature_index_numcls_minus_one <= 0;
		total_features_numcls_minus_one            <= 0;
		total_output_numcls_minus_one              <= 0;

		tree_weights_base                          <= 0;
		tree_feature_index_base                    <= 0;
		data_features_base                         <= 0;
		dt_out_base_addr                           <= 0;

		tuple_numcls                               <= 0;

		num_trees_minus_one                        <= 0;
		num_trees_index_out_cls_minus_one          <= 0;

		missing_value                              <= 0;
	end
	else if (start_um) begin
		params_valid                               <= 1'b1;

		// params CL#1
		tree_weights_base                          <= um_params[63:6];           // 58-bits
		tree_feature_index_base                    <= um_params[127:70];		 // 58-bits
		data_features_base                         <= um_params[191:134];        // 58-bits
		dt_out_base_addr                           <= um_params[255:198];        // 58-bits

		total_trees_weights_numcls_minus_one       <= um_params[287:256] - 1'b1;   // 4 bytes
        total_trees_weights_numcls                 <= um_params[287:256];
		total_trees_feature_index_numcls_minus_one <= um_params[319:288] - 1'b1;   // 4 bytes
        total_trees_feature_index_numcls           <= um_params[319:288];
		total_features_numcls_minus_one            <= um_params[351:320] - 1'b1;   // 4 bytes
		total_output_numcls                        <= um_params[383:352];          // 4 bytes

		tree_weights_numcls                        <= um_params[399:384];          // 2 bytes
		tree_weights_numcls_minus_one              <= um_params[399:384] - 1'b1;   
		tree_feature_index_numcls_minus_one        <= um_params[415:400] - 1'b1;   // 2 bytes
		tree_feature_index_numcls                  <= um_params[415:400];          
		tuple_numcls_minus_one                     <= um_params[431:416] - 1'b1;   // 2 bytes
		tuple_numcls                               <= um_params[431:416];          
		num_features_per_tuple                     <= um_params[447:432];          // 2 bytes

		num_trees_minus_one                        <= um_params[459:448] - 1'b1;   // 12 bits

		num_trees_index_out_cls_minus_one          <= um_params[459:453] - 1'b1 + (((um_params[452:448]) != 5'b0)? 1'b1 : 1'b0);   // 12 bits

		num_levels_per_tree_minus_one              <= um_params[463:460] - 1'b1;   // 4 bits     
		num_clusters_per_tuple                     <= um_params[467:464];          // 4 bits
        num_clusters_per_tuple_minus_one           <= um_params[467:464] - 1'b1; 

        program_en                                 <= um_params[468];               // flags
		process_en                                 <= um_params[469];
		partial_mode                               <= um_params[470];

        num_trees_per_pu_minus_one                 <= um_params[479:472] - 1'b1;    // 1 byte     

		missing_value                              <= um_params[511:480]; 		    // 4 bytes
	end
	else if(engine_done) begin
		program_en                                 <= 0;    
		process_en                                 <= 0;
	end
end

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////            Performance counters             /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
	if(~rst_n) begin
		programmingCycles <= 0;
		processingCycles <= 0;
	end 
	else begin
		if(engine_fsm_state == PROG_MODE) begin 
			programmingCycles <= programmingCycles + 1'b1;
		end
		
		if(engine_fsm_state == PROCESS_MODE) begin 
			processingCycles <= processingCycles + 1'b1;
		end
	end
end

assign um_state_counters       = {{(`NUM_USER_STATE_COUNTERS-2){32'b0}}, processingCycles, programmingCycles};
assign um_state_counters_valid = um_done;
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////            Engine State Machine             /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
	if(~rst_n) begin
		engine_fsm_state        <= IDLE;

		tree_weights_addr       <= 0;
		weights_mode            <= TREE_WEIGHTS_PROG;
		tree_feature_index_addr <= 0;
		program_read_done       <= 0;

		features_addr           <= 0;
 		
 		um_tx_rd_tag            <= 0;
		um_tx_rd_valid          <= 0;
		um_tx_rd_addr           <= 0;
		um_done                 <= 0;

		prog_schedule           <= 0;
		proc_schedule           <= 0;

		init_w                  <= 0;
		init_idx                <= 0;
		init_p                  <= 0;
		last_tree_latency_count <= 0;
		num_written_cls         <= 0;
         
        data_read_done          <= 0;
	end 
	else begin
		case (engine_fsm_state)
			IDLE: begin 

				if( program_en ) begin 
					//- If the engine require programming we go to the program 
					//  state first otherwise directly to process mode
					engine_fsm_state        <= PROG_MODE;
				end
				else if(process_en) begin
					engine_fsm_state        <= PROCESS_MODE;
				end

				tree_weights_addr       <= 0;
				weights_mode            <= TREE_WEIGHTS_PROG;
				tree_feature_index_addr <= 0;
				program_read_done       <= 0;

				features_addr           <= 0;
				um_done                 <= 0;
				um_tx_rd_tag            <= 0;
				um_tx_rd_valid          <= 0;
				um_tx_rd_addr           <= 0;

				init_w                  <= 0;
				init_idx                <= 0;
				init_p                  <= 0;
				last_tree_latency_count <= 0;
				num_written_cls         <= 0;

				case (num_clusters_per_tuple)
					4'd1: begin 
						prog_schedule <= {`NUM_DTPU_CLUSTERS{1'b1}};
						proc_schedule <= {{(`NUM_DTPU_CLUSTERS-1){1'b0}}, 1'b1};
					end
					4'd2: begin 
						prog_schedule <= {(`NUM_DTPU_CLUSTERS/2){2'b01}};
						proc_schedule <= {{(`NUM_DTPU_CLUSTERS-2){1'b0}}, 2'b11};
					end
					4'd3: begin 
						prog_schedule <= {(`NUM_DTPU_CLUSTERS/3){3'b001}};
						proc_schedule <= {{(`NUM_DTPU_CLUSTERS-3){1'b0}}, 3'b111};
					end
					4'd4: begin 
						prog_schedule <= {(`NUM_DTPU_CLUSTERS/4){4'b0001}};
						proc_schedule <= {{(`NUM_DTPU_CLUSTERS-4){1'b0}}, 4'b1111};
					end
					/*4'd8: begin 
						prog_schedule <= {(`NUM_DTPU_CLUSTERS/8){8'b00000001}};
						proc_schedule <= {{(`NUM_DTPU_CLUSTERS-8){1'b0}}, 8'b11111111};
					end*/
					default : begin 
						prog_schedule <= {(`NUM_DTPU_CLUSTERS/4){4'b0001}};
						proc_schedule <= {{(`NUM_DTPU_CLUSTERS-4){1'b0}}, 4'b1111};
					end
				endcase
			end
			PROG_MODE: begin 
			    // Programming mode is done when all trees are written to their destination PU
				if(program_done) begin 
					engine_fsm_state <= PROCESS_MODE;
					um_tx_rd_valid   <= 0;
				end
				else if(um_tx_rd_ready & ~program_read_done) begin 
					um_tx_rd_valid   <= 1'b1;
					//----------------- Generate Program read address ---------------------//
					// 
					if( weights_mode == TREE_WEIGHTS_PROG ) begin 
						um_tx_rd_tag      <= {4'b0101};                  // {process mode, prog mode, features index, weights}
						um_tx_rd_addr     <= tree_weights_base + tree_weights_addr;     // 58-bits

						// TODO: Make trees address jump to next tree in hybrid mode
						tree_weights_addr <= tree_weights_addr + 1'b1;                  // 32-bit

						if(tree_weights_addr == total_trees_weights_numcls_minus_one) begin 
							weights_mode <= TREE_FEATURE_INDEX_PROG;
						end
					end
					else begin 
						um_tx_rd_tag            <= {4'b0110};
						um_tx_rd_addr           <= tree_feature_index_base + tree_feature_index_addr;
						tree_feature_index_addr <= tree_feature_index_addr + 1'b1;

						if(tree_feature_index_addr == total_trees_feature_index_numcls_minus_one) begin 
							weights_mode <= TREE_WEIGHTS_PROG;

							program_read_done <= 1'b1;
						end
					end
				end
				else if(um_tx_rd_ready) begin
					um_tx_rd_valid   <= 0;
				end

				//- Initializing schedules shifter
				if (InDataFIFO_valid_out) begin   // tree weights
					init_w   <= init_w   | InDataFIFO_dout[515];
					init_idx <= init_idx | ~InDataFIFO_dout[515];
				end

				// weighting for last trees to reach destination pu
				if(receiving_program_done & InDataFIFO_empty) begin 
					last_tree_latency_count <=  last_tree_latency_count + 1'b1;
				end

			end
			PROCESS_MODE: begin 
				if(process_done) begin 
					engine_fsm_state <= ENGINE_DONE;
					um_tx_rd_valid   <= 1'b0;
				end
				else if(um_tx_rd_ready) begin 
					if(~data_read_done) begin
						um_tx_rd_tag   <= {4'b1000};
						um_tx_rd_valid <= 1'b1;
						um_tx_rd_addr  <= data_features_base + features_addr;
						features_addr  <= features_addr + 1'b1;

						if(features_addr == total_features_numcls_minus_one) begin 
							data_read_done <= 1'b1;
						end
					end
					else begin
						um_tx_rd_valid   <= 1'b0;
					end
				end 

				//- Initializing schedules shifter
				if (InDataFIFO_valid_out) begin   
					init_p   <= 1'b1;
				end

				//- count sent output cls
				if(um_tx_wr_valid & um_tx_wr_ready) begin 
					num_written_cls <= num_written_cls + 1'b1;
				end
			end
			ENGINE_DONE: begin 
				engine_fsm_state <= IDLE;
				um_done          <= 1'b1;
			end
		endcase
	end
end

assign receiving_program_done    = (received_weights_cls == total_trees_weights_numcls) & (received_findexes_cls == total_trees_feature_index_numcls);
assign distributing_program_done = last_tree_latency_count == WAIT_CYCLES_FOR_LAST_TREE;

assign program_done              = distributing_program_done & receiving_program_done;

assign process_done              = num_written_cls == total_output_numcls;

assign engine_done               = engine_fsm_state == ENGINE_DONE;


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////           Data Receive Section              /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// count received cls to check if full tree weights, or full tree indexes or full tuple has been received.
always @(posedge clk) begin
	if(~rst_n) begin
		received_cl_count     <= 0;
		received_weights_cls  <= 0;
		received_findexes_cls <= 0;
		received_tuples_cls   <= 0;
	end 
	else if( um_rx_rd_valid & um_rx_rd_ready ) begin
		if( um_rx_rd_tag[0] &  um_rx_rd_tag[2] ) begin 
			if(single_tree_weights_received) begin 
				received_cl_count <= 0;
			end
			else begin 
				received_cl_count <= received_cl_count + 1'b1;
			end

			received_weights_cls <= received_weights_cls + 1'b1;
		end
		else if( um_rx_rd_tag[1] &  um_rx_rd_tag[2] ) begin 
			if(single_tree_feature_indexes_received) begin 
				received_cl_count <= 0;
			end
			else begin 
				received_cl_count <= received_cl_count + 1'b1;
			end

			received_findexes_cls <= received_findexes_cls + 1'b1;
		end
		else if(um_rx_rd_tag[3]) begin
			if(single_tuple_features_received) begin 
				received_cl_count <= 0;
			end
			else begin 
				received_cl_count <= received_cl_count + 1'b1;
			end

			received_tuples_cls <= received_tuples_cls + 1'b1;
		end
	end
end


// detect full tree/tuple received.
assign single_tree_weights_received         = received_cl_count == tree_weights_numcls_minus_one;
assign single_tree_feature_indexes_received = received_cl_count == tree_feature_index_numcls_minus_one;
assign single_tuple_features_received       = received_cl_count == tuple_numcls_minus_one;

assign data_last_flag      = (um_rx_rd_tag[3])? single_tuple_features_received : 
                             (um_rx_rd_tag[0])? single_tree_weights_received   : single_tree_feature_indexes_received;

assign um_rx_rd_ready      = ~InDataFIFO_full;


quick_fifo  #(.FIFO_WIDTH(512 + 1 + 1 + 2),     // data + data valid flag + last flag + prog flags        
              .FIFO_DEPTH_BITS(9),
              .FIFO_ALMOSTFULL_THRESHOLD(508)
      ) InDataFIFO (
        .clk                (clk),
        .reset_n            (rst_n),
        .din                ({um_rx_rd_tag[0], um_rx_rd_tag[2], data_last_flag, um_rx_rd_tag[3], um_rx_data}),
        .we                 (um_rx_rd_valid),

        .re                 (InDataFIFO_re),
        .dout               (InDataFIFO_dout),
        .empty              (InDataFIFO_empty),
        .valid              (InDataFIFO_valid_out),
        .full               (InDataFIFO_full),
        .count              (),
        .almostfull         ()
    );

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////         Distributing Received Data          /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    We read data from the input FIFO and redistribute it to the clusters. The mapping of 
    trees/tuples to clusters is dictated by "prog_schedule, and "proc_schedule". 
*/

RLS  #(.DATA_WIDTH(`NUM_DTPU_CLUSTERS),
	   .DATA_WIDTH_BITS(`NUM_DTPU_CLUSTERS_BITS)
	) schedule_shifter(
	.clk            (clk),
	.rst_n          (rst_n),
	.shift_enable   (shift_enable),
	.data_in        (schedule_to_shift),
	.shift_count    (shift_count),
	.data_out       (shifted_schedule)
   );

always @(*) begin
	case (engine_fsm_state)
		PROG_MODE: begin 
			shift_enable = InDataFIFO_valid_out & (data_line_part & InDataFIFO_dout[513]) & (num_trees_sent_to_cluster == `NUM_PUS_PER_CLUSTER-1);

			shift_count  = {{(`NUM_DTPU_CLUSTERS_BITS-1){1'b0}}, 1'b1};

			if(InDataFIFO_valid_out) begin 
				if( (init_w & InDataFIFO_dout[515]) | (init_idx & ~InDataFIFO_dout[515]) ) begin 
					schedule_to_shift = shifted_schedule;
				end
				else begin 
					schedule_to_shift = prog_schedule;
				end
			end
			else begin 
				schedule_to_shift = shifted_schedule;
			end

			
		end
		PROCESS_MODE: begin 
			shift_enable = InDataFIFO_valid_out & (data_line_part & InDataFIFO_dout[513]);

			shift_count  = num_clusters_per_tuple[`NUM_DTPU_CLUSTERS_BITS-1:0];

			if(init_p) begin
				schedule_to_shift = shifted_schedule;
			end
			else begin
				schedule_to_shift = proc_schedule;
			end
		end
		default: begin 
			shift_enable      = 0;
			shift_count       = 0;
			schedule_to_shift = shifted_schedule;
		end
	endcase
end

// Read & Split lines

always @(posedge clk) begin
	if (~rst_n) begin
		data_line_distr_valid[0][0]  <= 0;
		data_line_distr_last[0][0]   <= 0;
		data_line_distr_ctrl[0][0]   <= 0;
		data_line_distr_prog[0][0]   <= 0;
		data_line_distr_en[0][0]     <= 0;

		data_line_part            <= 0;
		curr_pu 				  <= 0;
		num_trees_sent_to_cluster <= 0;
	end
	else begin 
		data_line_distr_ctrl[0][0]  <= params_valid & (engine_fsm_state == IDLE);

		if (InDataFIFO_valid_out & (target_clusters_ready | InDataFIFO_dout[514])) begin

			data_line_part              <= ~data_line_part;

			data_line_distr_valid[0][0] <= InDataFIFO_dout[512]; 
			data_line_distr_last[0][0]  <= InDataFIFO_dout[513] & data_line_part;
			data_line_distr_prog[0][0]  <= InDataFIFO_dout[515:514];
			
			data_line_distr_pu[0][0]    <= curr_pu;
			data_line_distr_en[0][0]    <= schedule_to_shift;

			if(InDataFIFO_dout[513]  & data_line_part) begin
				curr_pu <= curr_pu + 1'b1;
			end
		
			// if this is programming data then we count how many trees we send to a cluster
			if(InDataFIFO_dout[514] & InDataFIFO_dout[513] & data_line_part) begin
				if(num_trees_sent_to_cluster == `NUM_PUS_PER_CLUSTER-1) begin 
					num_trees_sent_to_cluster <= 0;
				end
				else begin 
					num_trees_sent_to_cluster <= num_trees_sent_to_cluster + 1'b1;
				end
			end
		end
		else begin
			data_line_distr_valid[0][0] <= 0;
			data_line_distr_last[0][0]  <= 0;
			data_line_distr_prog[0][0]  <= 0;
		end
	end
end

// select which part of the cache line to distribute
always @(posedge clk) begin
	if(engine_fsm_state == IDLE) begin 
		data_line_distr[0][0] <= {152'b0, {tuple_numcls[14:0], 1'b0}, missing_value, tree_feature_index_numcls, tree_weights_numcls, {4'b0, num_levels_per_tree_minus_one}, {7'b0, partial_mode}, num_trees_per_pu_minus_one};
	end
	else if( ~data_line_part ) begin
		data_line_distr[0][0] <= InDataFIFO_dout[255:0];
	end
	else begin
		data_line_distr[0][0] <= InDataFIFO_dout[511:256];
	end
end

assign InDataFIFO_re = data_line_part & ((InDataFIFO_valid_out & InDataFIFO_dout[514]) | target_clusters_ready);

assign target_clusters_ready = |(clusters_ready & schedule_to_shift);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////        Data Line Distribution Tree          /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

genvar i, j;
generate
for ( i = 0; i < DATA_LINE_DISTR_LEVELS; i=i+1) begin: DL1
  	for( j = 0; j < (1<<(i+1)); j = j+1) begin:DL2
  		always @(posedge clk) begin
			data_line_distr[i+1][j]       <= data_line_distr[i][j>>1];
		end

		always @(posedge clk) begin
			if(~rst_n) begin
				data_line_distr_valid[i+1][j] <= 0;
				data_line_distr_last[i+1][j]  <= 0;
				data_line_distr_ctrl[i+1][j]  <= 0;
				data_line_distr_prog[i+1][j]  <= 0;
				data_line_distr_pu[i+1][j]    <= 0;
				data_line_distr_en[i+1][j]    <= 0;
			end
			else begin 
				data_line_distr_valid[i+1][j] <= data_line_distr_valid[i][j>>1];
				data_line_distr_last[i+1][j]  <= data_line_distr_last[i][j>>1];
				data_line_distr_ctrl[i+1][j]  <= data_line_distr_ctrl[i][j>>1];
				data_line_distr_prog[i+1][j]  <= data_line_distr_prog[i][j>>1];
				data_line_distr_pu[i+1][j]    <= data_line_distr_pu[i][j>>1];
				data_line_distr_en[i+1][j]    <= data_line_distr_en[i][j>>1];
			end
			
		end
  	end

end
endgenerate

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////              Engine Clusters                /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
generate
	for (i = 0; i < `NUM_DTPU_CLUSTERS; i = i + 1)  begin: clusters
		DTPUCluster cluster_x(

		.clk                                (clk),
		.rst_n                              (rst_n),

		.data_line_in                       (data_line_array[i]),
		.data_line_in_valid                 (data_line_valid_array[i]),
		.data_line_in_last                  (data_line_last_array[i]),
		.data_line_in_ctrl                  (data_line_ctrl_array[i]),
		.data_line_in_prog                  (data_line_prog_array[i]),
		.data_line_in_pu                    (data_line_pu_array[i]),
		.data_line_in_ready                 (data_line_ready_array[i]),

		.partial_tree_node_index_out        (pu_tree_node_index_out[i]),
		.partial_tree_node_index_out_valid  (pu_tree_node_index_out_valid[i]),

		.partial_aggregation_out            (partial_aggregation_out[i]),
		.partial_aggregation_out_valid      (partial_aggregation_out_valid[i]),	
		.partial_aggregation_out_ready      (partial_aggregation_out_ready[i])	
		);

	assign data_line_array[i]       = data_line_distr[DATA_LINE_DISTR_LEVELS][i];
	assign data_line_valid_array[i] = data_line_distr_valid[DATA_LINE_DISTR_LEVELS][i] & data_line_distr_en[DATA_LINE_DISTR_LEVELS][i][i];
	assign data_line_last_array[i]  = data_line_distr_last[DATA_LINE_DISTR_LEVELS][i]  & data_line_distr_en[DATA_LINE_DISTR_LEVELS][i][i];
	assign data_line_ctrl_array[i]  = data_line_distr_ctrl[DATA_LINE_DISTR_LEVELS][i];
	assign data_line_prog_array[i]  = data_line_distr_prog[DATA_LINE_DISTR_LEVELS][i]  & {2{data_line_distr_en[DATA_LINE_DISTR_LEVELS][i][i]}};
	assign data_line_pu_array[i]    = data_line_distr_pu[DATA_LINE_DISTR_LEVELS][i];

	assign partial_aggregation_out_ready[i] = aggregator_ready & (curr_cluster == i);

	assign clusters_ready[i]        = data_line_ready_array[i];
 end
endgenerate

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////        Tree Index Output Collector          /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

generate 
	if(`NUM_DTPU_CLUSTERS == 8) begin 
		//------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

		assign partial_tree_node_indexes_CL1       = {pu_tree_node_index_out[3], pu_tree_node_index_out[2], pu_tree_node_index_out[1], pu_tree_node_index_out[0]};                          // Assume 8 PUs per cluster

		assign partial_tree_node_indexes_CL1_valid = pu_tree_node_index_out_valid[0] | pu_tree_node_index_out_valid[1] | pu_tree_node_index_out_valid[2] | pu_tree_node_index_out_valid[3];

		assign partial_tree_node_indexes_CL2       = {pu_tree_node_index_out[7], pu_tree_node_index_out[6], pu_tree_node_index_out[5], pu_tree_node_index_out[4]};                          // Assume 8 PUs per cluster

		assign partial_tree_node_indexes_CL2_valid = pu_tree_node_index_out_valid[4] | pu_tree_node_index_out_valid[5] | pu_tree_node_index_out_valid[6] | pu_tree_node_index_out_valid[7];

        //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
        quick_fifo  #(.FIFO_WIDTH(512),        
              		  .FIFO_DEPTH_BITS(9),
                      .FIFO_ALMOSTFULL_THRESHOLD(508)
                     ) tree_index_out_fifo1 (
        	.clk                (clk),
        	.reset_n            (rst_n),
        	.din                (partial_tree_node_indexes_CL1),
        	.we                 (partial_tree_node_indexes_CL1_valid),

        	.re                 (tree_index_out_fifo1_re),
        	.dout               (tree_index_out_fifo1_dout),
        	.empty              (),
        	.valid              (tree_index_out_fifo1_valid),
        	.full               (),
        	.count              (),
        	.almostfull         ()
    	);
        //
        quick_fifo  #(.FIFO_WIDTH(512),        
              		  .FIFO_DEPTH_BITS(9),
                      .FIFO_ALMOSTFULL_THRESHOLD(508)
                     ) tree_index_out_fifo2 (
        	.clk                (clk),
        	.reset_n            (rst_n),
        	.din                (partial_tree_node_indexes_CL2),
        	.we                 (partial_tree_node_indexes_CL2_valid),

        	.re                 (tree_index_out_fifo2_re),
        	.dout               (tree_index_out_fifo2_dout),
        	.empty              (),
        	.valid              (tree_index_out_fifo2_valid),
        	.full               (),
        	.count              (),
        	.almostfull         ()
    	);
	end
	else if(`NUM_DTPU_CLUSTERS == 4) begin 
		//------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
		assign partial_tree_node_indexes_CL1       = {pu_tree_node_index_out[3], pu_tree_node_index_out[2], pu_tree_node_index_out[1], pu_tree_node_index_out[0]};                          // Assume 8 PUs per cluster

		assign partial_tree_node_indexes_CL1_valid = pu_tree_node_index_out_valid[0] | pu_tree_node_index_out_valid[1] | pu_tree_node_index_out_valid[2] | pu_tree_node_index_out_valid[3];

		//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
        quick_fifo  #(.FIFO_WIDTH(512),        
              		  .FIFO_DEPTH_BITS(9),
                      .FIFO_ALMOSTFULL_THRESHOLD(508)
                     ) tree_index_out_fifo1 (
        	.clk                (clk),
        	.reset_n            (rst_n),
        	.din                (partial_tree_node_indexes_CL1),
        	.we                 (partial_tree_node_indexes_CL1_valid),

        	.re                 (tree_index_out_fifo1_re),
        	.dout               (tree_index_out_fifo1_dout),
        	.empty              (),
        	.valid              (tree_index_out_fifo1_valid),
        	.full               (),
        	.count              (),
        	.almostfull         ()
    	);

        assign tree_index_out_fifo2_valid = 1'b0;
	end
endgenerate


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////            Tree Leafs Aggregation           /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


//---------------- Further aggregating leaf values from multiple clusters --------------------//

assign curr_cluster       = tuple_cluster_base + tuple_cluster_offset;

assign curr_cluster_valid = partial_aggregation_out_valid[curr_cluster];

FPAggregator #(.FP_ADDER_LATENCY(FP_ADDER_LATENCY)) 

 cluster_aggregator(

		.clk                (clk),
		.rst_n              (rst_n),

		.fp_in              (partial_leaf_aggreg_value),
		.fp_in_valid        (partial_leaf_aggreg_value_valid),
		.fp_in_last         (partial_leaf_aggreg_value_last),
		.fp_in_ready        (aggregator_ready),

		.aggreg_out         (tuple_out_data),
		.aggreg_out_valid   (tuple_out_data_valid),
		.aggreg_out_ready   ()
	);

always @(posedge clk) begin
	if(~rst_n) begin
		partial_leaf_aggreg_value       <= 0;
		partial_leaf_aggreg_value_valid <= 0;
		partial_leaf_aggreg_value_last  <= 1'b0;
		tuple_cluster_offset            <= 0;
		tuple_cluster_base              <= 0;
		partial_leaf_aggreg_value_last  <= 0;
	end 
	else begin

		//---------------------- Select partial aggregation value from a cluster ----------------------------//
		if(aggregator_ready) begin
			partial_leaf_aggreg_value       <= partial_aggregation_out[curr_cluster];
			partial_leaf_aggreg_value_valid <= curr_cluster_valid;
			partial_leaf_aggreg_value_last  <= 1'b0;

			if(curr_cluster_valid) begin
				if(tuple_cluster_offset == (num_clusters_per_tuple_minus_one) ) begin 
					tuple_cluster_offset <= 0;

					if(num_clusters_per_tuple == `NUM_DTPU_CLUSTERS) begin 
						tuple_cluster_base   <= 0;
					end
					else begin 
						tuple_cluster_base   <= tuple_cluster_base + num_clusters_per_tuple[`NUM_DTPU_CLUSTERS_BITS-1:0];
					end

					partial_leaf_aggreg_value_last <= 1'b1;
				end
				else begin 
					tuple_cluster_offset <= tuple_cluster_offset + 1'b1;
				end
			end
		end
	end
end
//--------------------- Filling a cache line of tuple output data -------------------------//

// write aggregated value to correct position in cacheline
always @(posedge clk) begin
	if (tuple_out_data_valid) begin
		tree_leaf_aggreg_dout_array[tuple_pos] <= tuple_out_data;
	end
end

// tuple_pos in cacheline
always @(posedge clk) begin
	if (~rst_n) begin
		tree_leaf_aggreg_valid            <= 1'b0;
		tree_leaf_aggreg_dout_array_valid <= 1'b0; 
		tuple_pos                         <= 0;
	end
	else begin
		tree_leaf_aggreg_valid <= tree_leaf_aggreg_dout_array_valid;

		if (tuple_out_data_valid) begin
			tuple_pos  <= tuple_pos + 1'b1;

			if (tuple_pos == 4'b1111) begin
				tree_leaf_aggreg_dout_array_valid    <= 1'b1; 
			end
			else begin
				tree_leaf_aggreg_dout_array_valid    <= 1'b0; 
			end
		end
		else begin
			tree_leaf_aggreg_dout_array_valid    <= 1'b0;
		end
	end
end

generate
	for(i = 0; i < 16; i = i + 1) begin: tupleAggregations
		always @(posedge clk) begin
			tree_leaf_aggreg_dout[i*32+31:i*32] <= tree_leaf_aggreg_dout_array[i];
		end
	end
endgenerate

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////                            //////////////////////////////////
//////////////////////////////            Output Write Section             /////////////////////////
//////////////////////////////////////                            //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

assign tree_index_out_fifo1_re   = um_tx_wr_ready & ~pick_cl_id;
assign tree_index_out_fifo2_re   = um_tx_wr_ready & pick_cl_id;

assign tree_leaf_aggreg_valid_re = um_tx_wr_ready & ~partial_mode;

always @(posedge clk) begin
	if(um_tx_wr_ready) begin
		um_tx_wr_addr  <= dt_out_base_addr + curr_out_addr;
		if( partial_mode ) begin 
			if( pick_cl_id == 1'b0 ) begin 
				um_tx_data <= tree_index_out_fifo1_dout;
			end
			else begin
				um_tx_data <= tree_index_out_fifo2_dout;
			end
		end
		else begin 
			um_tx_data     <= tree_leaf_aggreg_dout;
		end
	end
end

always @(posedge clk) begin
	if(~rst_n) begin
		um_tx_wr_valid       <= 0;
		curr_out_addr        <= 0;
		pick_cl_id           <= 1'b0;
		num_written_tree_cls <= 0;
	end 
	else if(um_tx_wr_ready) begin
		um_tx_wr_valid <= tree_index_out_fifo1_valid | tree_index_out_fifo2_valid | tree_leaf_aggreg_valid;

		if(tree_index_out_fifo1_valid | tree_index_out_fifo2_valid | tree_leaf_aggreg_valid) begin 
			curr_out_addr <= curr_out_addr + 1'b1;
		end

		if( partial_mode ) begin 
			if( pick_cl_id == 1'b0 ) begin 
				if(tree_index_out_fifo1_valid) begin
					if(num_written_tree_cls == num_trees_index_out_cls_minus_one) begin 
						pick_cl_id           <= 1'b0;
						num_written_tree_cls <= 0;
					end
					else begin 
						if(`NUM_DTPU_CLUSTERS == 8) begin 
							pick_cl_id           <= 1'b1;
						end
						
						num_written_tree_cls <= num_written_tree_cls + 1'b1;
					end
				end
			end
			else begin
				if(tree_index_out_fifo2_valid) begin
					pick_cl_id           <= 1'b0;
					num_written_tree_cls <= num_written_tree_cls + 1'b1;
				end
			end
		end
	end
end

endmodule

