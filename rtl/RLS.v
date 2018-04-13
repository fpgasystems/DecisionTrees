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

 
`include "DTEngine_defines.vh"

module RLS #(parameter  DATA_WIDTH      = 8,
	         parameter  DATA_WIDTH_BITS = 3) 
	( 
	input  wire                            clk,
	input  wire                            rst_n,
	input  wire                            shift_enable,
	input  wire  [DATA_WIDTH-1:0]          data_in,
	input  wire  [DATA_WIDTH_BITS-1:0]     shift_count,

	output wire  [DATA_WIDTH-1:0]          data_out
   );

reg  [DATA_WIDTH-1:0]   shifted_data;

assign data_out = shifted_data;

generate
	if(`NUM_DTPU_CLUSTERS == 8) begin
		always @(posedge clk) begin
			if(shift_enable) begin 
				case (shift_count)
					3'b001: begin 
						shifted_data[0]              <= data_in[DATA_WIDTH-1];
						shifted_data[DATA_WIDTH-1:1] <= data_in[DATA_WIDTH-2:0];
					end
					3'b010: begin 
						shifted_data[1:0]            <= data_in[DATA_WIDTH-1:DATA_WIDTH-2];
						shifted_data[DATA_WIDTH-1:2] <= data_in[DATA_WIDTH-3:0];
					end
					3'b100: begin 
						shifted_data[3:0]            <= data_in[DATA_WIDTH-1:DATA_WIDTH-4];
						shifted_data[DATA_WIDTH-1:4] <= data_in[DATA_WIDTH-5:0];
					end
					default: begin 
						shifted_data <= data_in;
					end
				endcase
			end
			else begin 
				shifted_data <= data_in;
			end
		end
	end 
	else begin
		always @(posedge clk) begin
			if(shift_enable) begin 
				case (shift_count)
					2'b01: begin 
						shifted_data[0]              <= data_in[DATA_WIDTH-1];
						shifted_data[DATA_WIDTH-1:1] <= data_in[DATA_WIDTH-2:0];
					end
					2'b10: begin 
						shifted_data[1:0]            <= data_in[DATA_WIDTH-1:DATA_WIDTH-2];
						shifted_data[DATA_WIDTH-1:2] <= data_in[DATA_WIDTH-3:0];
					end
					default: begin 
						shifted_data <= data_in;
					end
				endcase
			end
			else begin 
				shifted_data <= data_in;
			end
		end
	end 
	
endgenerate
	

endmodule