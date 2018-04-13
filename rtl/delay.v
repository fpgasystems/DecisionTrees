
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
 

module delay #(parameter DATA_WIDTH   = 32,
	           parameter DELAY_CYCLES = 4 
	          ) (

	        input  wire                       clk,
	        input  wire                       rst_n,
	        input  wire  [DATA_WIDTH-1:0]     data_in,
	        input  wire                       data_in_valid,
	        output wire  [DATA_WIDTH-1:0]     data_out,
	        output wire                       data_out_valid
	       );


reg [DATA_WIDTH-1:0]       data_array[DELAY_CYCLES];
reg                        data_array_valid[DELAY_CYCLES];


always @(posedge clk) begin
	// Valid Bit
	if(~rst_n) begin
		data_array_valid[0] <= 0;
	end 
	else begin
		data_array_valid[0] <= data_in_valid;
	end
    // Data word
	data_array[0] <= data_in;
end


genvar i;
generate for (i = 1; i < DELAY_CYCLES; i = i +1) begin: delayPipe
	always @(posedge clk) begin
		// Valid Bit
		if(~rst_n) begin
		 	data_array_valid[i] <= 0;
		end 
		else begin
		 	data_array_valid[i] <= data_array_valid[i-1];
		end
        // Data word
		data_array[i] <= data_array[i-1];
	end
end
endgenerate

assign data_out       = data_array[DELAY_CYCLES-1];
assign data_out_valid = data_array_valid[DELAY_CYCLES-1];

endmodule // delay
