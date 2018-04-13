
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

 
`ifndef DT_ENGINE_DEFINES_VH
`define DT_ENGINE_DEFINES_VH

`define   NUM_PUS_PER_CLUSTER_BITS          3
`define   NUM_PUS_PER_CLUSTER               8
`define   NUM_DTPU_CLUSTERS 		        4
`define   NUM_DTPU_CLUSTERS_BITS            2
`define   NUM_TREES_PER_PU                  32

`define   FEATURES_DISTR_DELAY              8

`define   DATA_PRECISION                    32
`define   FIXED_POINT_ARITHMATIC            ((`DATA_PRECISION < 32)? 1 : 0)

`define   DATA_LINE_WIDTH                   ((`DATA_PRECISION == 32)? 256 : (`DATA_PRECISION == 16)? 128 :  64)

`endif
