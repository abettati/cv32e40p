// Copyright 2020 Silicon Labs, Inc.
//
// This file, and derivatives thereof are licensed under the
// Solderpad License, Version 2.0 (the "License").
//
// Use of this file means you agree to the terms and conditions
// of the license and are in full compliance with the License.
//
// You may obtain a copy of the License at:
//
//     https://solderpad.org/licenses/SHL-2.0/
//
// Unless required by applicable law or agreed to in writing, software
// and hardware implementations thereof distributed under the License
// is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS
// OF ANY KIND, EITHER EXPRESSED OR IMPLIED.
//
// See the License for the specific language governing permissions and
// limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Arjan Bink - arjan.bink@silabs.com                         //
//                                                                            //
// Design Name:    Sleep Unit                                                 //
// Project Name:   CV32E40P                                                   //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Sleep unit containing the instantiated clock gate which    //
//                 provides the gated clock (clk_gated_o) for the rest        //
//                 of the design.                                             //
//                                                                            //
//                 The clock is gated for the following scenarios:            //
//                                                                            //
//                 - While waiting for fetch to become enabled                //
//                 - While blocked on a WFI (PULP_CLUSTER = 0)                //
//                 - While clock_en_i = 0 during a p.elw (PULP_CLUSTER = 1)   //
//                                                                            //
//                 Sleep is signaled via core_sleep_o when:                   //
//                                                                            //
//                 - During a p.elw (except in debug (i.e. pending debug      //
//                   request, debug mode, single stepping, trigger match)     //
//                 - During a WFI (except in debug)                           //
//                                                                            //
// Requirements:   If PULP_CLUSTER = 1 the environment must guarantee:        //
//                                                                            //
//                 - If core_sleep_o    == 1'b0, then pulp_clock_en_i == 1'b1 //
//                 - If pulp_clock_en_i == 1'b0, then irq_i == 'b0            //
//                 - If pulp_clock_en_i == 1'b0, then debug_req_i == 1'b0     //
//                 - If pulp_clock_en_i == 1'b0, then instr_rvalid_i == 1'b0  //
//                 - If pulp_clock_en_i == 1'b0, then instr_gnt_i == 1'b0     //
//                 - If pulp_clock_en_i == 1'b0, then data_rvalid_i == 1'b0   //
//                 - If pulp_clock_en_i == 1'b0, then data_gnt_i == 1'b1      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e40p_sleep_unit
#(
  parameter PULP_CLUSTER = 0
)(
  // Clock, reset interface
  input  logic        clk_ungated_i,            // Free running clock
  input  logic        rst_n,
  output logic        clk_gated_o,              // Gated clock
  input  logic        scan_cg_en_i,             // Enable all clock gates for testing

  // Core sleep
  output logic        core_sleep_o,

  // Fetch enable
  input  logic        fetch_enable_i,
  output logic        fetch_enable_o,

  // Core status
  input  logic        if_busy_i,
  input  logic        ctrl_busy_i,
  input  logic        lsu_busy_i,
  input  logic        apu_busy_i,

  // PULP Cluster interface
  input  logic        pulp_clock_en_i,          // PULP clock enable (only used if PULP_CLUSTER = 1)
  input  logic        p_elw_start_i,
  input  logic        p_elw_finish_i,
  input  logic        debug_p_elw_no_sleep_i,

  // WFI wake
  input  logic        wake_from_sleep_i
);

  import cv32e40p_pkg::*;

  logic              fetch_enable_q;            // Sticky version of fetch_enable_i
  logic              fetch_enable_d;
  logic              core_busy_q;               // Is core still busy (and requires a clock) with what needs to finish before entering sleep?
  logic              core_busy_d;
  logic              p_elw_busy_q;              // Busy with p.elw (transaction in progress)?
  logic              p_elw_busy_d;
  logic              clock_en;                  // Final clock enable

  //////////////////////////////////////////////////////////////////////////////
  // Sleep FSM
  //////////////////////////////////////////////////////////////////////////////

  // Make sticky version of fetch_enable_i
  assign fetch_enable_d = fetch_enable_i ? 1'b1 : fetch_enable_q;

  generate
  if (PULP_CLUSTER) begin : PULP_SLEEP

    // Busy unless in a p.elw and IF/APU are no longer busy
    assign core_busy_d = p_elw_busy_d ? (if_busy_i || apu_busy_i) : 1'b1;

     // Enable the clock only after the initial fetch enable while busy or instructed so by PULP Cluster's pulp_clock_en_i
    assign clock_en = fetch_enable_q && (pulp_clock_en_i || core_busy_q);

    // Sleep only in response to p.elw onec no longer busy (but not during various debug scenarios)
    assign core_sleep_o = p_elw_busy_d && !core_busy_q && !debug_p_elw_no_sleep_i;

    // p.elw is busy between load start and load finish (data_req_o / data_rvalid_i)
    assign p_elw_busy_d = p_elw_start_i ? 1'b1 : (p_elw_finish_i ? 1'b0 : p_elw_busy_q);

  end else begin : NO_PULP_SLEEP

    // Busy when any of the sub units is busy (typically wait for the instruction buffer to fill up)
    assign core_busy_d = if_busy_i || ctrl_busy_i || lsu_busy_i || apu_busy_i;

    // Enable the clock only after the initial fetch enable while busy or waking up to become busy
    assign clock_en = fetch_enable_q && (wake_from_sleep_i || core_busy_q);

    // Sleep only in response to WFI which leads to clock disable; debug_wfi_no_sleep_o in
    // cv32e40p_controller determines the scenarios for which WFI can(not) cause sleep.
    assign core_sleep_o = fetch_enable_q && !clock_en;

    // p.elw does not exist for PULP_CLUSTER = 0
    assign p_elw_busy_d = 1'b0;

  end
  endgenerate

  always_ff @(posedge clk_ungated_i, negedge rst_n)
  begin
    if (rst_n == 1'b0) begin
      core_busy_q    <= 1'b0;
      p_elw_busy_q   <= 1'b0;
      fetch_enable_q <= 1'b0;
    end else begin
      core_busy_q    <= core_busy_d;
      p_elw_busy_q   <= p_elw_busy_d;
      fetch_enable_q <= fetch_enable_d;
    end
  end

  // Fetch enable for Controller
  assign fetch_enable_o = fetch_enable_q;

  // Main clock gate of CV32E40P
  cv32e40p_clock_gate core_clock_gate_i
  (
    .clk_i        ( clk_ungated_i   ),
    .en_i         ( clock_en        ),
    .scan_cg_en_i ( scan_cg_en_i    ),
    .clk_o        ( clk_gated_o     )
  );

endmodule // cv32e40p_sleep_unit
