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
// Design Name:    Prefetcher Controller SVA                                  //
// Project Name:   CV32E40P                                                   //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    SV Properties, Assertions, etc, for the CV32E40P           //
//                 Sleep Unit.                                                //
////////////////////////////////////////////////////////////////////////////////

module cv32e40p_sleep_unit_sva
#(
  parameter PULP_CLUSTER = 0
)(
  // Clock, reset interface
  input logic        clk_ungated_i,            // Free running clock
  input logic        rst_n,
  input logic        clk_gated_o,              // Gated clock
  input logic        scan_cg_en_i,             // Enable all clock gates for testing

  // Core sleep
  input logic        core_sleep_o,

  // Fetch enable
  input logic        fetch_enable_i,
  input logic        fetch_enable_o,

  // Core status
  input logic        if_busy_i,
  input logic        ctrl_busy_i,
  input logic        lsu_busy_i,
  input logic        apu_busy_i,

  // PULP Cluster interface
  input logic        pulp_clock_en_i,          // PULP clock enable (only used if PULP_CLUSTER = 1)
  input logic        p_elw_start_i,
  input logic        p_elw_finish_i,
  input logic        debug_p_elw_no_sleep_i,

  // WFI wake
  input logic        wake_from_sleep_i,

  // Internal signals used by the assertions
  input logic        clock_en,
  input logic        fetch_enable_q,
  input logic        fetch_enable_d,
  input logic        p_elw_busy_q,
  input logic        p_elw_busy_d,
  input logic        core_busy_q,
  input logic        core_busy_d
);

  import cv32e40p_pkg::*;
  import uvm_pkg::*; // needed for the UVM messaging service (`uvm_error(), etc.)

  // Clock gate is disabled during RESET state of the controller
  property p_clock_en_0;
    @(posedge clk_ungated_i) disable iff (!rst_n) ((id_stage_i.controller_i.ctrl_fsm_cs == cv32e40p_pkg::RESET) &&
                                                   (id_stage_i.controller_i.ctrl_fsm_ns == cv32e40p_pkg::RESET)) |-> (clock_en == 1'b0);
  endproperty

  a_clock_en_0:
    assert property(p_clock_en_0)
    else `uvm_error("Sleep Unit",
          $sformatf("Clock gate not disabled during RESET state of the controller\n"))

  // Clock gate is enabled when exit from RESET state is required
  property p_clock_en_1;
     @(posedge clk_ungated_i) disable iff (!rst_n) ((id_stage_i.controller_i.ctrl_fsm_cs == cv32e40p_pkg::RESET) &&
                                                    (id_stage_i.controller_i.ctrl_fsm_ns != cv32e40p_pkg::RESET)) |-> (clock_en == 1'b1);
  endproperty

  a_clock_en_1:
    assert property(p_clock_en_1)
    else `uvm_error("Sleep Unit",
          $sformatf("Clock gate not enabled when exit from RESET state is required\n"))

  // Clock gate is not enabled before receiving fetch_enable_i pulse
  property p_clock_en_2;
     @(posedge clk_ungated_i) disable iff (!rst_n) (fetch_enable_q == 1'b0) |-> (clock_en == 1'b0);
  endproperty

  a_clock_en_2:
    assert property(p_clock_en_2)
    else `uvm_error("Sleep Unit",
          $sformatf("Clock gate enabled when exit from RESET state is required\n"))

  generate
    if (PULP_CLUSTER) begin

      // Clock gate is only possibly disabled in RESET or when PULP_CLUSTER disables clock
      property p_clock_en_3;
         @(posedge clk_ungated_i) disable iff (!rst_n) (clock_en == 1'b0) -> ((id_stage_i.controller_i.ctrl_fsm_cs == cv32e40p_pkg::RESET) ||
                                                                              (PULP_CLUSTER && !pulp_clock_en_i));
      endproperty

      a_clock_en_3:
        assert property(p_clock_en_3)
        else `uvm_error("Sleep Unit",
              $sformatf("Error: clock gate should only be disabled in RESET or when PULP_CLUSTER disables clock\n"))

    // Core can only sleep in response to p.elw
    property p_only_sleep_during_p_elw;
       @(posedge clk_ungated_i) disable iff (!rst_n) (core_sleep_o == 1'b1) |-> (p_elw_busy_d == 1'b1);
    endproperty

    a_only_sleep_during_p_elw:
      assert property(p_only_sleep_during_p_elw)
      else `uvm_error("Sleep Unit",
            $sformatf("Core can only sleep in response to p.elw, p_elw_busy_d should be asserted\n"))

    // Environment fully controls clock_en during sleep
    property p_full_clock_en_control;
       @(posedge clk_ungated_i) disable iff (!rst_n) (core_sleep_o == 1'b1) |-> (pulp_clock_en_i == clock_en);
    endproperty

    a_full_clock_en_control:
    assert property(p_full_clock_en_control)
    else `uvm_error("Sleep Unit",
          $sformatf("Error: clock_en should be fully controlled by environment (pulp_clock_en_i) during sleep\n"))

    end else begin

      // Clock gate is only possibly disabled in RESET or SLEEP
      property p_clock_en_4;
        @(posedge clk_ungated_i) disable iff (!rst_n) (clock_en == 1'b0) -> ((id_stage_i.controller_i.ctrl_fsm_cs == cv32e40p_pkg::RESET) ||
                                                                             (id_stage_i.controller_i.ctrl_fsm_ns == cv32e40p_pkg::SLEEP));
      endproperty

      a_clock_en_4:
        assert property(p_clock_en_4)
        else `uvm_error("Sleep Unit",
              $sformatf("Clock gate can be only disabled in RESET or SLEEP\n"))

      // Clock gate is enabled when exit from SLEEP state is required
      property p_clock_en_5;
         @(posedge clk_ungated_i) disable iff (!rst_n)  ((id_stage_i.controller_i.ctrl_fsm_cs == cv32e40p_pkg::SLEEP) &&
                                                         (id_stage_i.controller_i.ctrl_fsm_ns != cv32e40p_pkg::SLEEP)) |-> (clock_en == 1'b1);
      endproperty

      a_clock_en_5:
        assert property(p_clock_en_5)
        else `uvm_error("Sleep Unit",
              $sformatf("Clock gate must be enabled when exiting from SLEEP state\n"))

      // Core sleep is only signaled in SLEEP state
      property p_core_sleep;
         @(posedge clk_ungated_i) disable iff (!rst_n) (core_sleep_o == 1'b1) -> ((id_stage_i.controller_i.ctrl_fsm_cs == cv32e40p_pkg::SLEEP));
      endproperty

      a_core_sleep:
        assert property(p_core_sleep)
        else `uvm_error("Sleep Unit",
              $sformatf("Core sleep must be only signaled in SLEEP state, current state: %d\n", id_stage_i.controller_i.ctrl_fsm_cs))

      // Core can only become non-busy due to SLEEP entry
      property p_non_busy;
         @(posedge clk_ungated_i) disable iff (!rst_n) (core_busy_d == 1'b0) |-> (id_stage_i.controller_i.ctrl_fsm_cs == cv32e40p_pkg::WAIT_SLEEP) ||
                                                                                 (id_stage_i.controller_i.ctrl_fsm_cs == cv32e40p_pkg::SLEEP);
      endproperty

      a_non_busy:
        assert property(p_non_busy)
        else `uvm_error("Sleep Unit",
              $sformatf("Core can only become non-busy when in SLEEP or WAIT_SLEEP state\n"))

      // During (PULP_CLUSTER = 0) sleep it should be allowed to externally gate clk_i
      property p_gate_clk_i;
        @(posedge clk_ungated_i) disable iff (!rst_n) (core_sleep_o == 1'b1) |-> (core_busy_q == core_busy_d) &&
                                                                                 (p_elw_busy_q == p_elw_busy_d) && (fetch_enable_q == fetch_enable_d);
      endproperty

      a_gate_clk_i:
        assert property(p_gate_clk_i)
        else `uvm_error("Sleep Unit",
              $sformatf("clk_i should be allowed to be externally gated when in SLEEP (PULP_CLUSTER = 0)\n"))

      // During sleep the internal clock is gated
      property p_gate_clock_during_sleep;
        @(posedge clk_ungated_i) disable iff (!rst_n) (core_sleep_o == 1'b1) |-> (clock_en == 1'b0);
      endproperty

      a_gate_clock_during_sleep:
        assert property(p_gate_clock_during_sleep)
        else `uvm_error("Sleep Unit",
              $sformatf("During SLEEP the internal clock must be gated (clock_en == 0)\n"))

      // Sleep mode can only be entered in response to a WFI instruction
      property p_only_sleep_for_wfi;
        @(posedge clk_ungated_i) disable iff (!rst_n) (core_sleep_o == 1'b1) |-> (id_stage_i.instr == { 12'b000100000101, 13'b0, OPCODE_SYSTEM });
      endproperty

      a_only_sleep_for_wfi:
        assert property(p_only_sleep_for_wfi)
        else `uvm_error("Sleep Unit",
              $sformatf("Sleep mode can only be entered in response to a WFI instruction: current id_stage_i.instr = %d\n", id_stage_i.instr))

      // In sleep mode the core will not be busy (e.g. no ongoing/outstanding instruction or data transactions)
      property p_not_busy_during_sleep;
        @(posedge clk_ungated_i) disable iff (!rst_n) (core_sleep_o == 1'b1) |-> ((core_busy_q == 1'b0) && (core_busy_d == 1'b0));
      endproperty

      a_not_busy_during_sleep:
        assert property(p_not_busy_during_sleep)
        else `uvm_error("Sleep Unit",
              $sformatf("In sleep mode the core must not be busy (e.g. no ongoing/outstanding instruction or data transactions)\n"))
    end
  endgenerate

endmodule : cv32e40p_sleep_unit_sva
