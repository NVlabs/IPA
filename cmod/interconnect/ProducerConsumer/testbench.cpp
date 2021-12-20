/*
 * SPDX-FileCopyrightText: Copyright (c) 2020-2021 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <systemc.h>
#include <mc_scverify.h>
#include <testbench/nvhls_rand.h>
#include <nvhls_connections.h>
#include <map>
#include <vector>
#include <utility>
#include <sstream>
#include <string>
#include <cstdlib>
#include <map>

#include "interconnect_config.hpp"

#include "ProducerConsumerArray.hpp"

class DUT : public sc_module {
 public:
  sc_in_clk clk;
  sc_in<bool> rst;

  ProducerConsumerArray pcmodulearray_inst;

  DUT(sc_module_name nm)
      : sc_module(nm), pcmodulearray_inst("pcmodulearray_inst") {
    pcmodulearray_inst.clk(clk);
    pcmodulearray_inst.rst(rst);
  }
};

SC_MODULE(testbench) {
 public:
  DUT dut;

  sc_clock clk;
  sc_signal<bool> rst;

  SC_HAS_PROCESS(testbench);
  testbench(sc_module_name name_)
      : sc_module(name_),
        dut("dut"),
        clk("clk", 1.0, SC_NS, 0.5, 0, SC_NS, false),
        rst("rst") {

    Connections::set_sim_clk(&clk);

    dut.clk(clk);
    dut.rst(rst);

    SC_THREAD(run);
  }

  void run() {
    // reset
    rst = 0;
    cout << "@" << sc_time_stamp() << " Asserting Reset " << endl;
    wait(2, SC_NS);
    cout << "@" << sc_time_stamp() << " Deasserting Reset " << endl;
    rst = 1;
    wait(10000.0, SC_NS);
    sc_stop();
  }
};

int sc_main(int argc, char* argv[]) {
  sc_report_handler::set_actions(SC_WARNING, SC_DO_NOTHING);
  testbench my_testbench("my_testbench");

  for (unsigned int i = 0; i < PE_WIDTH * PE_HEIGHT; ++i) {
    get_pc_log().PE_unicast_only[i] = true;
  }

  for (unsigned int i = 0; i < PE_WIDTH * PE_HEIGHT; ++i) {
    get_pc_log().PE_activity[i] = PE_ACTIVITY;

    int i_x_coor = i % PE_WIDTH;
    int i_y_coor = i / PE_WIDTH;
    for (unsigned int j = 0; j < PE_WIDTH * PE_HEIGHT; ++j) {
      if (i == j) {
        get_pc_log().PE_dest_probs[i][j] = 0;
        continue;  // Always skip loopback
      }
      int j_x_coor = j % PE_WIDTH;
      int j_y_coor = j / PE_WIDTH;
      int mdist = abs(i_x_coor - j_x_coor) + abs(i_y_coor - j_y_coor);
      if (PE_LOCAL_ONLY) {
        NVHLS_ASSERT(mdist > 0);
        if (mdist <= 1) {
          get_pc_log().PE_dest_probs[i][j] = 0.5;
        } else {
          get_pc_log().PE_dest_probs[i][j] = 0;
        }
      } else {
        // if(i == 0 && j == 7) {
        get_pc_log().PE_dest_probs[i][j] = 0.5;
        //} else {
        //  get_pc_log().PE_dest_probs[i][j] = 0;
        //}
      }
    }
  }

  sc_report_handler::set_actions(SC_ERROR, SC_DISPLAY);
  sc_start();

  bool rc = (sc_report_handler::get_count(SC_ERROR) > 0);

  // Don't print stats in elab only mode.
  const char* nvhls_elab_only = std::getenv("NVHLS_ELAB_ONLY");
  if (nvhls_elab_only && std::string(nvhls_elab_only) == "1") {
    return rc;
  }

#if !defined(INTERCONNECT_GEN)
  my_testbench.dut.pcmodulearray_inst.ic.pretty_print();
  my_testbench.dut.pcmodulearray_inst.ic.print_statistics();
#endif
  get_pc_log().print_stats();
  get_pc_log().print_pre_stats();

  if (rc)
    cout << "Simulation FAILED\n";
  else
    cout << "Simulation PASSED\n";
  return rc;
};
