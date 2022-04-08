/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES.
 * All rights reserved.
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

#ifndef __PRODUCER_CONSUMER_H__
#define __PRODUCER_CONSUMER_H__

#include <systemc.h>
#include <nvhls_connections.h>
#include <nvhls_packet.h>

#include <arbitrated_crossbar.h>
#include <interconnect/Interconnect.hpp>
#include "interconnect_config.hpp"

#ifdef INTERCONNECT_GEN
#include ic_header(INTERCONNECT_GEN)
#endif  // ifdef INTERCONNECT_GEN

#include "ProducerConsumerPart.hpp"

class ProducerConsumerArray : public match::Module {
 public:
  SC_HAS_PROCESS(ProducerConsumerArray);
  static const int kPEWidth = PE_WIDTH;
  static const int kPEHeight = PE_HEIGHT;

  ProducerConsumerPart *pe_part_inst[kPEWidth * kPEHeight];
  IC_HAS_INTERCONNECT(pcmodulearray);

  ProducerConsumerArray(sc_module_name nm) : match::Module(nm) {
    ic.clk(clk);
    ic.rst(rst);

    for (int i = 0; i < kPEWidth * kPEHeight; i++) {
      pe_part_inst[i] =
          new ProducerConsumerPart(sc_gen_unique_name("pe_part_inst"), i);

      pe_part_inst[i]->clk(clk);
      pe_part_inst[i]->rst(rst);
      IC_PART_BIND(*pe_part_inst[i], i);
    }
  }
};

#endif
