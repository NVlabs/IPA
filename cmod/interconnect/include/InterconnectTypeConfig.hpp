/*
 * SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION &
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

#ifndef __INTERCONNECT_TYPE_CONFIG_H__
#define __INTERCONNECT_TYPE_CONFIG_H__

#include <systemc.h>
#include <nvhls_connections.h>
#include "InterconnectChannels.hpp"
#include <tuple>

namespace interconnect {

#ifdef CONNECTIONS_SIM_ONLY

template <typename IM>
struct InterconnectTypeConfig {
  typedef IM IM_t;

  typedef Connections::OutBlocking<IM> IM_src_t;
  typedef Connections::InBlocking<IM> IM_dest_t;
  typedef Connections::Combinational<IM> IM_chan_t;
  typedef interconnect::CombinationalLink<IM> IM_chanlink_t;

  typedef std::vector<IM_src_t*> srcs_t;
  typedef std::vector<IM_dest_t*> dests_t;

  typedef std::map<IM_src_t*, IM_chan_t*> channels_begin_t;
  typedef std::map<IM_dest_t*, IM_chanlink_t*> channels_middle_inner_t;
  typedef std::map<IM_src_t*, channels_middle_inner_t> channels_middle_t;
  typedef std::map<IM_dest_t*, IM_chan_t*> channels_end_t;

  typedef std::map<IM_src_t*, std::size_t> srcs_typeid_t;
  typedef std::map<IM_dest_t*, std::size_t> dests_typeid_t;
  typedef std::map<IM_src_t*, int> srcs_user_typeid_t;
  typedef std::map<IM_dest_t*, int> dests_user_typeid_t;
  typedef std::map<IM_src_t*, unsigned int> srcs_width_t;
  typedef std::map<IM_dest_t*, unsigned int> dests_width_t;

  typedef std::vector<IM_src_t*> channels_valid_pairs_reverse_inner_t;
  typedef std::map<IM_dest_t*, channels_valid_pairs_reverse_inner_t>
      channels_valid_pairs_reverse_t;

  typedef NVUINT32 idx_t;
  typedef NVUINT32 dest_map_idx_t;
  typedef std::map<dest_map_idx_t, std::map<idx_t, idx_t> > dest_maps_t;
  typedef std::vector<dest_map_idx_t> dest_maps_idx_vec_t;

  typedef NVUINT64 cycles_count_t;
  typedef std::map<IM_src_t*, std::map<IM_dest_t*, cycles_count_t> >
      cycle_count_channel_t;
};

#endif
};

#endif  // __INTERCONNECT_TYPE_CONFIG_H__
