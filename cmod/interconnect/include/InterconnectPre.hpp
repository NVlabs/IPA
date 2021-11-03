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

#ifndef __INTERCONNECT_PRE_HPP__
#define __INTERCONNECT_PRE_HPP__

////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace interconnect {

// Declaration
template <typename MSG_TYPE>
class Msg;

template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
class InterconnectMessageSCLV;

template <unsigned int IC_ID = 0>
class Interconnect;

template <typename IM>
class InterconnectBase;
};  // namespace interconnect

template <typename PART_TYPE,
          Connections::connections_port_t PortType = AUTO_PORT>
class InterconnectInterface;

namespace interconnect {
enum { MaxDataWidth = 1500, MaxDestCount = 4096 };

// Internal generic interconnect message type
typedef InterconnectMessageSCLV<interconnect::MaxDataWidth,
                                interconnect::MaxDestCount> IM_t;

typedef unsigned int MSG_ID_t;
typedef unsigned int width_t;

#ifdef CONNECTIONS_SIM_ONLY

struct DestDirectoryEntry {
 protected:
  std::vector<unsigned int> dests;
  unsigned int counter;  // most recently used counter
  unsigned int total;

 public:
  DestDirectoryEntry() : counter(0), total(0){};

  void add(const unsigned int &i) {
    dests.push_back(i);
    total++;
    NVHLS_ASSERT(total == dests.size());
  }

  unsigned int get_next() {
    NVHLS_ASSERT(total == dests.size());
    NVHLS_ASSERT(total > 0);
    NVHLS_ASSERT(counter < total);

    unsigned int ret_val = dests[counter];
    counter++;
    if (counter >= total)
      counter = 0;
    return ret_val;
  }
};

class ICManager {
 public:
  std::map<unsigned int, InterconnectBase<IM_t> *> registered_ic;
  std::map<std::tuple<unsigned int, unsigned int, unsigned int>,
           DestDirectoryEntry> dest_directory;
};

template <class Dummy>
struct ICManager_statics {
  static ICManager icm;
};

inline ICManager &get_ICManager() {
  return ICManager_statics<void>::icm;
}

template <class Dummy>
ICManager ICManager_statics<Dummy>::icm;

#endif  // CONNECTIONS_SIM_ONLY

// Forward declarations
template <class Dummy>
unsigned int get_real_dest_idx(InterconnectBase<IM_t> &ic,
                               const unsigned int &dest_idx,
                               const unsigned int &dest_map_idx);

// Destination struct
struct Destination : public nvhls_message {
  NVUINT32 part_id;
  NVUINT32 port_id;

  explicit Destination(unsigned int part_id_ = 0, unsigned int port_id_ = 0) {
    part_id = part_id_;
    port_id = port_id_;
  }

  // Marshaller
  enum {
    width =
        Wrapped<decltype(part_id)>::width + Wrapped<decltype(port_id)>::width
  };

  template <unsigned int Size>
  void Marshall(Marshaller<Size> &m) {
    m &part_id;
    m &port_id;
  }
};
};

#endif  // ifndef __INTERCONNECT_PRE_HPP__
