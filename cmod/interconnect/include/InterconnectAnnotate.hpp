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

// Reimplementation of Connections back annotation, extended for congestion
// modeling support.
//
// See https://github.com/hlslibs/matchlib_connections for the original
// annotate.h.
//

//========================================================================
// InterconnectAnnotate.h
//========================================================================

#ifndef __INTERCONNECT_ANNOTATE_HPP__
#define __INTERCONNECT_ANNOTATE_HPP__

#include <iostream>
#include <string>
#include <functional>
#include <algorithm>

#include <unistd.h>

#include <nvhls_connections.h>

#include <rapidjson/document.h>
#include <rapidjson/pointer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/istreamwrapper.h>

namespace interconnect {
const int kDebugLevel = 5;
#ifdef CONNECTIONS_ACCURATE_SIM

template <typename IC>
void __annotate_vector(std::vector<Connections::Connections_BA_abs *> &v,
                       IC &ic, const std::string &root_name,
                       rapidjson::Document &d, bool connectivity_only = false) {

  std::map<std::string, typename IC::IM_chanlink_t *> middle_str_to_chan;

  NVHLS_ASSERT(d.IsObject());

  /////////////////////////// CHANNELS //////////////////////////////////
  // Check if array exists, if not add it.
  if (!d.HasMember("channels")) {
    rapidjson::Value v_channels;
    v_channels.SetObject();
    d.AddMember("channels", v_channels, d.GetAllocator());
  }
  rapidjson::Value &v_channels = d["channels"];

  for (typename IC::channels_middle_t::iterator it_src =
           ic.channels_middle.begin();
       it_src != ic.channels_middle.end(); ++it_src) {
    for (typename IC::channels_middle_inner_t::iterator it_dest =
             it_src->second.begin();
         it_dest != it_src->second.end(); ++it_dest) {

      // Determine name after removing root_name
      std::string it_name = (it_dest->second)->name();
      std::size_t pos = it_name.find(root_name);
      if (pos == std::string::npos) {
        continue;
      }  // Skip if doesn't match root_name
      it_name.erase(pos, root_name.length());

      // Add to string to chan mapping, used for links below.
      middle_str_to_chan[it_name] = it_dest->second;

      // Check if this channel exists in DOM, if not add it.
      if (!v_channels.HasMember(it_name.c_str())) {
        rapidjson::Value v_name;
        v_name.SetString(it_name.c_str(), d.GetAllocator());
        rapidjson::Value v_channel;
        v_channel.SetObject();
        v_channels.AddMember(v_name, v_channel, d.GetAllocator());
      }
      rapidjson::Value &v_channel = v_channels[it_name.c_str()];

      // Check if latency exists, if not add it
      if (!v_channel.HasMember("latency")) {
        rapidjson::Value v_latency;
        v_latency.SetInt(0);
        v_channel.AddMember("latency", v_latency, d.GetAllocator());
      }
      rapidjson::Value &v_latency = v_channel["latency"];

      // Check if capacity exists, if not add it
      if (!v_channel.HasMember("capacity")) {
        rapidjson::Value v_capacity;
        v_capacity.SetInt(0);
        v_channel.AddMember("capacity", v_capacity, d.GetAllocator());
      }
      rapidjson::Value &v_capacity = v_channel["capacity"];

      // Get the src_name and dest_name after subtracting off root
      std::string src_name = (it_dest->second)->src_name();
      {
        if (src_name.length() >= 4 &&
            src_name.substr(src_name.length() - 4, 4) == "_val") {
          src_name.erase(src_name.length() - 4, 4);
        }
        std::size_t pos = src_name.find(root_name);
        if (pos != std::string::npos) {
          src_name.erase(pos, root_name.length());
        }
      }

      std::string dest_name = (it_dest->second)->dest_name();
      {
        if (dest_name.length() >= 4 &&
            dest_name.substr(dest_name.length() - 4, 4) == "_val") {
          dest_name.erase(dest_name.length() - 4, 4);
        }
        std::size_t pos = dest_name.find(root_name);
        if (pos != std::string::npos) {
          dest_name.erase(pos, root_name.length());
        }
      }

      // Add the net driver/receiver, always an output never an input.
      if (v_channel.HasMember("src_name")) {
        if (strcmp(v_channel["src_name"].GetString(), src_name.c_str()) != 0) {
          cout << "Error: During annotation src_name in input doesn't match "
                  "real "
                  "src_name ("
               << v_channel["src_name"].GetString()
               << " != " << src_name.c_str() << ")" << endl;
          assert(0);
        }
      } else {
        rapidjson::Value v_src_name;
        v_src_name.SetString(src_name.c_str(), d.GetAllocator());
        v_channel.AddMember("src_name", v_src_name, d.GetAllocator());
      }

      if (v_channel.HasMember("dest_name")) {
        if (strcmp(v_channel["dest_name"].GetString(), dest_name.c_str()) !=
            0) {
          cout << "Error: During annotation dest_name in input doesn't match "
                  "real dest_name ("
               << v_channel["dest_name"].GetString()
               << " != " << dest_name.c_str() << ")" << endl;
          assert(0);
        }
      } else {
        rapidjson::Value v_dest_name;
        v_dest_name.SetString(dest_name.c_str(), d.GetAllocator());
        v_channel.AddMember("dest_name", v_dest_name, d.GetAllocator());
      }

      // Add width
      if (!v_channel.HasMember("width")) {
        rapidjson::Value v_width;
        v_channel.AddMember("width", v_width, d.GetAllocator());
      }
      rapidjson::Value &v_width = v_channel["width"];
      v_width.SetInt(ic.srcs_width[it_src->first]);

      // Add user_typeid
      int user_typeid = ic.srcs_user_typeid[it_src->first];
      NVHLS_ASSERT(user_typeid != -1);
      if (!v_channel.HasMember("user_typeid")) {
        rapidjson::Value v_user_typeid;
        v_channel.AddMember("user_typeid", v_user_typeid, d.GetAllocator());
      }
      rapidjson::Value &v_user_typeid = v_channel["user_typeid"];
      v_user_typeid.SetInt(ic.srcs_user_typeid[it_src->first]);

      if (v_channel.HasMember("msg_name")) {
        v_channel.RemoveMember("msg_name");
      }
      if (ic.msg_names.find(user_typeid) != ic.msg_names.end()) {
        rapidjson::Value v_msg_name;
        v_msg_name.SetString(ic.msg_names[user_typeid], d.GetAllocator());
        v_channel.AddMember("msg_name", v_msg_name, d.GetAllocator());
      } else {
        NVHLS_ASSERT_MSG(0, "userid is missing a msg_name");
      }

      // Warning: typeid isn't guarenteed to be stable between runs.
      // We can't use it as a key of types, but for groupings.
      // Add typeid
      if (v_channel.HasMember("typeid")) {
        v_channel.RemoveMember("typeid");
      }
      rapidjson::Value v_typeid;
      v_typeid.SetUint64(ic.srcs_typeid[it_src->first]);
      v_channel.AddMember("typeid", v_typeid, d.GetAllocator());

      // Sanity check v_latency
      if (!v_latency.IsInt()) {
        static const char *kTypeNames[] = {"Null",  "False",  "True",  "Object",
                                           "Array", "String", "Number"};
        CDCOUT("Error: Type of member v_latency is"
                   << kTypeNames[v_latency.GetType()]
                   << ". Please ensure it is of type Int." << endl,
               interconnect::kDebugLevel);
      }
      assert(v_latency.IsInt());

      // Sanity check v_capacity
      if (!v_capacity.IsInt()) {
        static const char *kTypeNames[] = {"Null",  "False",  "True",  "Object",
                                           "Array", "String", "Number"};
        CDCOUT("Error: Type of member v_capacity is"
                   << kTypeNames[v_capacity.GetType()]
                   << ". Please ensure it is of type Int." << endl,
               interconnect::kDebugLevel);
      }
      assert(v_capacity.IsInt());

      if (connectivity_only) {
        NVHLS_ASSERT(v_channel.HasMember("latency"));
        NVHLS_ASSERT(v_channel.HasMember("capacity"));
        v_channel.RemoveMember("latency");
        v_channel.RemoveMember("capacity");
      } else {
        // Annotate based on the value.
        (it_dest->second)->annotate(v_latency.GetInt(), v_capacity.GetInt());
      }
    }
  }

  // Check if it exists in the links
  // "links" holds array of channel groups.
  // Each channel group calls out a channel.
  // Check if array exists, if not add it.
  const char *ic_disable_congestion =
      std::getenv("INTERCONNECT_DISABLE_CONGESTION");
  if (d.HasMember("links") && ic_disable_congestion &&
      std::string(ic_disable_congestion) != "1") {
    rapidjson::Value &v_links = d["links"];

    assert(v_links.IsArray());
    for (rapidjson::SizeType i = 0; i < v_links.Size(); i++) {
      rapidjson::Value &v_linkgroup = v_links[i];
      assert(v_linkgroup.IsArray());

      interconnect::LinkGroup *link_group = new LinkGroup();
      interconnect::get_conManagerLinks().link_groups.push_back(link_group);

      for (rapidjson::SizeType j = 0; j < v_linkgroup.Size(); j++) {
        rapidjson::Value &v_channel_link = v_linkgroup[j];
        std::string chan_string = v_channel_link["channel_name"].GetString();
        typename std::map<std::string, typename IC::IM_chanlink_t *>::iterator
            it = middle_str_to_chan.find(chan_string);
        if (it != middle_str_to_chan.end()) {
          if (it->second->is_bypass()) {
            CDCOUT("Warning: Channel "
                       << chan_string
                       << " is in bypass, so link probability won't be active. "
                          "Adding to link group anyway..."
                       << endl,
                   interconnect::kDebugLevel);
          }
          if (v_channel_link.HasMember("bw_prob"))
            link_group->link_probs[it->second] =
                v_channel_link["bw_prob"].GetDouble();
          else
            link_group->link_probs[it->second] = 1.0;

          if (v_channel_link.HasMember("weight"))
            link_group->weights[it->second] =
                v_channel_link["weight"].GetDouble();
          else
            link_group->weights[it->second] = 1.0;
        } else {
          CDCOUT("Warning: Couldn't find channel "
                     << chan_string << " for link group during annotation!"
                     << endl,
                 interconnect::kDebugLevel);
        }
      }
    }
  }
}

template <typename IC>
void annotate_design(const sc_object &root, IC &ic, std::string base_name = "",
                     std::string input_dir_path = "",
                     std::string output_dir_path = "") {
  bool explicit_input_dir = false;

  // Add delim if non-empty base_name
  if (base_name.length() > 0) {
    base_name += ".";
  }

  base_name += root.name();
  base_name += ".";

  // Sanity check input and output paths if they exist
  if (input_dir_path.length() > 0) {
    input_dir_path += "/";
    explicit_input_dir = true;
  }
  if (output_dir_path.length() > 0) {
    output_dir_path += "/";
  }

  std::string input_path = input_dir_path + base_name + "input.json";
  std::string output_path = output_dir_path + base_name + "output.json";

  // Create DOM object.
  rapidjson::Document d;

  // Try reading document from input.json
  std::ifstream ifs(input_path.c_str());
  if (!ifs.fail()) {

    const unsigned int read_tries = 10;
    unsigned int read_tries_ = read_tries;
    bool first = false;
    CDCOUT("Info: Reading back-annotation from "
               << input_path << ". Please diff " << input_path << " and "
               << output_path << " to ensure it matches expectations." << endl,
           interconnect::kDebugLevel);
    do {
      if (read_tries_ < read_tries) {
        cout << "Info: retrying read of " << input_path << "..." << endl;
        sleep(5);
        ifs.close();
        ifs.open(input_path.c_str());
      }
      if (ifs.good()) {
        rapidjson::IStreamWrapper isw(ifs);
        d.ParseStream(isw);
      }
    } while (!d.IsObject() && --read_tries_);
    if (read_tries_ == 0) {
      cout << "Error: failed to read in correct annotation object from "
           << input_path << endl;
      exit(-1);
    } else {
      cout << "Info: Annotated. " << dec << read_tries_
           << " tries were remaining." << endl;
    }
  } else {
    cout << "Warning: Could not read input json " << input_path.c_str() << endl;
    d.SetObject();
  }

  std::string root_name = std::string(root.name()) + ".";
  assert(root_name.length() > 1);  // Should be something other than just "."

  __annotate_vector(interconnect::get_conManagerLinks().tracked_annotate, ic,
                    root_name, d);
  // Output DOM to file
  std::ofstream ofs(output_path.c_str());
  rapidjson::OStreamWrapper osw(ofs);
  rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
  d.Accept(writer);
}

template <typename IC>
void save_connectivity(const sc_object &root, IC &ic,
                       std::string base_name = "",
                       std::string connectivity_dir_path = "") {
  // Add delim if non-empty base_name
  if (base_name.length() > 0) {
    base_name += ".";
  }

  base_name += root.name();
  base_name += ".";

  // Sanity check input and output paths if they exist
  if (connectivity_dir_path.length() > 0) {
    connectivity_dir_path += "/";
  }

  std::string connectivity_path =
      connectivity_dir_path + base_name + "connectivity.json";

  // Create DOM object.
  rapidjson::Document d;
  d.SetObject();

  std::string root_name = std::string(root.name()) + ".";
  assert(root_name.length() > 1);  // Should be something other than just "."

  /////////////////////////// MESSAGES //////////////////////////////////
  // List of message types and corresponding ids

  for (auto it = ic.msg_names.begin(); it != ic.msg_names.end(); ++it) {
    rapidjson::Pointer(std::string() + "/msgs/" + it->second).Set(d, it->first);
  }

  /////////////////////////// IC_ID //////////////////////////////////
  rapidjson::Pointer("/ic_id").Set(d, ic.ic_id);

  /////////////////////////// IC INTERFACES /////////////////////////////
  for (auto it = ic.part_names.begin(); it != ic.part_names.end(); ++it) {
    const unsigned int part_id = it->first;
    const char *part_name = it->second;
    rapidjson::Pointer(std::string() + "/parts/" + part_name + "/part_id")
        .Set(d, part_id);

    NVHLS_ASSERT(ic.part_to_part_type.size() == ic.part_inst_names.size())

    int i = 0;
    for (auto ij = ic.part_to_part_type.begin();
         ij != ic.part_to_part_type.end(); ++ij) {
      if (ij->second == part_id) {
        const unsigned int user_part_id = ij->first;
        rapidjson::Pointer(std::string() + "/parts/" + part_name + "/insts/" +
                           std::to_string(i))
            .Set(d, ij->first);
        rapidjson::Pointer(std::string() + "/parts/" + part_name +
                           "/inst_names/" + std::to_string(i))
            .Set(d, ic.part_inst_names[ij->first]);
        i++;
      }
    }

    for (auto ij = ic.part_msgs_srcs[part_id].begin();
         ij != ic.part_msgs_srcs[part_id].end(); ++ij) {
      const unsigned int msg_id = ij->first;
      int i = 0;
      for (auto port_id_p = ij->second.begin(); port_id_p != ij->second.end();
           ++port_id_p) {
        rapidjson::Pointer(std::string() + "/parts/" + part_name +
                           "/src_ports/" + ic.msg_names[msg_id] + "/" +
                           std::to_string(i++))
            .Set(d, *port_id_p);
      }
    }

    for (auto ij = ic.part_msgs_dests[part_id].begin();
         ij != ic.part_msgs_dests[part_id].end(); ++ij) {
      const unsigned int msg_id = ij->first;
      int i = 0;
      for (auto port_id_p = ij->second.begin(); port_id_p != ij->second.end();
           ++port_id_p) {
        rapidjson::Pointer(std::string() + "/parts/" + part_name +
                           "/dest_ports/" + ic.msg_names[msg_id] + "/" +
                           std::to_string(i++))
            .Set(d, *port_id_p);
      }
    }
  }

  /////////////////////////// CHANNELS //////////////////////////////////
  __annotate_vector(Connections::get_conManager().tracked_annotate, ic,
                    root_name, d, true);

  // Output DOM to file
  std::ofstream ofs(connectivity_path.c_str());
  rapidjson::OStreamWrapper osw(ofs);
  rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
  d.Accept(writer);
}

#else

/**
 * \brief Back annotate Combinational connections for a design
 * \ingroup Connections
 *
 * \tparam root          Root sc_object to annotate.
 * \tparam base_name     Base file name (optional, defaults to name of root
 * within hierarchy)
 * \tparam base_name     Input directory (optional, defaults to current working
 * directory)
 * \tparam base_name     Output directory (optional, defaults to current working
 * directory)
 *
 * \par Description
 *      When a design is annotated, it writes out a base_name.output.json with
 * all annotatable
 *      Combinational LI channels, defaulting to a latency and capacity of 0 for
 * each (simple
 *      wire). base_name.output.json can be used as a template for
 * base_name.input.json, which
 *      it will read in and annotate based on given latency and capacity.
 * Latency is given in
 *      units of global clock cycles, while capacity is number of units that can
 * be stored in
 *      the channel. For a retimed path, latency and capacity should be equal,
 * while for
 *      buffered paths capacity will exceed latency.
 *
 *      Back annotation will only work with CONNECTIONS_ACCURATE_SIM define
 * enabled, and
 *      in MARSHALL_PORT or DIRECT_PORT modes. It will not work in TLM_PORT
 * (CONNECTIONS_FAST_SIM)
 *      or SYN_PORT modes. Additionally, it is dependent on RapidJSON as a git
 * submodule to read
 *      and write JSON file format.
 *
 *      In the base_name.output.json, a list of combinational names and
 * connecting ports is given.
 *      UNBOUND indicates a Combinational channel that hasn't been Bind()'d on
 * one or more ports,
 *      while TLM_INTERFACE indicates a channel that is along a TLM_PORT
 * interface in cosimulation
 *      cases.
 *
 * \par A Simple Example
 * \code
 *      #include <connections/connections.h>
 *      #include <connections/annotate.h>
 *
 *      ...
 *      // Add to sc_main, to annotate testbench DUT.
 *      // Always generates a my_testbench.dut.output.json of the annotated
 * design
 *      // and tries to read in my_testbench.dut.input.json if it exists.
 *      annotate_design(my_testbench.dut);
 *      ...
 * \endcode
 * \par
 *
 */
void annotate_design(const sc_object &root, std::string base_name = "",
                     std::string input_dir_path = "",
                     std::string output_dir_path = "") {
  cerr << "WARNING: Cannot annotate_design() unless running in sim accurate "
          "mode!"
       << endl;
  cerr << "WARNING: Design will not be annotated." << endl;
}

template <typename IC>
void save_connectivity(const sc_object &root, IC &ic,
                       std::string base_name = "",
                       std::string connectivity_dir_path = "") {
  cerr << "WARNING: Cannot save_connectivity() unless running in sim accurate "
          "mode!"
       << endl;
  cerr << "WARNING: Connectivity will not be saved." << endl;
}

#endif
};

#endif  // __INTERCONNECT_ANNOTATE_HPP__
