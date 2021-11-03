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

#ifndef PRODUCER_CONSUMER_PART_H
#define PRODUCER_CONSUMER_PART_H

#include <deque>

#define NUM_DESTS 16

typedef struct flit_data {
  unsigned int payload;
  sc_time send_time;
} flit_data_t;

typedef struct stat {

  sc_time cum_latency;
  sc_time max_latency;
  unsigned long long num_msgs;

  stat() {
    cum_latency = sc_time(0, SC_NS);
    max_latency = sc_time(0, SC_NS);
    num_msgs = 0;
  }

} stat_t;

class ProducerConsumerLog {
 public:
  static const int kDebugLevel = 4;

  unsigned int num_flits_in_flight;
  unsigned int num_flits_send_total;
  unsigned int num_flits_recv_total;

  std::map<unsigned int, std::map<unsigned int, stat_t> > stats;
  std::map<unsigned int, std::map<unsigned int, std::deque<flit_data_t> > >
      flits_in_flight;

  unsigned int pre_num_flits_in_flight;
  unsigned int pre_num_flits_send_total;
  unsigned int pre_num_flits_recv_total;
  std::map<unsigned int, std::map<unsigned int, stat_t> > pre_stats;
  std::map<unsigned int, std::map<unsigned int, std::deque<flit_data_t> > >
      pre_flits_in_flight;

  std::array<float, PE_WIDTH * PE_HEIGHT>
      PE_activity;  // 1.0 = every cycle, 0.0 = no cycles
  std::array<bool, PE_WIDTH * PE_HEIGHT>
      PE_unicast_only;  // 0 = multicast allowed, 1 = unicast only
  std::array<std::array<float, PE_WIDTH * PE_HEIGHT>, PE_WIDTH * PE_HEIGHT>
      PE_dest_probs;  // indiviual probs to send to each. Outer = sender, inner
                      // = dest.

  ProducerConsumerLog() {
    num_flits_in_flight = 0;
    num_flits_send_total = 0;
    num_flits_recv_total = 0;

    pre_num_flits_in_flight = 0;
    pre_num_flits_send_total = 0;
    pre_num_flits_recv_total = 0;

    for (unsigned int i = 0; i < PE_WIDTH * PE_HEIGHT; ++i) {
      PE_activity[i] = 0;
      PE_unicast_only[i] = false;
      for (unsigned int j = 0; j < PE_WIDTH * PE_HEIGHT; ++j) {
        PE_dest_probs[i][j] = 0;
      }
    }
  }

  void record_pre_send(unsigned int pe_id_sender, unsigned int pe_id_receiver,
                       unsigned int payload, sc_time pre_time) {
    pre_num_flits_in_flight++;
    pre_num_flits_send_total++;

    flit_data_t data({.payload = payload, .send_time = pre_time});
    pre_flits_in_flight[pe_id_sender][pe_id_receiver].push_back(data);
  }

  void record_send(unsigned int pe_id_sender, unsigned int pe_id_receiver,
                   unsigned int payload) {
    num_flits_in_flight++;
    num_flits_send_total++;

    flit_data_t data({.payload = payload, .send_time = sc_time_stamp()});
    flits_in_flight[pe_id_sender][pe_id_receiver].push_back(data);
  }

  void record_recv(unsigned int pe_id_receiver, unsigned int payload) {
    num_flits_in_flight--;
    num_flits_recv_total++;

    std::vector<unsigned int> pe_id_senders;
    for (auto sender_it = flits_in_flight.begin();
         sender_it != flits_in_flight.end(); ++sender_it) {
      if (sender_it->second.find(pe_id_receiver) == sender_it->second.end())
        continue;
      if (sender_it->second[pe_id_receiver].front().payload == payload) {
        pe_id_senders.push_back(sender_it->first);
      }
    }
    NVHLS_ASSERT(pe_id_senders.size() == 1);

    unsigned int pe_id_sender = pe_id_senders[0];

    NVHLS_ASSERT(!flits_in_flight[pe_id_sender][pe_id_receiver].empty());

    sc_time tof =
        sc_time_stamp() -
        flits_in_flight[pe_id_sender][pe_id_receiver].front().send_time;
    if (stats[pe_id_sender][pe_id_receiver].max_latency < tof) {
      stats[pe_id_sender][pe_id_receiver].max_latency = tof;
    }
    stats[pe_id_sender][pe_id_receiver].cum_latency += tof;
    stats[pe_id_sender][pe_id_receiver].num_msgs++;
    flits_in_flight[pe_id_sender][pe_id_receiver].pop_front();
  }

  void record_pre_recv(unsigned int pe_id_receiver, unsigned int payload) {
    pre_num_flits_in_flight--;
    pre_num_flits_recv_total++;

    std::vector<unsigned int> pe_id_senders;
    for (auto sender_it = pre_flits_in_flight.begin();
         sender_it != pre_flits_in_flight.end(); ++sender_it) {
      if (sender_it->second.find(pe_id_receiver) == sender_it->second.end())
        continue;
      if (sender_it->second[pe_id_receiver].front().payload == payload) {
        pe_id_senders.push_back(sender_it->first);
      }
    }
    NVHLS_ASSERT(pe_id_senders.size() == 1);

    unsigned int pe_id_sender = pe_id_senders[0];

    NVHLS_ASSERT(!pre_flits_in_flight[pe_id_sender][pe_id_receiver].empty());

    sc_time tof =
        sc_time_stamp() -
        pre_flits_in_flight[pe_id_sender][pe_id_receiver].front().send_time;
    if (pre_stats[pe_id_sender][pe_id_receiver].max_latency < tof) {
      pre_stats[pe_id_sender][pe_id_receiver].max_latency = tof;
    }
    pre_stats[pe_id_sender][pe_id_receiver].cum_latency += tof;
    pre_stats[pe_id_sender][pe_id_receiver].num_msgs++;
    pre_flits_in_flight[pe_id_sender][pe_id_receiver].pop_front();
  }

  void print_stats() {
    sc_time cum_latency(0, SC_NS);
    sc_time max_latency(0, SC_NS);
    unsigned long long num_msgs(0);

    for (auto sender_it = stats.begin(); sender_it != stats.end();
         ++sender_it) {
      for (auto receiver_it = sender_it->second.begin();
           receiver_it != sender_it->second.end(); ++receiver_it) {
        unsigned int pe_id_sender = sender_it->first;
        unsigned int pe_id_receiver = receiver_it->first;
        stat_t stat = receiver_it->second;

        cout << "Avg latency from " << pe_id_sender << " to " << pe_id_receiver
             << " = " << (stat.cum_latency / stat.num_msgs) / sc_time(1, SC_NS)
             << " cycles (sent " << stat.num_msgs << ", max "
             << stat.max_latency << ")" << endl;

        cum_latency += stat.cum_latency;
        num_msgs += stat.num_msgs;
        if (max_latency < stat.max_latency) {
          max_latency = stat.max_latency;
        }
      }
    }

    cout << "Overall network avg latency: "
         << (cum_latency / num_msgs) / sc_time(1, SC_NS) << " cycles ("
         << num_msgs << " total messages, max " << max_latency << ")" << endl;
  }

  void print_pre_stats() {
    sc_time cum_latency(0, SC_NS);
    unsigned long long num_msgs(0);

    for (auto sender_it = pre_stats.begin(); sender_it != pre_stats.end();
         ++sender_it) {
      for (auto receiver_it = sender_it->second.begin();
           receiver_it != sender_it->second.end(); ++receiver_it) {
        unsigned int pe_id_sender = sender_it->first;
        unsigned int pe_id_receiver = receiver_it->first;
        stat_t stat = receiver_it->second;

        CDCOUT(sc_time_stamp()
                   << ": "
                   << "Avg latency from " << pe_id_sender << " to "
                   << pe_id_receiver << " = "
                   << (stat.cum_latency / stat.num_msgs) / sc_time(1, SC_NS)
                   << " cycles (sent " << stat.num_msgs << ")" << endl,
               kDebugLevel);

        cum_latency += stat.cum_latency;
        num_msgs += stat.num_msgs;
      }
    }

    cout << "Overall packet avg latency: "
         << (cum_latency / num_msgs) / sc_time(1, SC_NS) << " cycles ("
         << num_msgs << " total messages)" << endl;
  }
};

template <class Dummy>
struct pc_log_statics {
  static ProducerConsumerLog pc_log;
  static unsigned int num_flits_in_flight;
};

template <class Dummy>
ProducerConsumerLog pc_log_statics<Dummy>::pc_log;

inline ProducerConsumerLog& get_pc_log() {
  return pc_log_statics<void>::pc_log;
}

template <typename Dummy>
class ProducerConsumer : public sc_module {
  SC_HAS_PROCESS(ProducerConsumer);

  unsigned int pe_id;

 public:
  sc_in_clk clk;
  sc_in<bool> rst;

  Connections::In<IC_MESSAGE(my_msg)> in_port;
  Connections::Out<IC_MESSAGE(my_msg)> out_port;

  // Connections::Out<IC_MESSAGE(my_ctrl)> out_port_ctrl;
  // Connections::In<IC_MESSAGE(my_ctrl)> in_port_ctrl;

  ProducerConsumer(sc_module_name nm, unsigned int pe_id_) : sc_module(nm) {
    pe_id = pe_id_;

    SC_THREAD(ProducerRun);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

    SC_THREAD(ConsumerRun);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void ProducerRun() {
    out_port.Reset();

    while (1) {
      wait();

      float bw_prob = get_pc_log().PE_activity[pe_id];
      if (bw_prob == 0.0)
        continue;
      if (bw_prob < 1.0 && ((static_cast<double>(rand()) /
                             static_cast<double>(RAND_MAX)) >= bw_prob))
        continue;

      IC_MESSAGE(my_msg) im;
      im = rand();

      std::vector<unsigned int> dest_idxs;

      if (!get_pc_log().PE_unicast_only[pe_id]) {
        for (unsigned int dest_idx = 0; dest_idx < PE_WIDTH * PE_HEIGHT;
             ++dest_idx) {
          float bw_prob = get_pc_log().PE_dest_probs[pe_id][dest_idx];
          if (bw_prob == 0.0)
            continue;
          if (bw_prob < 1.0 && ((static_cast<double>(rand()) /
                                 static_cast<double>(RAND_MAX)) >= bw_prob))
            continue;
          im << IC_TO(dest_idx);
          dest_idxs.push_back(dest_idx);
        }
      } else {
        float total_prob = 0;
        for (unsigned int dest_idx = 0; dest_idx < PE_WIDTH * PE_HEIGHT;
             ++dest_idx) {
          float bw_prob = get_pc_log().PE_dest_probs[pe_id][dest_idx];
          total_prob += bw_prob;
        }
        if (total_prob == 0.0)
          continue;

        float rand_result =
            (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
            total_prob;
        for (unsigned int dest_idx = 0; dest_idx < PE_WIDTH * PE_HEIGHT;
             ++dest_idx) {
          float bw_prob = get_pc_log().PE_dest_probs[pe_id][dest_idx];
          if (bw_prob == 0.0)
            continue;
          rand_result -= bw_prob;
          if (rand_result <= 0) {
            im << IC_TO(dest_idx);
            dest_idxs.push_back(dest_idx);
            break;
          }
        }
      }

      // Record how long we've been waiting to send in "pre_time"
      sc_time pre_time = sc_time_stamp();

      out_port.Push(im);

      for (auto it = dest_idxs.begin(); it != dest_idxs.end(); ++it) {
        get_pc_log().record_pre_send(pe_id, *it, im.get_msg(), pre_time);
        get_pc_log().record_send(pe_id, *it, im.get_msg());
      }
    }
  }
  void ConsumerRun() {
    in_port.Reset();

    while (1) {
      wait();
      IC_MESSAGE(my_msg) im = in_port.Pop();
      get_pc_log().record_recv(pe_id, im.get_msg());
      get_pc_log().record_pre_recv(pe_id, im.get_msg());
    }
  }
};

class ProducerConsumerPart : public sc_module {
 public:
  IC_HAS_INTERFACE(ProducerConsumerPart);

  sc_in_clk clk;
  sc_in<bool> rst;

  ProducerConsumer<void> pc_inst;

  ProducerConsumerPart(sc_module_name nm, unsigned int pe_id)
      : sc_module(nm), pc_inst("pc_inst", pe_id) {
// During actual HLS we want ProducerConsumerPart to bind clock,
// but the scverify wrapper doesn't bind clock, so we must bind
// it here.
#if defined(__SYNTHESIS__) || !defined(INTERCONNECT_GEN)
    ic_interface.clk(clk);
    ic_interface.rst(rst);
#endif

    // Bind to pe module
    pc_inst.clk(clk);
    pc_inst.rst(rst);

    // Bind to interconnect interface
    IC_PORT_BIND(pc_inst.in_port);
    IC_PORT_BIND(pc_inst.out_port);
  }
};

#endif
