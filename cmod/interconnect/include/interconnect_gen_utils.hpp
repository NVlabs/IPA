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

#ifndef __INTERCONNECT_GEN_UTILS_HPP__
#define __INTERCONNECT_GEN_UTILS_HPP__

// 8/21/20 npinckney - Ensure that nvhls_assert.h overrides SystemC
#undef NVHLS_ASSERT_H
#include <nvhls_assert.h>
#include <CombinationalBufferedPorts.h>
#include <interconnect/include/Interconnect.hpp>
#include <Arbiter.h>
#include <arbitrated_crossbar.h>
#include <interconnect/include/LUT_Routers.hpp>
#include <one_hot_to_bin.h>

namespace interconnect {

///////////// Interconnect Message ///////////////
template <typename MSG_TYPE, unsigned int dest_count>
class LinkMsg : public nvhls_message {
 public:
  static const MSG_ID_t MSG_ID = MSG_TYPE::msg_id;
  typedef typename MSG_TYPE::data_t data_t;

  data_t msg;
  sc_lv<dest_count> dest_bits;

  //// Constructor-Destructor
  LinkMsg<MSG_TYPE, dest_count>() { dest_bits = 0; }
  LinkMsg<MSG_TYPE, dest_count>(const data_t &m) {
    dest_bits = 0;
    set_msg(m);
  }

  virtual inline unsigned int dest_map(const Destination &dest) {
    NVHLS_ASSERT(0);
    return 0;
  }

  //// Operators
  LinkMsg<MSG_TYPE, dest_count> &operator=(const data_t &m) {
    set_msg(m);
    return *this;
  }
  LinkMsg<MSG_TYPE, dest_count> &operator<<(const Destination &dest) {
    dest_bits[dest_map(dest)] = 1;
    NVHLS_ASSERT(0);
    return *this;
  }
  operator data_t() { return get_msg(); }

  // Some message types implement this for mapping (e.g. supermsg types)
  sc_lv<dest_count> get_dest_insts() { return dest_bits; }

  //// Setter-Getter
  void set_msg(const data_t &m) { msg = m; }
  data_t get_msg() const { return msg; }
  //// Temporary getters and setters until API changed
  template <typename interconnect_t>
  void set_msg(interconnect_t &ic, const data_t &m) {
    set_msg(m);
  }
  template <typename interconnect_t>
  void get_msg(interconnect_t &ic, data_t &m) {
    m = get_msg();
  }
  // Conversion to generic type
  template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
  operator InterconnectMessageSCLV<MaxDataWidth, MaxDestCount>() {
    // NVHLS_ASSERT(0);
    assert(MaxDestCount == interconnect::MaxDestCount);
    assert(Wrapped<data_t>::width <= MaxDataWidth);

    InterconnectMessageSCLV<MaxDataWidth, MaxDestCount> new_im;
    new_im.set_msg(msg);
    return new_im;
  }

  //// Marshall support
  enum { width = Wrapped<data_t>::width + Wrapped<sc_lv<dest_count>>::width };

  template <unsigned int Size>
  void Marshall(Marshaller<Size> &m) {
    m &msg;
    m &dest_bits;
  }
};

// Stub specialization
template <typename MSG_TYPE>
class LinkMsg<MSG_TYPE, 0> : public nvhls_message {
 public:
  static const MSG_ID_t MSG_ID = MSG_TYPE::msg_id;
  typedef typename MSG_TYPE::data_t data_t;

  // The actual message payload.
  data_t msg;

  //// Constructor-Destructor
  LinkMsg<MSG_TYPE, 0>() {}
  LinkMsg<MSG_TYPE, 0>(const data_t &m) { set_msg(m); }

  virtual inline unsigned int dest_map(const Destination &dest) {
    NVHLS_ASSERT(0);
    return 0;
  }

  //// Operators
  LinkMsg<MSG_TYPE, 0> &operator=(const data_t &m) {
    set_msg(m);
    return *this;
  }

  LinkMsg<MSG_TYPE, 0> &operator<<(const Destination &dest) {
    NVHLS_ASSERT(0);
    return *this;
  }
  operator data_t() { return get_msg(); }

  //// Setter-Getter
  void set_msg(const data_t &m) { msg = m; }
  data_t get_msg() { return msg; }
  //// Temporary getters and setters until API changed
  template <typename interconnect_t>
  void set_msg(interconnect_t &ic, const data_t &m) {
    set_msg(m);
  }
  template <typename interconnect_t>
  void get_msg(interconnect_t &ic, data_t &m) {
    m = get_msg();
  }
  // Conversion to generic type
  template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
  operator InterconnectMessageSCLV<MaxDataWidth, MaxDestCount>() {
    // NVHLS_ASSERT(0);
    assert(MaxDestCount == interconnect::MaxDestCount);
    assert(Wrapped<data_t>::width <= MaxDataWidth);

    InterconnectMessageSCLV<MaxDataWidth, MaxDestCount> new_im;
    new_im.set_msg(msg);
    return new_im;
  }
  template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
  explicit LinkMsg<MSG_TYPE, 0>(
      const InterconnectMessageSCLV<MaxDataWidth, MaxDestCount> &im) {}

  //// Marshall support
  enum { width = Wrapped<data_t>::width };

  template <unsigned int Size>
  void Marshall(Marshaller<Size> &m) {
    m &msg;
  }
};

// ///////////// Interconnect Retiming Stage ///////////////

// Internal single internal retiming stage.
template <typename Message, unsigned int extra_capacity,
          Connections::connections_port_t PortType = AUTO_PORT>
class RetimingStagesInt : public sc_module {
  SC_HAS_PROCESS(RetimingStagesInt);

 public:
  sc_in_clk clk;
  sc_in<bool> rst;

  // New version (fifo based)
  Connections::In<Message> in_port;
  Connections::OutBuffered<Message, 1 + extra_capacity> out_port;

  RetimingStagesInt(sc_module_name name_) : sc_module(name_) {
    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst, false);
  }

  void run() {
    in_port.Reset();
    out_port.Reset();

    Message m;
    bool popped = false;

#pragma hls_pipeline_init_interval 1
#pragma pipeline_stall_mode flush
    while (1) {
      wait();
      if (!popped)
        popped = in_port.PopNB(m);
      if (!out_port.Full() && popped) {
        out_port.Push(m);
        popped = false;
      }
      out_port.TransferNB();
    }
  }
};

// Variable retiming stages.
template <typename Message, unsigned int num_stages,
          unsigned int extra_capacity,
          Connections::connections_port_t PortType = AUTO_PORT>
class RetimingStages : public sc_module {
 public:
  sc_in_clk clk;
  sc_in<bool> rst;

  // New version (fifo based)
  Connections::InBuffered<Message, 1> in_port;
  Connections::OutBuffered<Message, 1 + extra_capacity> out_port;

  nvhls::nv_array<RetimingStagesInt<Message, 0, PortType>, num_stages> fifos;
  nvhls::nv_array<Connections::Combinational<Message>, num_stages - 1>
      fifos_conn;

  RetimingStages()
      : sc_module(sc_gen_unique_name("retimingstages")),
        fifos("fifos"),
        fifos_conn("fifos_conn") {
    for (int i = 0; i < num_stages; i++) {
      fifos[i].clk(clk);
      fifos[i].rst(rst);
      if (i == 0) {
        fifos[i].in_port(in_port);
      } else {
        fifos[i].in_port(fifos_conn[i - 1]);
      }
      if (i == num_stages - 1) {
        fifos[i].out_port(out_port);
      } else {
        fifos[i].out_port(fifos_conn[i]);
      }
    }
  }

  explicit RetimingStages(sc_module_name name_)
      : sc_module(name_), fifos("fifos"), fifos_conn("fifos_conn") {
    for (int i = 0; i < num_stages; i++) {
      fifos[i].clk(clk);
      fifos[i].rst(rst);
      if (i == 0) {
        fifos[i].in_port(in_port);
      } else {
        fifos[i].in_port(fifos_conn[i - 1]);
      }
      if (i == num_stages - 1) {
        fifos[i].out_port(out_port);
      } else {
        fifos[i].out_port(fifos_conn[i]);
      }
    }
  }
};

// Specialization for single stage case.
template <typename Message, unsigned int extra_capacity,
          Connections::connections_port_t PortType>
class RetimingStages<Message, 1, extra_capacity, PortType> : public sc_module {
 public:
  sc_in_clk clk;
  sc_in<bool> rst;

  Connections::InBuffered<Message, 1> in_port;
  Connections::OutBuffered<Message, 1 + extra_capacity> out_port;

  nvhls::nv_array<RetimingStagesInt<Message, 0, PortType>, 1> fifos;

  RetimingStages()
      : sc_module(sc_gen_unique_name("retimingstages")), fifos("fifos") {
    fifos[0].clk(clk);
    fifos[0].rst(rst);
    fifos[0].in_port(in_port);
    fifos[0].out_port(out_port);
  }

  explicit RetimingStages(sc_module_name name_)
      : sc_module(name_), fifos("fifos") {
    fifos[0].clk(clk);
    fifos[0].rst(rst);
    fifos[0].in_port(in_port);
    fifos[0].out_port(out_port);
  }
};

// Alias to 1 case since we always need a buffer to get extra capacity.
template <typename Message, unsigned int extra_capacity,
          Connections::connections_port_t PortType>
class RetimingStages<Message, 0, extra_capacity, PortType>
    : public RetimingStages<Message, 1, extra_capacity - 1, PortType> {
 public:
  RetimingStages()
      : RetimingStages<Message, 1, extra_capacity - 1, PortType>(
            sc_gen_unique_name("retimingstages")) {}

  explicit RetimingStages(sc_module_name name_)
      : RetimingStages<Message, 1, extra_capacity - 1, PortType>(name_) {}
};

// Specialization for 0 retiming stages, just a bypass instead.
template <typename Message, Connections::connections_port_t PortType>
class RetimingStages<Message, 0, 0, PortType> : public sc_module {
  SC_HAS_PROCESS(RetimingStages);

 public:
  Connections::In<Message> in_port;
  Connections::Out<Message> out_port;

  RetimingStages() : sc_module(sc_gen_unique_name("retimingstages")) {
    declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                           SC_CURRENT_USER_MODULE, do_bypass);
    sensitive << in_port.msg << in_port.val << out_port.rdy;

#ifdef CONNECTIONS_SIM_ONLY
    in_port.disable_spawn();
    out_port.disable_spawn();
#endif
  }

  RetimingStages(sc_module_name name_) : sc_module(name_) {
    declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                           SC_CURRENT_USER_MODULE, do_bypass);
    sensitive << in_port.msg << in_port.val << out_port.rdy;

#ifdef CONNECTIONS_SIM_ONLY
    in_port.disable_spawn();
    out_port.disable_spawn();
#endif
  }

  void do_bypass() {
    out_port.val.write(in_port.val.read());
    in_port.rdy.write(out_port.rdy.read());
    out_port.msg.write(in_port.msg.read());
  }
};

///////////// Interconnect Interface ///////////////
//#pragma hls_ungroup
template <typename MSG_TYPE, int dest_count,
          Connections::connections_port_t PortType = AUTO_PORT>
class LinkSender : public sc_module {
  SC_HAS_PROCESS(LinkSender);

 protected:
  Connections::Combinational<interconnect::Msg<MSG_TYPE>> in_port;

 public:
  static const int kDebugLevel = 0;

  sc_in_clk clk;
  sc_in<bool> rst;
  Connections::OutBuffered<interconnect::Msg<MSG_TYPE>, 1, PortType>
      out_ports[dest_count];

  LinkSender() : sc_module(sc_module_name(sc_gen_unique_name("link_sender"))) {
    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst, false);
  }

  void Bind(Connections::OutBlocking<interconnect::Msg<MSG_TYPE>> &p) {
    p(in_port);
  }

  void run() {
    in_port.ResetRead();

#pragma hls_unroll yes
    for (int i = 0; i < dest_count; ++i) {
      out_ports[i].Reset();
    }

    bool all_sent = true;
    interconnect::Msg<MSG_TYPE> data;
    sc_lv<dest_count> dest_bits = 0;

#pragma hls_pipeline_init_interval 1
#pragma pipeline_stall_mode flush
    while (1) {
      wait();

      if (all_sent) {
        interconnect::Msg<MSG_TYPE> in_data;
        CDCOUT(sc_time_stamp() << " " << name() << " LinkSender - "
                               << MSG_TYPE::name << " Checking input port\n",
               kDebugLevel);
        if (in_port.PopNB(in_data)) {
          data = in_data;
          dest_bits = data.dest_bits;
          CDCOUT(sc_time_stamp() << " " << name() << " LinkSender - "
                                 << MSG_TYPE::name << " Popped data: "
                                 << ((typename MSG_TYPE::data_t)data) << endl,
                 kDebugLevel);
          all_sent = false;
        }
      }

// Push and update.
#pragma hls_unroll yes
      for (int i = 0; i < dest_count; ++i) {
        if (dest_bits[i] == 1) {
          if (!out_ports[i].Full()) {
            out_ports[i].Push(data);
            CDCOUT(sc_time_stamp() << " " << name() << " LinkSender - "
                                   << MSG_TYPE::name << " Dest: " << i
                                   << " FIFO push output data: "
                                   << ((typename MSG_TYPE::data_t)data) << endl,
                   kDebugLevel);
            dest_bits[i] = 0;
          }
        }
      }

      // End condition
      all_sent = (dest_bits == 0);

#pragma hls_unroll yes
      for (int i = 0; i < dest_count; ++i) {
        out_ports[i].TransferNB();
      }
    }
  }
};

// Stub specialization
//#pragma hls_ungroup
template <typename MSG_TYPE, Connections::connections_port_t PortType>
class LinkSender<MSG_TYPE, 0, PortType> : public sc_module {
  SC_HAS_PROCESS(LinkSender);

 protected:
  Connections::Combinational<interconnect::Msg<MSG_TYPE>> in_port;

 public:
  sc_in_clk clk;
  sc_in<bool> rst;

  LinkSender() : sc_module(sc_module_name(sc_gen_unique_name("link_sender"))) {
    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst, false);
  }

  void Bind(Connections::OutBlocking<interconnect::Msg<MSG_TYPE>> &p) {
    p(in_port);
  }

  void run() {
    in_port.ResetRead();

    interconnect::Msg<MSG_TYPE> data;

#pragma hls_pipeline_init_interval 1
#pragma pipeline_stall_mode flush
    while (1) {
      wait();
      in_port.Pop();  // Always run, since no destinations do not block
    }
  }
};

//#pragma hls_ungroup
template <typename MSG_TYPE, int dest_count,
          Connections::connections_port_t PortType = AUTO_PORT>
class LinkSenderPort : public sc_module {
  SC_HAS_PROCESS(LinkSenderPort);

 public:
  static const int kDebugLevel = 0;

  sc_in_clk clk;
  sc_in<bool> rst;
  Connections::In<interconnect::Msg<MSG_TYPE>> in_port;
  Connections::OutBuffered<interconnect::Msg<MSG_TYPE>, 1, PortType>
      out_ports[dest_count];

  LinkSenderPort()
      : sc_module(sc_module_name(sc_gen_unique_name("link_sender"))) {
    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst, false);
  }

  void Bind(Connections::InBlocking<interconnect::Msg<MSG_TYPE>> &p) {
    in_port(p);
  }

  void run() {
    in_port.Reset();

#pragma hls_unroll yes
    for (int i = 0; i < dest_count; ++i) {
      out_ports[i].Reset();
    }

    bool all_sent = true;
    interconnect::Msg<MSG_TYPE> data;
    sc_lv<dest_count> dest_bits = 0;

#pragma hls_pipeline_init_interval 1
#pragma pipeline_stall_mode flush
    while (1) {
      wait();

      if (all_sent) {
        interconnect::Msg<MSG_TYPE> in_data;
        CDCOUT(sc_time_stamp() << " " << name() << " LinkSender - "
                               << MSG_TYPE::name << " Checking input port\n",
               kDebugLevel);
        if (in_port.PopNB(in_data)) {
          data = in_data;
          dest_bits = data.dest_bits;
          CDCOUT(sc_time_stamp() << " " << name() << " LinkSender - "
                                 << MSG_TYPE::name << " Popped data: "
                                 << ((typename MSG_TYPE::data_t)data) << endl,
                 kDebugLevel);
          all_sent = false;
        }
      }

// Push and update.
#pragma hls_unroll yes
      for (int i = 0; i < dest_count; ++i) {
        if (dest_bits[i] == 1) {
          if (!out_ports[i].Full()) {
            out_ports[i].Push(data);
            CDCOUT(sc_time_stamp() << " " << name() << " LinkSender - "
                                   << MSG_TYPE::name << " Dest: " << i
                                   << " FIFO push output data: "
                                   << ((typename MSG_TYPE::data_t)data) << endl,
                   kDebugLevel);
            dest_bits[i] = 0;
          }
        }
      }

      // End condition
      all_sent = (dest_bits == 0);

#pragma hls_unroll yes
      for (int i = 0; i < dest_count; ++i) {
        out_ports[i].TransferNB();
      }
    }
  }
};

//#pragma hls_ungroup
template <typename MSG_TYPE, int src_count,
          Connections::connections_port_t PortType = AUTO_PORT>
class LinkReceiver : public sc_module {
  SC_HAS_PROCESS(LinkReceiver);

 protected:
  Connections::CombinationalBufferedPorts<interconnect::Msg<MSG_TYPE>, 0, 1>
      out_port;

 public:
  sc_in_clk clk;
  sc_in<bool> rst;
  Connections::InBuffered<interconnect::Msg<MSG_TYPE>, 1, PortType>
      in_ports[src_count];

  LinkReceiver()
      : sc_module(sc_module_name(sc_gen_unique_name("link_receiver"))) {
    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst, false);
  }

  void Bind(Connections::InBlocking<interconnect::Msg<MSG_TYPE>> &p) {
    p(out_port);
  }

  void run() {
    Arbiter<src_count> arb;

    out_port.ResetWrite();

#pragma hls_unroll yes
    for (int i = 0; i < src_count; ++i) {
      in_ports[i].Reset();
    }

    interconnect::Msg<MSG_TYPE> data;

#pragma hls_pipeline_init_interval 1
#pragma pipeline_stall_mode flush
    while (1) {
      wait();

#pragma hls_unroll yes
      for (int i = 0; i < src_count; ++i) {
        in_ports[i].TransferNB();
      }

      NVUINTW(src_count) valid, select;
      NVUINTW(nvhls::index_width<src_count>::val) select_id;
#pragma hls_unroll yes
      for (int i = 0; i < src_count; i++) {
        valid[i] = !in_ports[i].Empty();
      }
      select = arb.pick(valid);
      one_hot_to_bin<src_count, nvhls::index_width<src_count>::val>(select,
                                                                    select_id);
      if ((valid != 0) && !out_port.FullWrite()) {
        out_port.Push(in_ports[select_id].Pop());
      }

      out_port.TransferNBWrite();
    }
  }
};

// Stub specialization
//#pragma hls_ungroup
template <typename MSG_TYPE, Connections::connections_port_t PortType>
class LinkReceiver<MSG_TYPE, 0, PortType> : public sc_module {
  SC_HAS_PROCESS(LinkReceiver);

 protected:
  Connections::CombinationalBufferedPorts<interconnect::Msg<MSG_TYPE>, 0, 1>
      out_port;

 public:
  sc_in_clk clk;
  sc_in<bool> rst;

  LinkReceiver()
      : sc_module(sc_module_name(sc_gen_unique_name("link_receiver"))) {}

  void Bind(Connections::InBlocking<interconnect::Msg<MSG_TYPE>> &p) {
    p(out_port);
  }
};

//#pragma hls_ungroup
template <typename MSG_TYPE, int src_count,
          Connections::connections_port_t PortType = AUTO_PORT>
class LinkReceiverPort : public sc_module {
  SC_HAS_PROCESS(LinkReceiverPort);

 public:
  sc_in_clk clk;
  sc_in<bool> rst;
  Connections::OutBuffered<interconnect::Msg<MSG_TYPE>, 1> out_port;
  Connections::InBuffered<interconnect::Msg<MSG_TYPE>, 1, PortType>
      in_ports[src_count];

  LinkReceiverPort()
      : sc_module(sc_module_name(sc_gen_unique_name("link_receiver"))) {
    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst, false);
  }

  void Bind(Connections::OutBlocking<interconnect::Msg<MSG_TYPE>> &p) {
    out_port(p);
  }

  void run() {
    Arbiter<src_count> arb;

    out_port.Reset();

#pragma hls_unroll yes
    for (int i = 0; i < src_count; ++i) {
      in_ports[i].Reset();
    }

    interconnect::Msg<MSG_TYPE> data;

#pragma hls_pipeline_init_interval 1
#pragma pipeline_stall_mode flush
    while (1) {
      wait();

#pragma hls_unroll yes
      for (int i = 0; i < src_count; ++i) {
        in_ports[i].TransferNB();
      }

      NVUINTW(src_count) valid, select;
      NVUINTW(nvhls::index_width<src_count>::val) select_id;
#pragma hls_unroll yes
      for (int i = 0; i < src_count; i++) {
        valid[i] = !in_ports[i].Empty();
      }
      select = arb.pick(valid);
      one_hot_to_bin<src_count, nvhls::index_width<src_count>::val>(select,
                                                                    select_id);
      if ((valid != 0) && !out_port.Full()) {
        out_port.Push(in_ports[select_id].Pop());
      }

      out_port.TransferNB();
    }
  }
};

//#pragma hls_ungroup
template <typename MSG_TYPE, unsigned int num_srcs, unsigned int num_dests,
          unsigned int LenInputBuffer, unsigned int LenOutputBuffer,
          Connections::connections_port_t PortType = AUTO_PORT>
class CrossbarICTop : public sc_module {
  SC_HAS_PROCESS(CrossbarICTop);

 public:
  sc_in_clk clk;
  sc_in<bool> rst;

  Connections::In<interconnect::Msg<MSG_TYPE>, PortType> in_ports[num_srcs];
  Connections::OutBuffered<interconnect::Msg<MSG_TYPE>, 1, PortType>
      out_ports[num_dests];

  ArbitratedCrossbar<interconnect::Msg<MSG_TYPE>, num_srcs, num_dests,
                     LenInputBuffer, LenOutputBuffer> arbxbar;

  CrossbarICTop() : sc_module(sc_module_name(sc_gen_unique_name("crossbar"))) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void run() {
#pragma hls_unroll yes
    for (int inp_lane = 0; inp_lane < num_srcs; inp_lane++) {
      in_ports[inp_lane].Reset();
    }

#pragma hls_unroll yes
    for (int out_lane = 0; out_lane < num_dests; out_lane++) {
      out_ports[out_lane].Reset();
    }

#pragma hls_pipeline_init_interval 1
    while (1) {
      wait();

      interconnect::Msg<MSG_TYPE> data_in_reg[num_srcs];
      NVUINTW(nvhls::index_width<num_dests>::val) dest_in_reg[num_srcs];
      bool valid_in_reg[num_srcs];

#pragma hls_unroll yes
      for (int inp_lane = 0; inp_lane < num_srcs; inp_lane++) {
        if (!arbxbar.isInputFull(inp_lane) && LenInputBuffer > 0) {
          valid_in_reg[inp_lane] =
              in_ports[inp_lane].PopNB(data_in_reg[inp_lane]);
          sc_lv<num_dests> v = data_in_reg[inp_lane].get_dest_insts();

          dest_in_reg[inp_lane] =
              0;  // FIXME: get the destination from the packet.

#pragma hls_unroll yes
          for (int i = 0; i < num_dests; ++i) {
            if (v[i] == 1) {
              dest_in_reg[inp_lane] = i;
              break;
            }
          }
        } else {
          valid_in_reg[inp_lane] = false;
        }
      }

      // Outputs from ArbitratedCrossbar
      interconnect::Msg<MSG_TYPE> data_out_reg[num_dests];
      bool valid_out_reg[num_dests];
      bool ready_reg[num_srcs];

      arbxbar.run(data_in_reg, dest_in_reg, valid_in_reg, data_out_reg,
                  valid_out_reg, ready_reg);

// Push only the valid outputs.
#pragma hls_unroll yes
      for (int out_lane = 0; out_lane < num_dests; out_lane++) {
        if (valid_out_reg[out_lane]) {
          if (!out_ports[out_lane].Full()) {
            out_ports[out_lane].Push(arbxbar.pop(out_lane));
          }
        }
        out_ports[out_lane].TransferNB();
      }
    }
  }
};

// We have a couple options for handling destinations:
// 1. UnionMsg type only has a "dest_bit" of the type message,
// the subtype with the partition dest is has to be unmarshalled out.
// However, may not be too bad for bit swizzling?
//
// 2. Instead create a union destination based on number of unique partitions.
// Encode this along with message type. Probably necessary to route effectively,
// when we do serdes already
//
// 3. Or dest_bits is max of sub msg_bit types. msg is payload only portion of
// other bit types.
//
// This seems the most reasonable... we will need a per-msg dest map though
// hmmm.
template <unsigned int num_bits, unsigned int num_dests, unsigned int num_msgs>
class UnionMsgMulticast : public nvhls_message {
 public:
  static const unsigned int num_dests_ = num_dests;
  typedef sc_lv<num_bits> data_t;

  data_t msg_bits;
  sc_lv<num_dests> dest_bits;
  NVUINTW(nvhls::index_width<num_msgs>::val)
  msg_type_bits;  // FIXME: probably roll this into payload, so that it is Msg
                  // compatible.

  //// Constructor-Destructor
  UnionMsgMulticast<num_bits, num_dests, num_msgs>() { dest_bits = 0; }
  UnionMsgMulticast<num_bits, num_dests, num_msgs>(const data_t &m) {
    dest_bits = 0;
    set_msg(m);
  }

  virtual inline unsigned int dest_map(const Destination &dest) {
    NVHLS_ASSERT(0);
    return 0;
  }

  // Borrows heavily from SCLV mechanics, given a
  // marshable type <= num_dests and num_msg_bits
  // add it to the message and dest.
  template <typename Message>
  void __set_msg(const interconnect::Msg<Message> &msg_) {
    NVHLS_ASSERT_MSG(Wrapped<typename Message::data_t>::width <= num_bits,
                     "Other message exceeded supported bits");
    NVHLS_ASSERT_MSG(interconnect::Msg<Message>::dest_count <= num_dests,
                     "Other message exceeded supported dests");

    // Convert from Message to general sc_lv type
    Marshaller<Wrapped<typename Message::data_t>::width> marshaller;
    Wrapped<typename Message::data_t> wm(msg_.get_msg());
    wm.Marshall(marshaller);
    msg_bits = 0;
    msg_bits.range(Wrapped<typename Message::data_t>::width - 1, 0) =
        marshaller.GetResult();
  }

  template <typename Message>
  void __get_msg(interconnect::Msg<Message> &m_) {
    NVHLS_ASSERT_MSG(Wrapped<typename Message::data_t>::width <= num_bits,
                     "Other message exceeded supported bits");
    NVHLS_ASSERT_MSG(interconnect::Msg<Message>::dest_count <= num_dests,
                     "Other message exceeded supported dests");

    // Convert from general sc_lv type to Message
    sc_lv<Wrapped<typename Message::data_t>::width> mbits;
    mbits = msg_bits.range(Wrapped<typename Message::data_t>::width - 1, 0);
    Marshaller<Wrapped<typename Message::data_t>::width> marshaller(mbits);
    Wrapped<typename Message::data_t> result;
    result.Marshall(marshaller);
    m_ = result.val;
  }

  //// Marshall support
  enum {
    width = Wrapped<data_t>::width + Wrapped<sc_lv<num_dests>>::width +
            Wrapped<NVUINTW(nvhls::index_width<num_msgs>::val)>::width
  };

  template <unsigned int Size>
  void Marshall(Marshaller<Size> &m) {
    m &msg_bits;
    m &dest_bits;
    m &msg_type_bits;
  }
};
};

#endif  // ifndef __INTERCONNECT_GEN_UTILS_HPP__
