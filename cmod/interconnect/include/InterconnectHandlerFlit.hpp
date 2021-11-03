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

#ifndef __INTERCONNECT_HANDLER_FLIT_H__
#define __INTERCONNECT_HANDLER_FLIT_H__

#include "Interconnect.hpp"

#include <nvhls_packet.h>

namespace interconnect {

#ifdef CONNECTIONS_SIM_ONLY
// CRTP Mix-in to handle fixed ports
template <typename IC, typename IM>
class InterconnectHandlerFlit {

 protected:
  // Declarations
  class FlitSrcHandlerIface;
  class FlitDestHandlerIface;

 public:
  typedef IC interconnect_t;
  typedef IM IM_t;
  typedef Connections::OutBlocking<IM> IM_src_t;
  typedef Connections::InBlocking<IM> IM_dest_t;

  typedef typename InterconnectTypeConfig<IM>::idx_t idx_t;
  typedef typename InterconnectTypeConfig<IM>::dest_map_idx_t dest_map_idx_t;

  std::map<Connections::Blocking_abs *, FlitSrcHandlerIface *> flit_srcs;
  std::map<Connections::Blocking_abs *, FlitDestHandlerIface *> flit_dests;

  template <typename MSG_TYPE, typename Flit_t,
            Connections::connections_port_t PortType>
  void BindFlit(Connections::OutBlocking<Flit_t, PortType> &p) {
    interconnect_t &ic = static_cast<interconnect_t &>(*this);

    auto *is =
        new FlitSrcHandler<Flit_t, typename MSG_TYPE::router_t, PortType>(
            ic, p, MSG_TYPE::map_id);
    is->clk(ic.clk);
    is->rst(ic.rst);
    ic.template Bind<Flit_t>(is->im, MSG_TYPE::msg_id, MSG_TYPE::name);
    flit_srcs[&p] = is;

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }
    ic.set_src_name(&is->im, port_name);
  }

  template <typename MSG_TYPE, typename Flit_t,
            Connections::connections_port_t PortType>
  void BindFlit(Connections::InBlocking<Flit_t, PortType> &p) {
    interconnect_t &ic = static_cast<interconnect_t &>(*this);

    auto *ij =
        new FlitDestHandler<Flit_t, typename MSG_TYPE::router_t, PortType>(
            ic, p, MSG_TYPE::map_id);
    ij->clk(ic.clk);
    ij->rst(ic.rst);
    ic.template Bind<Flit_t>(ij->im, MSG_TYPE::msg_id, MSG_TYPE::name);
    flit_dests[&p] = ij;

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }
    ic.set_dest_name(&ij->im, port_name);
  }

  template <typename MSG_TYPE, typename Flit_t,
            Connections::connections_port_t PortType>
  void BindFlit(Connections::OutBlocking<Flit_t, PortType> &p,
                unsigned int custom_id) {
    this->BindFlit<MSG_TYPE, Flit_t>(p);
    // No map for sources
  }

  template <typename MSG_TYPE, typename Flit_t,
            Connections::connections_port_t PortType>
  void BindFlit(Connections::InBlocking<Flit_t, PortType> &p,
                unsigned int custom_id) {
    interconnect_t &ic = static_cast<interconnect_t &>(*this);

    NVHLS_ASSERT_MSG(custom_id > 0,
                     "Must set a custom destination index for In flit ports!");
    ic.create_dest_map(MSG_TYPE::map_id, true);
    this->BindFlit<MSG_TYPE, Flit_t>(p);
    this->config_dest_map_idx_flit(MSG_TYPE::map_id, custom_id, p);
  }

  template <typename Flit_t, Connections::connections_port_t PortType>
  void config_dest_map_idx_flit(dest_map_idx_t dest_map_idx, idx_t dest_idx,
                                Connections::InBlocking<Flit_t, PortType> &p) {
    interconnect_t &ic = static_cast<interconnect_t &>(*this);

    ic.config_dest_map_idx(
        dest_map_idx, dest_idx,
        *(flit_dests[(Connections::Blocking_abs *)&p]->im_ptr));
  }

 protected:
  // Interface for handler that is template-less
  struct FlitSrcHandlerIface {
   public:
    typename interconnect_t::IM_src_t *im_ptr;
  };

  template <typename Flit_t, typename router_t,
            Connections::connections_port_t PortType>
  class FlitSrcHandler : public sc_module,
                         public Connections::Blocking_abs,
                         public FlitSrcHandlerIface {
    SC_HAS_PROCESS(FlitSrcHandler);

   protected:
    static const int BUFFERSIZE = 17;

    dest_map_idx_t dest_map_idx;

    // Local State
    sc_lv<Wrapped<Flit_t>::width * BUFFERSIZE> packet_buf;
    unsigned int packet_buf_pos;
    FIFO<NVUINT5, BUFFERSIZE, 1> num_flit_fifo;
    NVUINT5 num_flits;
    typename interconnect_t::route_t *current_route;

   public:
    sc_in_clk clk;
    sc_in<bool> rst;

    interconnect_t &ic;

    typename interconnect_t::IM_src_t im;
    Connections::OutBlocking<Flit_t, PortType> &p;
    Connections::Combinational<Flit_t> int_chan;

    explicit FlitSrcHandler(interconnect_t &ic_,
                            Connections::OutBlocking<Flit_t, PortType> &p_,
                            dest_map_idx_t dest_map_idx_)
        : sc_module(sc_module_name(sc_gen_unique_name("flit_src_handler"))),
          dest_map_idx(dest_map_idx_),
          packet_buf_pos(0),
          clk("clk"),
          rst("rst"),
          ic(ic_),
          im(sc_gen_unique_name("im")),
          p(p_),
          int_chan("int_chan") {
      // Pass up to handler interface
      this->im_ptr = &im;

      p(int_chan);

      SC_THREAD(flit_src_handler_run);
      sensitive << clk.pos();
      async_reset_signal_is(rst, false);
    }

   protected:
    void FillInFifo() {
      Flit_t noc_flit;
      if (int_chan.PopNB(noc_flit)) {
        num_flits++;
        bool is_head_flit =
            noc_flit.flit_id.isHeader() || noc_flit.flit_id.isSingle();
        bool is_tail_flit =
            noc_flit.flit_id.isTail() || noc_flit.flit_id.isSingle();

        Marshaller<Wrapped<Flit_t>::width> flit_marshaller;
        Wrapped<Flit_t> flit_wm(noc_flit);
        flit_wm.Marshall(flit_marshaller);
        packet_buf.range(Wrapped<Flit_t>::width + packet_buf_pos - 1,
                         packet_buf_pos) = flit_marshaller.GetResult();
        packet_buf_pos += Wrapped<Flit_t>::width;

        // Decode the destination
        if (is_head_flit) {
          bool is_mcast = noc_flit.data[router_t::mcast_dest_width];
          if (is_mcast) {
            // Multicast
            // Extract different local and global destinations
            NVUINTW(router_t::mcast_dest_width)
            route =
                nvhls::get_slc<router_t::mcast_dest_width>(noc_flit.data, 0);
            NVUINTW(router_t::NoC_mcast_dest_width)
            NoC_out_dest_local = nvhls::get_slc<router_t::NoC_mcast_dest_width>(
                route, router_t::NoP_mcast_dest_width);

            current_route = ic.create_route();
            for (int i = 0; i < router_t::NoC_mcast_dest_width; i++) {
              if (NoC_out_dest_local[i]) {
                current_route->add_dest(i, dest_map_idx);
              }
            }
          } else {
            // Unicast
            // Extract different local and global destinations
            NVUINTW(router_t::ucast_dest_width)
            route =
                nvhls::get_slc<router_t::ucast_dest_width>(noc_flit.data, 0);
            NVUINTW(router_t::NoC_ucast_dest_width)
            NoC_out_dest_local =
                nvhls::get_slc<router_t::NoC_ucast_dest_width>(route, 0);

            current_route = ic.create_route();
            current_route->add_dest(NoC_out_dest_local.to_uint64(),
                                    dest_map_idx);
          }
        }

        if (is_tail_flit) {
          num_flit_fifo.push(num_flits, 0);
          num_flits = 0;
        }
      }
    }

    void flit_src_handler_run() {
      int_chan.ResetRead();
      im.Reset();
      packet_buf = 0;
      packet_buf_pos = 0;
      num_flits = 0;
      current_route = 0;

      while (1) {
        wait();
        FillInFifo();
        if (!num_flit_fifo.isEmpty(0)) {

          // Create the InterconnectMessage
          typename interconnect_t::IM_t im_msg;
          im_msg.set_msg(ic, packet_buf);

          im_msg.set_route(ic, current_route);

          // Push it onto the interface
          im.Push(im_msg);

          // Reset for next packet
          num_flit_fifo.incrHead(0);
          packet_buf = 0;
          packet_buf_pos = 0;
        }
      }
    }
  };  // class FlitSrcHandler

  // Interface for handler that is template-less
  class FlitDestHandlerIface {
   public:
    typename interconnect_t::IM_dest_t *im_ptr;
  };

  // class FlitDestHandler
  template <typename Flit_t, typename router_t,
            Connections::connections_port_t PortType>
  class FlitDestHandler : public sc_module,
                          public Connections::Blocking_abs,
                          public FlitDestHandlerIface {
    SC_HAS_PROCESS(FlitDestHandler);

   protected:
    static const int BUFFERSIZE = 17;

    dest_map_idx_t dest_map_idx;

    // Local State
    sc_lv<Wrapped<Flit_t>::width * BUFFERSIZE> packet_buf;
    unsigned int packet_buf_pos;

   public:
    sc_in_clk clk;
    sc_in<bool> rst;

    interconnect_t &ic;
    typename interconnect_t::IM_dest_t im;
    Connections::InBlocking<Flit_t, PortType> &p;
    Connections::Combinational<Flit_t> int_chan;

    explicit FlitDestHandler(interconnect_t &ic_,
                             Connections::InBlocking<Flit_t, PortType> &p_,
                             dest_map_idx_t dest_map_idx_)
        : sc_module(sc_module_name(sc_gen_unique_name("interconnect_joiner"))),
          dest_map_idx(dest_map_idx_),
          clk("clk"),
          rst("rst"),
          ic(ic_),
          im(sc_gen_unique_name("im")),
          p(p_),
          int_chan("int_chan") {
      // Pass up to handler interface
      this->im_ptr = &im;

      p(int_chan);

      SC_THREAD(flit_dest_handler_run);
      sensitive << clk.pos();
      async_reset_signal_is(rst, false);
    }

   protected:
    void flit_dest_handler_run() {
      int_chan.ResetWrite();
      im.Reset();
      packet_buf = 0;
      packet_buf_pos = 0;

      while (1) {
        wait();
        if (packet_buf_pos == 0) {
          typename interconnect_t::IM_t im_msg;
          im_msg = im.Pop();
          im_msg.get_msg(ic, packet_buf);
        }

        Flit_t flit;
        Marshaller<Wrapped<Flit_t>::width> flit_marshaller(packet_buf.range(
            Wrapped<Flit_t>::width + packet_buf_pos - 1, packet_buf_pos));
        Wrapped<Flit_t> flit_result;
        flit_result.Marshall(flit_marshaller);
        flit = flit_result.val;

        int_chan.Push(flit);

        if (flit.flit_id.isTail() || flit.flit_id.isSingle()) {
          packet_buf_pos = 0;
        } else {
          packet_buf_pos += Wrapped<Flit_t>::width;
        }
      }
    }
  };  // classFlitDestHandler

};  // class InterconnecHandlerFlit

#endif
};

#endif  // __INTERCONNECT_HANDLER_FLIT_H__
