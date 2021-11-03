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

#ifndef __INTERCONNECT_HANDLER_FIXED_H__
#define __INTERCONNECT_HANDLER_FIXED_H__

#include "Interconnect.hpp"

namespace interconnect {

#ifdef CONNECTIONS_SIM_ONLY

// CRTP Mix-in to handle fixed ports
template <typename IC, typename IM>
class InterconnectHandlerFixed {

 protected:
  // Declarations
  class FixedSrcHandlerIface;
  class FixedDestHandlerIface;

 public:
  typedef IC interconnect_t;
  typedef IM IM_t;
  typedef Connections::OutBlocking<IM> IM_src_t;
  typedef Connections::InBlocking<IM> IM_dest_t;

  std::map<Connections::Blocking_abs *, FixedSrcHandlerIface *> fixed_srcs;
  std::map<Connections::Blocking_abs *, FixedDestHandlerIface *> fixed_dests;

  template <typename Message>
  void BindFixed(Connections::OutBlocking<Message> &p) {
    interconnect_t &ic = static_cast<interconnect_t &>(*this);

    FixedSrcHandler<Message> *is = new FixedSrcHandler<Message>(ic, p);
    ic.template Bind<Message>(is->im);
    fixed_srcs[&p] = is;

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }
    ic.set_src_name(&is->im, port_name);
  }

  template <typename Message>
  void BindFixed(Connections::InBlocking<Message> &p) {
    interconnect_t &ic = static_cast<interconnect_t &>(*this);

    FixedDestHandler<Message> *ij = new FixedDestHandler<Message>(ic, p);
    ic.template Bind<Message>(ij->im);
    fixed_dests[&p] = ij;

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }
    ic.set_dest_name(&ij->im, port_name);
  }

  template <typename Message>
  void set_fixed(Connections::OutBlocking<Message> &s,
                 Connections::InBlocking<Message> &d) {
    interconnect_t &ic = static_cast<interconnect_t &>(*this);

    NVHLS_ASSERT(ic.find_src_idx(fixed_srcs[&s]->im_ptr) != -1);
    NVHLS_ASSERT(ic.find_dest_idx(fixed_dests[&d]->im_ptr) != -1);

    NVHLS_ASSERT(fixed_srcs.find(&s) != fixed_srcs.end() && fixed_srcs[&s]);
    NVHLS_ASSERT(fixed_dests.find(&d) != fixed_dests.end() && fixed_dests[&d]);

    fixed_srcs[&s]->src_idx = ic.find_src_idx(fixed_srcs[&s]->im_ptr);
    fixed_srcs[&s]->dest_idx = ic.find_dest_idx(fixed_dests[&d]->im_ptr);

    fixed_dests[&d]->src_idx = ic.find_src_idx(fixed_srcs[&s]->im_ptr);
    fixed_dests[&d]->dest_idx = ic.find_dest_idx(fixed_dests[&d]->im_ptr);
  }

 protected:
  // Interface for handler that is template-less
  struct FixedSrcHandlerIface {
   public:
    typename interconnect_t::IM_src_t *im_ptr;
    typename interconnect_t::idx_t src_idx;
    typename interconnect_t::idx_t dest_idx;
  };

  // class FixedSrcHandler
  template <typename Message>
  class FixedSrcHandler : public sc_module,
                          public Connections::Blocking_abs,
                          public FixedSrcHandlerIface {
    SC_HAS_PROCESS(FixedSrcHandler);

   public:
    interconnect_t &ic;

    typename interconnect_t::IM_src_t im;
    Connections::OutBlocking<Message> &p;
    sc_signal<sc_lv<Wrapped<Message>::width> > int_msg;
    sc_signal<bool> int_rdy;
    sc_signal<bool> int_val;

    explicit FixedSrcHandler(interconnect_t &ic_,
                             Connections::OutBlocking<Message> &p_)
        : sc_module(sc_module_name(sc_gen_unique_name("fixed_src_handler"))),
          ic(ic_),
          im(sc_gen_unique_name("im")),
          p(p_),
          int_msg("int_msg"),
          int_rdy("int_rdy"),
          int_val("int_val") {
      // Pass up to handler interface
      this->im_ptr = &im;

      p.msg(int_msg);
      p.rdy(int_rdy);
      p.val(int_val);

#ifdef CONNECTIONS_SIM_ONLY
      // SC_METHOD(do_bypass);
      declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                             SC_CURRENT_USER_MODULE, do_bypass);
      this->sensitive << p.msg << p.val << im.rdy;

      im.disable_spawn();
#endif
    }

    void do_bypass() {
      bool is_val = int_val.read();

      // Set val and msg respectively
      im.val.write(is_val);
      int_rdy.write(im.rdy.read());

      // Only convert if is_val (to prevent X's)
      if (is_val) {
        // Convert msg
        Marshaller<Wrapped<Message>::width> marshaller(int_msg.read());
        Wrapped<Message> result;
        result.Marshall(marshaller);

        // Create the InterconnectMessage
        typename interconnect_t::IM_t im_msg;
        Message m = result.val;
        im_msg.set_msg(ic, m);
        im_msg.set_src_idx(ic, ic.get_src_from_idx(this->src_idx));

        typename interconnect_t::route_t *current_route = ic.create_route();
        current_route->add_dest(this->dest_idx);
        im_msg.set_route(ic, current_route);
        delete current_route;

        // Convert back to Marshall'd form
        Marshaller<Wrapped<typename interconnect_t::IM_t>::width> im_marshaller;
        Wrapped<typename interconnect_t::IM_t> im_wm(im_msg);
        im_wm.Marshall(im_marshaller);
        im.msg.write(im_marshaller.GetResult());
      }
    }
  };  // class FixedSrcHandler

  // Interface for handler that is template-less
  class FixedDestHandlerIface {
   public:
    typename interconnect_t::IM_dest_t *im_ptr;
    typename interconnect_t::idx_t src_idx;
    typename interconnect_t::idx_t dest_idx;
  };

  // class FixedDestHandler
  template <typename Message>
  class FixedDestHandler : public sc_module,
                           public Connections::Blocking_abs,
                           public FixedDestHandlerIface {
    SC_HAS_PROCESS(FixedDestHandler);

   public:
    interconnect_t &ic;
    typename interconnect_t::IM_dest_t im;
    Connections::InBlocking<Message> &p;
    sc_signal<sc_lv<Wrapped<Message>::width> > int_msg;
    sc_signal<bool> int_rdy;
    sc_signal<bool> int_val;

    explicit FixedDestHandler(interconnect_t &ic_,
                              Connections::InBlocking<Message> &p_)
        : sc_module(sc_module_name(sc_gen_unique_name("interconnect_joiner"))),
          ic(ic_),
          im(sc_gen_unique_name("im")),
          p(p_),
          int_msg("int_msg"),
          int_rdy("int_rdy"),
          int_val("int_val") {
      // Pass up to handler interface
      this->im_ptr = &im;

      p.msg(int_msg);
      p.rdy(int_rdy);
      p.val(int_val);

#ifdef CONNECTIONS_SIM_ONLY
      // SC_METHOD(do_bypass);
      declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                             SC_CURRENT_USER_MODULE, do_bypass);
      this->sensitive << im.msg << im.val << p.rdy;

      im.disable_spawn();
#endif
    }

    void do_bypass() {
      bool is_val = im.val.read();

      // Set val and msg respectively
      int_val.write(is_val);
      im.rdy.write(int_rdy.read());

      // Only convert if is_val (to prevent X's)
      if (is_val) {
        // Convert msg from bits
        Marshaller<Wrapped<typename interconnect_t::IM_t>::width> im_marshaller(
            im.msg.read());
        Wrapped<typename interconnect_t::IM_t> im_result;
        im_result.Marshall(im_marshaller);
        typename interconnect_t::IM_t im_msg = im_result.val;

        // Convert msg to bits and write
        Marshaller<Wrapped<Message>::width> marshaller;
        Message m;
        im_msg.get_msg(ic, m);
        Wrapped<Message> wm(m);
        wm.Marshall(marshaller);
        int_msg.write(marshaller.GetResult());
      }
    }
  };  // classFixedDestHandler

};  // class InterconnecHandlerFixed

#endif
};

#endif  // __INTERCONNECT_HANDLER_FIXED_H__
