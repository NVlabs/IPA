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

/*! \file Interconnect.hpp
    \brief Primary IPA header file
*/

#ifndef __INTERCONNECT_H__
#define __INTERCONNECT_H__

#include <systemc.h>
#include <nvhls_connections.h>
#include <nvhls_message.h>

#ifndef __SYNTHESIS__
#include <boost/format.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/functional/hash.hpp>
#include <typeinfo>
#endif

#include <boost/preprocessor/facilities/overload.hpp>

// Channel type class.
#include "InterconnectChannels.hpp"

// Include more global type settings.
// Needed before defining CRTP mix-in's
#include "InterconnectTypeConfig.hpp"

// Annotate and others depend on this
#include "InterconnectPre.hpp"

// Include our own annotation class
#include "InterconnectAnnotate.hpp"

// CRTP Mix-in
#include "InterconnectHandlerFixed.hpp"
#include "InterconnectHandlerFlit.hpp"

/*! \def ic_header(s)
    \brief Required include macro header.
    \param s Should always be set to "INTERCONNECT_GEN"

    Only use at the top of the *** file:
    \code
      #include ic_header(INTERCONNECT_GEN)
    \endcode
*/
#define ic_header(s) __ic_str(s)
#define __ic_str(s) #s

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interconnect macros
////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Defer resolution, to be used with __COUNTER__ so we get unique names.
#define __IC_MACRO_DEFERED2(x) x
#define __IC_MACRO_DEFERED(x) __IC_MACRO_DEFERED2(x)

//// Configuration-related macros

/*!
  \def IC_BEGIN_INTERCONNECTS
  \brief Close the interconnect object declarations.
*/
#define IC_BEGIN_INTERCONNECTS struct interconnects {
/*!
  \def IC_ADD_INTERCONNECT(x)
  \brief Declare an interconnect with name \a x.

   \param x identifier for new interconnect
*/
#define IC_ADD_INTERCONNECT(x)                        \
  struct x {                                          \
    enum { ic_id = __IC_MACRO_DEFERED(__COUNTER__) }; \
  };
/*!
  \def IC_END_INTERCONNECTS
  \brief Close the interconnect object declarations.
*/
#define IC_END_INTERCONNECTS \
  }                          \
  ;

// +1 below so that it can never be 0
/*!
  \def IC_BEGIN_DEST_MAPS
  \brief Declare custom destination index maps

  \par For now, custom maps are unsupported, but these macros are still required
  to create the global map:

  \code
    IC_BEGIN_DEST_MAPS
    IC_END_DEST_MAPS
  \endcode
*/
#define IC_BEGIN_DEST_MAPS \
  struct maps {            \
    struct global {        \
      enum { map_id = 0 }; \
    };
#define IC_ADD_DEST_MAP(x)                                   \
  struct x {                                                 \
    enum { map_id = (__IC_MACRO_DEFERED(__COUNTER__) + 1) }; \
  };
/*!
  \def IC_END_DEST_MAPS
  \brief End declarations custom destination index maps

  \par For now, custom maps are unsupported, but these macros are still required
  to create the global map:

  \code
    IC_BEGIN_DEST_MAPS
    IC_END_DEST_MAPS
  \endcode
*/
#define IC_END_DEST_MAPS \
  }                      \
  ;

/*!
  \def IC_BEGIN_PARTITION_TYPES
  \brief Begin list of declared partitions for IPA.
*/
#define IC_BEGIN_PARTITION_TYPES struct parts {
/*!
  \def IC_BEGIN_PARTITION(x)
  \brief Declare a partition with name \a x.

   \param x identifier for new partition
*/
#define IC_BEGIN_PARTITION(x)               \
  struct x {                                \
    enum { part_id = __COUNTER__ };         \
    static constexpr const char *name = #x; \
    struct ports {
//#define IC_ADD_PORT(x) struct x { enum { port_id =
//__IC_MACRO_DEFERED(__COUNTER__) }; };
/*!
  \def IC_END_PARTITION
  \brief Close the interconnect partition declarations.
*/
#define IC_END_PARTITION \
  }                      \
  ;                      \
  }                      \
  ;
/*!
  \def IC_END_PARTITION_TYPES
  \brief End list of declared partitions for IPA.
*/
#define IC_END_PARTITION_TYPES \
  }                            \
  ;

/*!
  \def IC_BEGIN_MESSAGE_TYPES
  \brief Begin list of declared message types for IPA.
*/
#define IC_BEGIN_MESSAGE_TYPES struct msgs {
#define IC_ADD_MESSAGE_TYPE_3(x, d, z)          \
  struct x {                                    \
    enum {                                      \
      msg_id = __IC_MACRO_DEFERED(__COUNTER__), \
      map_id = maps::z::map_id                  \
    };                                          \
    typedef d data_t;                           \
    static constexpr const char *name = #x;     \
  };
#define IC_ADD_MESSAGE_TYPE_2(msg_type, msg_data) \
  IC_ADD_MESSAGE_TYPE_3(msg_type, msg_data, global)
/*!
  \def IC_ADD_MESSAGE_TYPE(...)
  \brief Declare a new message type.

  \param x User-specified identifier for message type
  \param d Payload (class or primitive) for message type
  \param map_id Optional parameter for custom destination map (currently
  unsupported)
*/
#define IC_ADD_MESSAGE_TYPE(...) \
  BOOST_PP_OVERLOAD(IC_ADD_MESSAGE_TYPE_, __VA_ARGS__)(__VA_ARGS__)

#define IC_ADD_MESSAGE_TYPE_FLIT(x, d, z)       \
  struct x {                                    \
    enum {                                      \
      msg_id = __IC_MACRO_DEFERED(__COUNTER__), \
      map_id = maps::z::map_id                  \
    };                                          \
    typedef d router_t;                         \
    static constexpr const char *name = #x;     \
  };
/*!
  \def IC_END_MESSAGE_TYPES
  \brief End list of declared message types for IPA.
*/
#define IC_END_MESSAGE_TYPES \
  }                          \
  ;

// Dest Map
// Currently not supported
#define IC_DEST_MAP(x) interconnect_config::maps::x::map_id
#define IC_PORT_MAP_SET_IDX(x, y, z) \
  ic_interface.set_map_port_idx(IC_DEST_MAP(x), y, z)

///////////////////////////////////////////////////////////////////////////////
//// Top-level macros
///////////////////////////////////////////////////////////////////////////////

/*!
  \def IC_HAS_INTERCONNECT(x)
  \brief Used within a top-level sc_module class declaration to indicate it
  contains interconnect \a x.

  \param x User-specified identifier for interconnect
*/
#define IC_HAS_INTERCONNECT(x)                                      \
  typedef interconnect::Interconnect<                               \
      interconnect_config::interconnects::x::ic_id> interconnect_t; \
  interconnect_t ic

/*!
  \def IC_PART_BIND(inst, part_id)
  \brief Bind partition instance \a inst and designate it as partition id \a
  part_id for interconnect object \at ic

  \par Used within the constructor of the top-level sc_module containing the
  interconnect as specified with IC_HAS_INTERCONNECT.

  \param inst partition instance
  \param part_id user-specified partition ID used as the destination ID by
  messsage sources to address this instance partition
*/
#define IC_PART_BIND(inst, part_id)                                   \
  interconnect::do_interconnect_to_interface_bind<__IC_MACRO_DEFERED( \
      __COUNTER__)>(ic, (inst).ic_interface, inst, part_id)
/*!
  \def IC_AT_PART_BIND(ic, inst, part_id)
  \brief Bind partition instance \a inst and designate it as partition id \a
  part_id for interconnect object \at ic

  \par Under normal circumstances, IC_PART_BIND should be used instead, and
  called directly within the constructor of the top-level sc_module containing
  the interconnect.
  However, this macro can be useful if the interconnect object sits elsewhere in
  the hierarchy, but may not be as well supported for interconect generation.

  \param ic interconnect object
  \param inst partition instance
  \param part_id user-specified partition ID used as the destination ID by
  messsage sources to address this instance partition
*/
#define IC_AT_PART_BIND(ic, inst, part_id)                            \
  interconnect::do_interconnect_to_interface_bind<__IC_MACRO_DEFERED( \
      __COUNTER__)>(ic, (inst).ic_interface, inst, part_id)

///////////////////////////////////////////////////////////////////////////////
//// Partition-level macros
///////////////////////////////////////////////////////////////////////////////

// Interface declaration
/*!
  \def IC_HAS_INTERFACE(x)
  \brief Used within a unit-level sc_module class declaration to indicate it
  contains an interface to bind to an interconnect.

  \param x User-specified identifier for partition
*/
#define IC_HAS_INTERFACE(x)                                              \
  typedef interconnect_config::parts::x part_info;                       \
  typedef interconnect::InterconnectInterface<part_info> ic_interface_t; \
  ic_interface_t ic_interface

// IC_MESSAGE ports
#define IC_PORT_BIND_1(port) \
  ic_interface.Bind<__IC_MACRO_DEFERED(__COUNTER__)>(port, 0)
#define IC_PORT_BIND_2(port, port_id) \
  ic_interface.Bind<__IC_MACRO_DEFERED(__COUNTER__)>(port, port_id)
/*!
  \def IC_PORT_BIND(port,port_id)
  \brief Bind a port of a lower level module within hierarchy

  \par For binding a port of the current-level of hierarchy, use IC_PIN_BIND
  instead.

  \param port Port instance to bind
  \param port_id Optional user-specified port_id for sharing multiple message
  types on the unit; currently unsupported
*/
#define IC_PORT_BIND(...) \
  BOOST_PP_OVERLOAD(IC_PORT_BIND_, __VA_ARGS__)(__VA_ARGS__)

#define IC_PIN_BIND_1(port) IC_PORT_BIND(interconnect::new_port_pin2(port)->pin)
#define IC_PIN_BIND_2(port, port_id) \
  IC_PORT_BIND(interconnect::new_port_pin2(port)->pin, port_id)
/*!
  \def IC_PIN_BIND(port,port_id)
  \brief Bind a port of current level module within hierarchy

  \par For binding a port of the lower level of hierarchy, use IC_PORT_BIND
  instead.

  \param port Port instance to bind
  \param port_id Optional user-specified port_id for sharing multiple message
  types on the unit; currently unsupported
*/
#define IC_PIN_BIND(...) \
  BOOST_PP_OVERLOAD(IC_PIN_BIND_, __VA_ARGS__)(__VA_ARGS__)

/// Flit ports

#define IC_AT_PORT_BIND_FLIT_3(ic, msg_type, port) \
  ic.BindFlit<interconnect_config::msgs::msg_type>(port);
#define IC_AT_PORT_BIND_FLIT_4(ic, msg_type, port, custom_id) \
  ic.BindFlit<interconnect_config::msgs::msg_type>(port, custom_id);
#define IC_AT_PORT_BIND_FLIT(...) \
  BOOST_PP_OVERLOAD(IC_AT_PORT_BIND_FLIT_, __VA_ARGS__)(__VA_ARGS__)

#define IC_AT_PIN_BIND_FLIT_3(ic, msg_type, port) \
  IC_AT_PORT_BIND_FLIT(ic, msg_type, interconnect::new_port_pin2(port)->pin)
#define IC_AT_PIN_BIND_FLIT_4(ic, msg_type, port, custom_id)                 \
  IC_AT_PORT_BIND_FLIT(ic, msg_type, interconnect::new_port_pin2(port)->pin, \
                       custom_id)
#define IC_AT_PIN_BIND_FLIT(...) \
  BOOST_PP_OVERLOAD(IC_AT_PIN_BIND_FLIT_, __VA_ARGS__)(__VA_ARGS__)

#define IC_PORT_BIND_FLIT(...) IC_AT_PORT_BIND_FLIT(ic, __VA_ARGS__)
#define IC_PIN_BIND_FLIT(...) IC_AT_PIN_BIND_FLIT(ic, __VA_ARGS__)

// To add additional port id's to a destination port
// Note: we don't use this in the bind3 version, since bind3 can accomodate in
// and out ports, and no way ot differentiate
#define IC_AT_PORT_ID_FLIT(ic, msg_type, port, custom_id)                  \
  ic.config_dest_map_idx_flit(interconnect_config::msgs::msg_type::map_id, \
                              custom_id, port)
#define IC_PORT_ID_FLIT(...) IC_AT_PORT_ID_FLIT(ic, __VA_ARGS__)

///////////////////////////////////////////////////////////////////////////////
//// Message-level macros
///////////////////////////////////////////////////////////////////////////////

// IC_PORT_ID is currently unused
#define IC_PORT_ID(y) part_info::ports::y::port_id
/*!
  \def IC_MESSAGE(x)
  \brief Create new message of message type \a x

  \param x message type identifier
*/
#define IC_MESSAGE(x) typename interconnect::Msg<interconnect_config::msgs::x>

// If the values can change at runtime e.g. from jtag.
#define IC_TO_DYNAMIC_1(part_id) interconnect::Destination(part_id, 0)
#define IC_TO_DYNAMIC_2(part_id, port_id) \
  interconnect::Destination(part_id, port_id)
#define IC_TO_DYNAMIC(...) \
  BOOST_PP_OVERLOAD(IC_TO_DYNAMIC_, __VA_ARGS__)(__VA_ARGS__)

// If the values are fixed/constant
#define IC_TO_CONST_1(part_id) interconnect::Destination(part_id, 0)
#define IC_TO_CONST_2(part_id, port_id) \
  interconnect::Destination(part_id, port_id)
#define IC_TO_CONST(...) \
  BOOST_PP_OVERLOAD(IC_TO_CONST_, __VA_ARGS__)(__VA_ARGS__)

/*!
  \def IC_TO(part_id,port_id)
  \brief Send a message to partition instance with id \a part_id

  \param part_id destination user-specified partition id
  \param port_id optional user-specified port_id; currently unsupported
*/
#define IC_TO(...) IC_TO_CONST(__VA_ARGS__)

///////////////////////////////////////////////////////////////////////////////
//// Legacy/testbench macros
///////////////////////////////////////////////////////////////////////////////

//// Legacy port bind for when we can't or don't want to use IC_INTERFACE
#define IC_AT_LEGACY_PORT_BIND_5(ic, msg_type, port, part_id, port_id) \
  ic.Bind(port);                                                       \
  ic.add_to_dest_directory(port, part_id, port_id)
#define IC_AT_LEGACY_PORT_BIND_4(ic, msg_type, port, part_id) \
  ic.Bind(port);                                              \
  ic.add_to_dest_directory(port, part_id)
#define IC_AT_LEGACY_PORT_BIND_3(ic, msg_type, port) ic.Bind(port);
#define IC_AT_LEGACY_PORT_BIND(...) \
  BOOST_PP_OVERLOAD(IC_AT_LEGACY_PORT_BIND_, __VA_ARGS__)(__VA_ARGS__)

#define IC_AT_LEGACY_DISTRIBUTED_PORT_BIND_5(ic, msg_type, port, part_id, \
                                             port_id)                     \
  ic.Bind(port);                                                          \
  ic.add_to_dest_directory(port, part_id, port_id, true)
#define IC_AT_LEGACY_DISTRIBUTED_PORT_BIND_4(ic, msg_type, port, part_id) \
  ic.Bind(port);                                                          \
  ic.add_to_dest_directory(port, part_id, 0, true)
#define IC_AT_LEGACY_DISTRIBUTED_PORT_BIND_3(ic, msg_type, port) ic.Bind(port);
#define IC_AT_LEGACY_DISTRIBUTED_PORT_BIND(...) \
  BOOST_PP_OVERLOAD(IC_AT_LEGACY_DISTRIBUTED_PORT_BIND_, __VA_ARGS__)(__VA_ARGS__)

#define IC_LEGACY_PORT_BIND(...) IC_AT_LEGACY_PORT_BIND(ic, __VA_ARGS__)
#define IC_LEGACY_DISTRIBUTED_PORT_BIND(...) \
  IC_AT_LEGACY_DISTRIBUTED_PORT_BIND(ic, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef __SYNTHESIS__

namespace interconnect {

/*!
  \brief Message instance class

  Do not instantiate directly, instead use the `#IC_MESSAGE(x)` macro

  \tparam MSG_TYPE identifier for message type
*/
template <typename MSG_TYPE>
class Msg : public nvhls_message {
  static const MSG_ID_t MSG_ID = MSG_TYPE::msg_id;
  typedef typename MSG_TYPE::data_t data_t;

 public:
  /*!
    \brief Payload data
  */
  data_t msg;
  /*!
    \brief Destination mask for this message
  */
  sc_lv<interconnect::MaxDestCount> dest_bits;

 public:
  //// Constructor-Destructor
  /*!
    \brief Default constructor
  */
  Msg<MSG_TYPE>() { dest_bits = 0; }

  /*!
    \brief Constructor to initialize from payload data
  */
  Msg<MSG_TYPE>(const data_t &m) {
    dest_bits = 0;
    set_msg(m);
  }

  //// Operators
  /*!
    \brief Set payload of message
  */
  Msg<MSG_TYPE> &operator=(const data_t &m) {
    set_msg(m);
    return *this;
  }

  /*!
    \brief Set message destination, destination id should be used through
    #IC_TO(part_id,port_id) macro
  */
  Msg<MSG_TYPE> &operator<<(const Destination &dest) {
    auto tuple_idx =
        std::make_tuple(dest.part_id, MSG_TYPE::msg_id, dest.port_id);
    if (interconnect::get_ICManager().dest_directory.find(tuple_idx) ==
        interconnect::get_ICManager().dest_directory.end()) {
      cout << dec << "Couldn't find destination part_id=" << dest.part_id
           << ", msg_id=" << MSG_TYPE::msg_id << ", port_id=" << dest.port_id
           << endl;
      NVHLS_ASSERT(0);
    }
    add_route(
        interconnect::get_ICManager().dest_directory[tuple_idx].get_next(), 0);

    return *this;
  }

  //// Setter-Getter

  /*!
    \brief Set message payload
  */
  void set_msg(const data_t &m) { msg = m; }

  /*!
    \brief Get message payload
  */
  data_t get_msg() { return msg; }

  /*!
    \brief Get message payload through cast
  */
  operator data_t() { return get_msg(); }

  /*!
    \brief Set destination for message
  */
  template <typename interconnect_t, typename route_t>
  void set_route(interconnect_t &ic, route_t *route) {
    dest_bits = 0;
    for (typename route_t::iterator it = route->begin(); it != route->end();
         ++it) {
      add_route(ic.find_dest_idx(*it));
    }
  }

  // Conversion to generic type
  template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
  operator InterconnectMessageSCLV<MaxDataWidth, MaxDestCount>() {
    assert(MaxDestCount == interconnect::MaxDestCount);
    assert(Wrapped<data_t>::width <= MaxDataWidth);

    InterconnectMessageSCLV<MaxDataWidth, MaxDestCount> new_im;
    new_im.set_msg(msg);
    new_im.dest_bits = dest_bits;
    return new_im;
  }
  template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
  explicit Msg<MSG_TYPE>(
      const InterconnectMessageSCLV<MaxDataWidth, MaxDestCount> &im) {
    assert(MaxDestCount == interconnect::MaxDestCount);
    assert(Wrapped<data_t>::width <= MaxDataWidth);
    im.get_msg(msg);
    dest_bits = im.dest_bits;
  }
  template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
  Msg<MSG_TYPE> &operator=(
      const InterconnectMessageSCLV<MaxDataWidth, MaxDestCount> &im) {
    assert(MaxDestCount == interconnect::MaxDestCount);
    assert(Wrapped<data_t>::width <= MaxDataWidth);
    im.get_msg(msg);
    dest_bits = im.dest_bits;
    return *this;
  }

  //// Route Operators
  void add_route(const unsigned int &dest_idx,
                 const unsigned int &dest_map_idx = 0) {
    unsigned int real_dest_idx;
    real_dest_idx = interconnect::get_real_dest_idx<void>(
        *interconnect::get_ICManager().registered_ic[0], dest_idx,
        dest_map_idx);
    dest_bits[real_dest_idx] = 1;
  }
  void clear_route() { dest_bits = 0; }

  //// Marshall support
  enum { width = Wrapped<data_t>::width + interconnect::MaxDestCount };

  template <unsigned int Size>
  void Marshall(Marshaller<Size> &m) {
    m &msg;
    m &dest_bits;
  }
};

template <typename IM_t>
struct GenericDestHandlerIface {
 public:
  typename Connections::InBlocking<IM_t> *im_ptr;
};

template <typename IM_t>
struct GenericSrcHandlerIface {
 public:
  typename Connections::OutBlocking<IM_t> *im_ptr;
};

template <typename IM_t, typename MSG_TYPE>
class ICMFromSpecialized : public sc_module,
                           public Connections::Blocking_abs,
                           public GenericSrcHandlerIface<IM_t> {
  SC_HAS_PROCESS(ICMFromSpecialized);

  static const interconnect::MSG_ID_t MSG_ID = MSG_TYPE::msg_id;

 public:
  Connections::OutBlocking<IM_t> im;
  Connections::OutBlocking<interconnect::Msg<MSG_TYPE> > &p;
  sc_signal<sc_lv<Wrapped<interconnect::Msg<MSG_TYPE> >::width> > int_msg;
  sc_signal<bool> int_rdy;
  sc_signal<bool> int_val;

  explicit ICMFromSpecialized(
      Connections::OutBlocking<interconnect::Msg<MSG_TYPE> > &p_)
      : sc_module(sc_module_name(sc_gen_unique_name("im_from_specific")))
        // ,ic(ic_)
        ,
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
      Marshaller<Wrapped<interconnect::Msg<MSG_TYPE> >::width> marshaller(
          int_msg.read());
      Wrapped<interconnect::Msg<MSG_TYPE> > result;
      result.Marshall(marshaller);

      // Create the InterconnectMessage
      IM_t im_msg;
      interconnect::Msg<MSG_TYPE> m = result.val;
      im_msg = m;

      // Convert back to Marshall'd form
      Marshaller<Wrapped<IM_t>::width> im_marshaller;
      Wrapped<IM_t> im_wm(im_msg);
      im_wm.Marshall(im_marshaller);
      im.msg.write(im_marshaller.GetResult());
    }
  }
};  // class ICMNewToOld

template <typename IM_t, typename MSG_TYPE>
class ICMToSpecialized : public sc_module,
                         public Connections::Blocking_abs,
                         public GenericDestHandlerIface<IM_t> {
  SC_HAS_PROCESS(ICMToSpecialized);

  static const interconnect::MSG_ID_t MSG_ID = MSG_TYPE::msg_id;

 public:
  Connections::InBlocking<IM_t> im;
  Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p;
  sc_signal<sc_lv<Wrapped<interconnect::Msg<MSG_TYPE> >::width> > int_msg;
  sc_signal<bool> int_rdy;
  sc_signal<bool> int_val;

  explicit ICMToSpecialized(
      Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p_)
      : sc_module(sc_module_name(sc_gen_unique_name("icm_to_specialized"))),
        // ic(ic_),
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
      Marshaller<Wrapped<IM_t>::width> im_marshaller(im.msg.read());
      Wrapped<IM_t> im_result;
      im_result.Marshall(im_marshaller);
      IM_t im_msg = im_result.val;

      // Convert msg to bits and write
      Marshaller<Wrapped<interconnect::Msg<MSG_TYPE> >::width> marshaller;
      interconnect::Msg<MSG_TYPE> m(im_msg);
      Wrapped<interconnect::Msg<MSG_TYPE> > wm(m);
      wm.Marshall(marshaller);
      int_msg.write(marshaller.GetResult());
    }
  }
};  // classFixedDestHandler
};

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace interconnect {

#ifdef CONNECTIONS_SIM_ONLY

// Pointer based message passing
template <typename IM>
class InterconnectBase
    : public sc_module,
      public InterconnectHandlerFixed<InterconnectBase<IM>, IM>,
      public InterconnectHandlerFlit<InterconnectBase<IM>, IM> {
  SC_HAS_PROCESS(InterconnectBase);

 protected:
  // Declarations
  class Route;

 public:
  typedef IM IM_t;
  typedef Route route_t;

  // These are now pushed down into a separate class, so that Handler classse
  // can access these without templatization. Is there a better way to do this?
  typedef typename InterconnectTypeConfig<IM>::IM_src_t IM_src_t;
  typedef typename InterconnectTypeConfig<IM>::IM_dest_t IM_dest_t;
  typedef typename InterconnectTypeConfig<IM>::IM_chan_t IM_chan_t;
  typedef typename InterconnectTypeConfig<IM>::IM_chanlink_t IM_chanlink_t;

  typedef typename InterconnectTypeConfig<IM>::srcs_t srcs_t;
  typedef typename InterconnectTypeConfig<IM>::dests_t dests_t;

  typedef
      typename InterconnectTypeConfig<IM>::channels_begin_t channels_begin_t;
  typedef typename InterconnectTypeConfig<IM>::channels_middle_inner_t
      channels_middle_inner_t;
  typedef
      typename InterconnectTypeConfig<IM>::channels_middle_t channels_middle_t;
  typedef typename InterconnectTypeConfig<IM>::channels_end_t channels_end_t;

  typedef typename InterconnectTypeConfig<IM>::srcs_typeid_t srcs_typeid_t;
  typedef typename InterconnectTypeConfig<IM>::dests_typeid_t dests_typeid_t;
  typedef typename InterconnectTypeConfig<IM>::srcs_user_typeid_t
      srcs_user_typeid_t;
  typedef typename InterconnectTypeConfig<IM>::dests_user_typeid_t
      dests_user_typeid_t;
  typedef typename InterconnectTypeConfig<IM>::srcs_width_t srcs_width_t;
  typedef typename InterconnectTypeConfig<IM>::dests_width_t dests_width_t;

  typedef
      typename InterconnectTypeConfig<IM>::channels_valid_pairs_reverse_inner_t
          channels_valid_pairs_reverse_inner_t;
  typedef typename InterconnectTypeConfig<IM>::channels_valid_pairs_reverse_t
      channels_valid_pairs_reverse_t;

  typedef typename InterconnectTypeConfig<IM>::idx_t idx_t;
  typedef typename InterconnectTypeConfig<IM>::dest_map_idx_t dest_map_idx_t;
  typedef typename InterconnectTypeConfig<IM>::dest_maps_t dest_maps_t;
  typedef typename InterconnectTypeConfig<IM>::dest_maps_idx_vec_t
      dest_maps_idx_vec_t;

  typedef typename InterconnectTypeConfig<IM>::cycles_count_t cycles_count_t;
  typedef typename InterconnectTypeConfig<IM>::cycle_count_channel_t
      cycle_count_channel_t;

  const unsigned int ic_id;

 public:
  std::map<IM_src_t *, Connections::Blocking_abs *> srcs_block_abs;
  std::map<IM_dest_t *, Connections::Blocking_abs *> dests_block_abs;

 protected:
  // Vectors of ports
  srcs_t srcs;
  dests_t dests;

  srcs_t disabled_srcs;
  dests_t disabled_dests;
  std::vector<std::tuple<Connections::Blocking_abs *,
                         Connections::Blocking_abs *> > disabled_pairs;

  std::map<IM_src_t *, std::string> srcs_names;
  std::map<IM_dest_t *, std::string> dests_names;

  // Maps of channels
  channels_begin_t channels_begin;

 public:
  channels_middle_t channels_middle;

 protected:
  channels_end_t channels_end;

  channels_valid_pairs_reverse_t channels_valid_pairs_reverse;

 public:
  // Tracking to find valid combinations of channels
  srcs_typeid_t srcs_typeid;
  dests_typeid_t dests_typeid;
  srcs_user_typeid_t srcs_user_typeid;
  dests_user_typeid_t dests_user_typeid;
  std::map<unsigned int, const char *> msg_names;   // key = msg_id
  std::map<unsigned int, const char *> part_names;  // key = part_type_id
  std::map<unsigned int, unsigned int>
      part_to_part_type;  // key = user_part_Id, value = part_type_id
  std::map<unsigned int, const char *>
      part_inst_names;  // key = user_part_Id, value is part_name
  std::map<unsigned int, std::map<unsigned int, std::set<unsigned int> > >
      part_msgs_srcs;  // key = part_type_id, set is msg ids
  std::map<unsigned int, std::map<unsigned int, std::set<unsigned int> > >
      part_msgs_dests;  // key = part_type_id, set is msg ids
  srcs_width_t srcs_width;
  dests_width_t dests_width;

 protected:
  // Instrumentation
  cycles_count_t cycle_count_total;
  cycle_count_channel_t cycle_count_channel;

 public:
  // Ports
  sc_in_clk clk;
  sc_in<bool> rst;

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Constructors

  InterconnectBase(const unsigned int &ic_id_)
      : sc_module(sc_module_name(sc_gen_unique_name("interconnect"))),
        clk(sc_gen_unique_name("clk")),
        rst(sc_gen_unique_name("rst")),
        ic_id(ic_id_) {

    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst, false);
  }

  explicit InterconnectBase(const char *name, const unsigned int &ic_id_)
      : sc_module(sc_module_name(name)),
        clk(sc_gen_unique_name("clk")),
        rst(sc_gen_unique_name("rst")),
        ic_id(ic_id_) {

    SC_CTHREAD(run, clk.pos());
    async_reset_signal_is(rst, false);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Run loop

  void run() {
    cycle_count_total = 0;

    wait();

    while (1) {
      cycle_count_total++;
      wait();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Channel binding

  template <typename T>
  void Bind(IM_src_t &p) {
    Bind(p, Wrapped<T>::width, typeid(T).name());
  }

  template <typename T>
  void Bind(IM_dest_t &p) {
    Bind(p, Wrapped<T>::width, typeid(T).name());
  }

  void Bind(IM_src_t &p, unsigned int type_width, const char *type_name) {
    srcs.push_back(&p);

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }
    set_src_name(&p, port_name);

    boost::hash<const char *> typeid_hash;
    srcs_typeid[&p] = typeid_hash(type_name);
    srcs_width[&p] = type_width;
  }

  void Bind(IM_dest_t &p, unsigned int type_width, const char *type_name) {
    dests.push_back(&p);

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }
    set_dest_name(&p, port_name);

    boost::hash<const char *> typeid_hash;
    dests_typeid[&p] = typeid_hash(type_name);
    dests_width[&p] = type_width;
  }

  template <typename T>
  void Bind(IM_dest_t &p, unsigned int userid, const char *msg_name) {
    Bind<T>(p);
    dests_user_typeid[&p] = userid;
    msg_names[userid] = msg_name;
  }
  template <typename T>
  void Bind(IM_src_t &p, unsigned int userid, const char *msg_name) {
    Bind<T>(p);
    srcs_user_typeid[&p] = userid;
    msg_names[userid] = msg_name;
  }

  void Bind(IM_dest_t &p, unsigned int userid, unsigned int type_width,
            const char *type_name, const char *msg_name) {
    Bind(p, type_width, type_name);
    dests_user_typeid[&p] = userid;
    msg_names[userid] = msg_name;
  }
  void Bind(IM_src_t &p, unsigned int userid, unsigned int type_width,
            const char *type_name, const char *msg_name) {
    Bind(p, type_width, type_name);
    srcs_user_typeid[&p] = userid;
    msg_names[userid] = msg_name;
  }

  ////////////////////////////////////////////////////////////////////////////
  // New IC_MESSAGE

  struct GenericDestHandlerIface {
   public:
    typename InterconnectBase<IM>::IM_dest_t *im_ptr;
  };

  struct GenericSrcHandlerIface {
   public:
    typename InterconnectBase<IM>::IM_src_t *im_ptr;
  };

  std::map<Connections::Blocking_abs *, GenericDestHandlerIface *>
      generic_iface_dests;
  std::map<Connections::Blocking_abs *, GenericSrcHandlerIface *>
      generic_iface_srcs;

  template <typename MSG_TYPE>
  class ICMFromSpecialized : public sc_module,
                             public Connections::Blocking_abs,
                             public GenericSrcHandlerIface {
    SC_HAS_PROCESS(ICMFromSpecialized);

   public:
    typedef InterconnectBase<IM> interconnect_t;
    interconnect_t &ic;

    typename interconnect_t::IM_src_t im;
    Connections::OutBlocking<interconnect::Msg<MSG_TYPE> > &p;
    sc_signal<sc_lv<Wrapped<interconnect::Msg<MSG_TYPE> >::width> > int_msg;
    sc_signal<bool> int_rdy;
    sc_signal<bool> int_val;

    explicit ICMFromSpecialized(
        interconnect_t &ic_,
        Connections::OutBlocking<interconnect::Msg<MSG_TYPE> > &p_)
        : sc_module(sc_module_name(sc_gen_unique_name("im_from_specific"))),
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
        Marshaller<Wrapped<interconnect::Msg<MSG_TYPE> >::width> marshaller(
            int_msg.read());
        Wrapped<interconnect::Msg<MSG_TYPE> > result;
        result.Marshall(marshaller);

        // Create the InterconnectMessage
        typename interconnect_t::IM_t im_msg;
        interconnect::Msg<MSG_TYPE> m = result.val;
        im_msg = m;

        // Convert back to Marshall'd form
        Marshaller<Wrapped<typename interconnect_t::IM_t>::width> im_marshaller;
        Wrapped<typename interconnect_t::IM_t> im_wm(im_msg);
        im_wm.Marshall(im_marshaller);
        im.msg.write(im_marshaller.GetResult());
      }
    }
  };  // class ICMFromSpecialized

  template <typename MSG_TYPE>
  class ICMToSpecialized : public sc_module,
                           public Connections::Blocking_abs,
                           public GenericDestHandlerIface {
    SC_HAS_PROCESS(ICMToSpecialized);

    static const interconnect::MSG_ID_t MSG_ID = MSG_TYPE::msg_id;

   public:
    typedef InterconnectBase<IM> interconnect_t;
    interconnect_t &ic;
    typename interconnect_t::IM_dest_t im;
    Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p;
    sc_signal<sc_lv<Wrapped<interconnect::Msg<MSG_TYPE> >::width> > int_msg;
    sc_signal<bool> int_rdy;
    sc_signal<bool> int_val;

    explicit ICMToSpecialized(
        interconnect_t &ic_,
        Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p_)
        : sc_module(sc_module_name(sc_gen_unique_name("icm_to_specialized"))),
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
        Marshaller<Wrapped<interconnect::Msg<MSG_TYPE> >::width> marshaller;
        interconnect::Msg<MSG_TYPE> m(im_msg);
        Wrapped<interconnect::Msg<MSG_TYPE> > wm(m);
        wm.Marshall(marshaller);
        int_msg.write(marshaller.GetResult());
      }
    }
  };  // ICMToSpecialized

  template <typename MSG_TYPE>
  void Bind(Connections::OutBlocking<interconnect::Msg<MSG_TYPE> > &p) {
    ICMFromSpecialized<MSG_TYPE> *is =
        new ICMFromSpecialized<MSG_TYPE>(*this, p);
    Bind<typename MSG_TYPE::data_t>(is->im, MSG_TYPE::msg_id, MSG_TYPE::name);

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }
    set_src_name(&is->im, port_name);

    generic_iface_srcs[&p] = is;
    msg_names[MSG_TYPE::msg_id] = MSG_TYPE::name;
  }

  template <typename MSG_TYPE>
  void Bind(Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p) {
    ICMToSpecialized<MSG_TYPE> *ij = new ICMToSpecialized<MSG_TYPE>(*this, p);
    Bind<typename MSG_TYPE::data_t>(ij->im, MSG_TYPE::msg_id, MSG_TYPE::name);

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }
    set_dest_name(&ij->im, port_name);

    generic_iface_dests[&p] = ij;
    msg_names[MSG_TYPE::msg_id] = MSG_TYPE::name;
  }

  template <typename T, typename MSG_TYPE>
  void Bind(Connections::OutBlocking<interconnect::Msg<MSG_TYPE> > &p,
            unsigned int userid) {
    assert(userid == MSG_TYPE::msg_id);
    Bind(p);
  }

  template <typename T, typename MSG_TYPE>
  void Bind(Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p,
            unsigned int userid) {
    assert(userid == MSG_TYPE::msg_id);
    Bind(p);
  }
  template <typename MSG_TYPE>
  void config_dest_map_idx(
      dest_map_idx_t dest_map_idx, idx_t dest_idx,
      Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p) {
    config_dest_map_idx(
        dest_map_idx, dest_idx,
        *(generic_iface_dests[(Connections::Blocking_abs *)&p]->im_ptr));
  }

  ////////////////////////////////////////////////////////////////////////////

  // Disable a source or dest port. Still binds and resets it,
  // but otherwise disabled it for sending or receiving.
  // Useful when there are additional ports that need to be bound
  // but will be unused (edges of an array, testbench, etc).
  void disable_port(IM_dest_t &p) { disabled_dests.push_back(&p); }
  void disable_port(IM_src_t &p) { disabled_srcs.push_back(&p); }

  void disable_pair_of_ports(Connections::Blocking_abs &p1,
                             Connections::Blocking_abs &p2) {
    disabled_pairs.push_back(std::make_tuple(&p1, &p2));
  }

  template <typename MSG_TYPE>
  void disable_port(Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p) {
    disabled_dests.push_back(
        generic_iface_dests[(Connections::Blocking_abs *)&p]->im_ptr);
  }
  template <typename MSG_TYPE>
  void disable_port(Connections::OutBlocking<interconnect::Msg<MSG_TYPE> > &p) {
    disabled_srcs.push_back(
        generic_iface_srcs[(Connections::Blocking_abs *)&p]->im_ptr);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Modifiers

  idx_t find_src_idx(IM_src_t *p) {
    unsigned long i = 0;
    for (typename srcs_t::iterator it_src = srcs.begin(); it_src != srcs.end();
         ++it_src, ++i) {
      if ((*it_src) == p) {
        return i;
      }
    }
    return -1;
  }

  idx_t find_dest_idx(IM_dest_t *p) {
    unsigned long i = 0;
    for (typename dests_t::iterator it_dest = dests.begin();
         it_dest != dests.end(); ++it_dest, ++i) {
      if ((*it_dest) == p) {
        return i;
      }
    }
    return -1;
  }

  IM_src_t *get_src_from_idx(idx_t src_idx) {
    NVHLS_ASSERT_MSG(src_idx != -1, "Source index is not found");
    NVHLS_ASSERT_MSG(src_idx < get_src_size(),
                     "Source index exceeds number of sources");

    return srcs[src_idx];
  }

  idx_t get_real_dest_idx(idx_t dest_idx, dest_map_idx_t dest_map_idx = 0) {
    idx_t real_dest_idx;

    if (dest_map_idx == 0) {
      real_dest_idx = dest_idx;
    } else {
      // Assert that dest_map_idx map exists
      NVHLS_ASSERT_MSG(
          std::find(dest_maps_idx_vec.begin(), dest_maps_idx_vec.end(),
                    dest_map_idx) != dest_maps_idx_vec.end(),
          "Destination map not found!");

      // Assert that the dest_idx exists in that map.
      NVHLS_ASSERT_MSG(dest_maps[dest_map_idx].find(dest_idx) !=
                           dest_maps[dest_map_idx].end(),
                       "Interconnect coundn't find dest_idx in dest_map_idx.");

      // Get the real idx
      real_dest_idx = dest_maps[dest_map_idx][dest_idx];
    }
    return real_dest_idx;
  }

  IM_dest_t *get_dest_from_idx(idx_t dest_idx,
                               dest_map_idx_t dest_map_idx = 0) {
    idx_t real_dest_idx;

    real_dest_idx = get_real_dest_idx(dest_idx, dest_map_idx);

    NVHLS_ASSERT_MSG(real_dest_idx != -1,
                     "Real dest index is -1, indicating not found!");
    NVHLS_ASSERT_MSG(real_dest_idx < get_dest_size(),
                     "Real dest index exceeds number of dests in design!");

    return dests[real_dest_idx];
  }

  void set_src_name(IM_src_t *p, std::string &s) { srcs_names[p] = s; }
  void set_dest_name(IM_dest_t *p, std::string &s) { dests_names[p] = s; }
  std::string get_src_name(IM_src_t *p) { return srcs_names[p]; }
  std::string get_dest_name(IM_dest_t *p) { return dests_names[p]; }

  unsigned int get_src_size() { return srcs.size(); }

  unsigned int get_dest_size() { return dests.size(); }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Route creation

  dest_maps_t dest_maps;
  dest_maps_idx_vec_t dest_maps_idx_vec;

  dest_map_idx_t create_dest_map() {
    // Seed starting value
    dest_map_idx_t dest_map_idx = 1;

    // Find lowest integer not in dest_maps.
    sort(dest_maps_idx_vec.begin(), dest_maps_idx_vec.end());
    for (typename dest_maps_idx_vec_t::iterator it = dest_maps_idx_vec.begin();
         it != dest_maps_idx_vec.end(); ++it) {
      if ((*it) >= dest_map_idx) {
        if ((*it) == dest_map_idx)
          dest_map_idx++;  // Found it
        else
          break;  // current value will work
      }
    }
    create_dest_map(dest_map_idx);
    return dest_map_idx;
  }

  dest_map_idx_t create_dest_map(dest_map_idx_t dest_map_idx,
                                 const bool &allow_existing = false) {
    NVHLS_ASSERT(dest_map_idx != 0);

    if (allow_existing &&
        std::find(dest_maps_idx_vec.begin(), dest_maps_idx_vec.end(),
                  dest_map_idx) != dest_maps_idx_vec.end()) {
      return dest_map_idx;
    }

    NVHLS_ASSERT(std::find(dest_maps_idx_vec.begin(), dest_maps_idx_vec.end(),
                           dest_map_idx) == dest_maps_idx_vec.end());

    dest_maps_idx_vec.push_back(dest_map_idx);

    return dest_map_idx;
  }

  void config_dest_map_idx(dest_map_idx_t dest_map_idx, idx_t dest_idx,
                           idx_t real_dest_idx) {
    NVHLS_ASSERT_MSG(dest_map_idx != 0,
                     "The default dest_map_idx=0 is fixed, and cannot be "
                     "manually assigned. Use a dest_map_idx > 0.");
    NVHLS_ASSERT_MSG(
        std::find(dest_maps_idx_vec.begin(), dest_maps_idx_vec.end(),
                  dest_map_idx) != dest_maps_idx_vec.end(),
        "Couldn't find dest_map_idx!");

    dest_maps[dest_map_idx][dest_idx] = real_dest_idx;
  }

  void config_dest_map_idx(dest_map_idx_t dest_map_idx, idx_t dest_idx,
                           IM_dest_t &p) {
    config_dest_map_idx(dest_map_idx, dest_idx, this->find_dest_idx(&p));
  }

  void add_to_dest_directory(IM_dest_t &p, unsigned int part_id,
                             unsigned int msg_id, unsigned int port_id = 0,
                             bool allow_duplicate = false) {
    auto tuple_idx = std::make_tuple(part_id, msg_id, port_id);
    if (!allow_duplicate) {
      NVHLS_ASSERT_MSG(
          interconnect::get_ICManager().dest_directory.find(tuple_idx) ==
              interconnect::get_ICManager().dest_directory.end(),
          "Already have a destination matching that index");
    }
    interconnect::get_ICManager().dest_directory[tuple_idx].add(
        this->find_dest_idx(&p));
  }

  template <typename MSG_TYPE>
  void add_to_dest_directory(
      Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p,
      unsigned int part_id, unsigned int port_id = 0,
      bool allow_duplicate = false) {
    add_to_dest_directory(*generic_iface_dests[&p]->im_ptr, part_id,
                          MSG_TYPE::msg_id, port_id, allow_duplicate);
  }

  route_t *create_route() { return new Route(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Helpers

  // Sets w to smallest width that is >= min_w. Returns false if no min width is
  // found.
  template <typename T>
  bool ports_min_width(T &ref, unsigned int &w, unsigned int min_w = 0) {
    bool found = false;
    for (typename T::iterator it_ref = ref.begin(); it_ref != ref.end();
         ++it_ref) {
      if (it_ref->second >= min_w && (it_ref->second < w || !found)) {
        w = it_ref->second;
        found = true;
      }
    }
    return found;
  }

  bool srcs_and_dests_min_width(unsigned int &w, unsigned int min_w = 0) {
    return ports_min_width(srcs_width, w, min_w) ||
           ports_min_width(dests_width, w, min_w);
  }

  void pretty_print() {
    cout << std::string(100, '#') << endl;

    unsigned int current_port_width;
    bool port_width_found = srcs_and_dests_min_width(current_port_width);
    while (port_width_found) {
      cout << "# " << dec << current_port_width << "-bit Ports" << endl;
      cout << "## Sources" << endl;
      for (typename srcs_t::iterator it_src = srcs.begin();
           it_src != srcs.end(); ++it_src) {
        if (srcs_width[*it_src] == current_port_width) {
          if (find(disabled_srcs.begin(), disabled_srcs.end(), *it_src) !=
              disabled_srcs.end()) {
            cout << "  - " << get_src_name(*it_src) << " is disabled." << endl;
          } else if (srcs_user_typeid.find(*it_src) != srcs_user_typeid.end()) {
            cout << "  - " << get_src_name(*it_src)
                 << " (user typeid = " << srcs_user_typeid[*it_src]
                 << ") connects to "
                 << num_compatible_channels_from_src(*it_src)
                 << " destinations." << endl;
          } else {
            cout << "  - " << get_src_name(*it_src) << " connects to "
                 << num_compatible_channels_from_src(*it_src)
                 << " destinations." << endl;
          }
        }
      }
      cout << endl;

      cout << "## Destinations" << endl;
      for (typename dests_t::iterator it_dest = dests.begin();
           it_dest != dests.end(); ++it_dest) {
        if (dests_width[*it_dest] == current_port_width) {
          if (find(disabled_dests.begin(), disabled_dests.end(), *it_dest) !=
              disabled_dests.end()) {
            cout << "  - " << get_dest_name(*it_dest) << " is disabled."
                 << endl;
          } else if (dests_user_typeid.find(*it_dest) !=
                     dests_user_typeid.end()) {
            cout << "  - " << get_dest_name(*it_dest)
                 << " (user typeid = " << dests_user_typeid[*it_dest]
                 << ") connects from "
                 << num_compatible_channels_to_dest(*it_dest) << " sources."
                 << endl;
          } else {
            cout << "  - " << get_dest_name(*it_dest) << " connects from "
                 << num_compatible_channels_to_dest(*it_dest) << " sources."
                 << endl;
          }
        }
      }
      cout << endl;
      cout << endl;

      // update
      port_width_found =
          srcs_and_dests_min_width(current_port_width, current_port_width + 1);
    }

    cout << "# Summary" << endl;
    cout << "Source Count = " << get_src_size() << endl;
    cout << "Destination Count = " << get_dest_size() << endl;
    cout << std::string(100, '#') << endl;
    cout << endl;
  }

  void print_statistics(bool include_no_traffic = false) {
    cout << std::string(100, '#') << endl;
    cout << "# Traffic Summary" << endl;

    unsigned int current_port_width;
    bool port_width_found = srcs_and_dests_min_width(current_port_width);
    while (port_width_found) {
      bool found_one = false;

      cout << "# " << dec << current_port_width << "-bit Ports" << endl;
      cout << endl;

      for (typename channels_middle_t::iterator it_src =
               channels_middle.begin();
           it_src != channels_middle.end(); ++it_src) {
        for (typename channels_middle_inner_t::iterator it_dest =
                 it_src->second.begin();
             it_dest != it_src->second.end(); ++it_dest) {

          if (srcs_width[it_src->first] == current_port_width) {
            NVHLS_ASSERT_MSG(
                dests_width[it_dest->first] == current_port_width,
                "Src and dest widths should match, but they don't!");

            if (cycle_count_channel[it_src->first][it_dest->first]
                    .to_uint64() == 0) {
              if (include_no_traffic) {
                cout << "Src = " << get_src_name(it_src->first)
                     << "Dest = " << get_dest_name(it_dest->first)
                     << ", NO TRAFFIC" << endl;
                found_one = true;
              }
            } else {
              cout << "  - " << get_src_name(it_src->first) << "  -->  "
                   << get_dest_name(it_dest->first) << ", messages = " << dec
                   << cycle_count_channel[it_src->first][it_dest->first]
                          .to_uint64()
                   << " ("
                   << boost::format("%2.f%%") %
                          (100.0 *
                           cycle_count_channel[it_src->first][it_dest->first]
                               .to_uint64() /
                           cycle_count_total.to_uint64())
                   << ")" << endl;
              found_one = true;
            }
          }
        }
      }

      if (found_one == false) {
        cout << "   (none with non-zero message counts)" << endl;
      }

      cout << endl;

      // update
      port_width_found =
          srcs_and_dests_min_width(current_port_width, current_port_width + 1);
    }
    cout << endl
         << "Total cycles was: " << cycle_count_total.to_uint64() << endl;
    cout << std::string(100, '#') << endl;
    cout << endl;
  }

 protected:
  // Unsupported in this code-- implement in future
  void enable_fixed_bypass(IM_src_t &s, IM_dest_t &d) {
    // Remove channels_middle
    NVHLS_ASSERT(0);
  }

  void before_end_of_elaboration() {
    for (typename srcs_t::iterator it_src = srcs.begin(); it_src != srcs.end();
         ++it_src) {
      for (typename dests_t::iterator it_dest = dests.begin();
           it_dest != dests.end(); ++it_dest) {
        if (is_compatible_channels_pre_elab(*it_src, *it_dest)) {
          // Generate the name
          std::string new_channel_name =
              get_src_name(*it_src) + "__to__" + get_dest_name(*it_dest);
          boost::replace_all(new_channel_name, ".", "_");

          // Create the new channel object
          IM_chanlink_t *new_channel =
              new IM_chanlink_t(new_channel_name.c_str());
          new_channel->in_str = get_src_name(*it_src).c_str();
          new_channel->out_str = get_dest_name(*it_dest).c_str();

          // Add it to channels_middle
          channels_middle[(*it_src)][(*it_dest)] = new_channel;

          // Add it to reverse channels
          channels_valid_pairs_reverse[(*it_dest)].push_back(*it_src);
        }
      }
    }

    // Source read channels
    for (typename srcs_t::iterator it_src = srcs.begin(); it_src != srcs.end();
         ++it_src) {
      // Create new channel
      IM_chan_t *new_channel =
          new IM_chan_t(sc_gen_unique_name("new_channel_src"));
      channels_begin[(*it_src)] = new_channel;

      // Bind it and disable annotation
      (*it_src)->Bind(*new_channel);
      new_channel->disable_annotate();

      // Create a new input arbitration object, depending on size
      if (num_compatible_channels_from_src(*it_src) == 1) {
        // Special bypass
        new InputBypass(*this, (*it_src),
                        channels_middle[*it_src].begin()->first, channels_begin,
                        channels_middle, channels_end);
      } else {
        // Otherwise the normal input decode module
        InputDecoder *i_dec = new InputDecoder(*this, (*it_src), channels_begin,
                                               channels_middle, channels_end);
        i_dec->clk(clk);
        i_dec->rst(rst);
      }
    }

    // Destination write channels
    for (typename dests_t::iterator it_dest = dests.begin();
         it_dest != dests.end(); ++it_dest) {
      // Create new channel
      IM_chan_t *new_channel =
          new IM_chan_t(sc_gen_unique_name("new_channel_dest"));
      channels_end[(*it_dest)] = new_channel;

      // Bind it and disable annotation
      (*it_dest)->Bind(*new_channel);
      new_channel->disable_annotate();

      // Create a new output arbitration object
      if (num_compatible_channels_to_dest(*it_dest) == 1) {
        OutputBypass *o_bypass =
            new OutputBypass(*this, (*it_dest),
                             *(channels_valid_pairs_reverse[*it_dest].begin()),
                             channels_begin, channels_middle, channels_end,
                             channels_valid_pairs_reverse, cycle_count_channel);
        o_bypass->clk(clk);
        o_bypass->rst(rst);
      } else {
        OutputArbiter *o_arb = new OutputArbiter(
            *this, (*it_dest), channels_begin, channels_middle, channels_end,
            channels_valid_pairs_reverse, cycle_count_channel);
        o_arb->clk(clk);
        o_arb->rst(rst);
      }
    }

    const char *BUILD_PREFIX = std::getenv("BUILD");
    std::string interconnect_dir = "interconnect";
    if (BUILD_PREFIX) {
      interconnect_dir += std::string("_") + std::string(BUILD_PREFIX);
      BUILD_PREFIX = "";
    } else {
      BUILD_PREFIX = "";
    }

    // Write out latency here.
    const char *nvhls_elab_only = std::getenv("NVHLS_ELAB_ONLY");
    if (nvhls_elab_only && std::string(nvhls_elab_only) == "1") {
      interconnect::save_connectivity(*this, *this, BUILD_PREFIX,
                                      interconnect_dir);
      cout << "Info: Elaboration only run. Quiting..." << endl;
      sc_stop();
    } else {
      interconnect::annotate_design(*this, *this, BUILD_PREFIX,
                                    interconnect_dir, interconnect_dir);
    }
  }

  bool is_compatible_channels_pre_elab(IM_src_t *src, IM_dest_t *dest) {
    bool src_exists = (srcs_typeid.find(src) != srcs_typeid.end());
    bool dest_exists = (dests_typeid.find(dest) != dests_typeid.end());

    bool src_disabled = find(disabled_srcs.begin(), disabled_srcs.end(), src) !=
                        disabled_srcs.end();
    bool dest_disabled = find(disabled_dests.begin(), disabled_dests.end(),
                              dest) != disabled_dests.end();

    // investigate why ModuleArray fails this assertion. Probably need to
    // populate. srcs_block_abs/dests_block_abs with other Bind()
    // Disabling for now.
    // Connections::Blocking_abs *src_port_block_abs = srcs_block_abs[src];
    // Connections::Blocking_abs *dest_port_block_abs = dests_block_abs[dest];

    // assert(src_port_block_abs && dest_port_block_abs);

    // bool pair_disabled = find(disabled_pairs.begin(), disabled_pairs.end(),
    // std::make_tuple(src_port_block_abs, dest_port_block_abs))
    //                         != disabled_pairs.end();
    bool pair_disabled = false;

    bool src_has_user_typeid =
        (srcs_user_typeid.find(src) != srcs_user_typeid.end());
    bool dest_has_user_typeid =
        (dests_user_typeid.find(dest) != dests_user_typeid.end());

    if (!(src_exists && dest_exists)) {
      if (!src_exists)
        CDCOUT("Warning: Interconnect bound source port "
                   << get_src_name(src) << " doesn't have an assigned type."
                   << endl,
               interconnect::kDebugLevel);
      if (!dest_exists)
        CDCOUT("Warning: Interconnect bound destination port "
                   << get_dest_name(dest) << " doesn't have an assigned type."
                   << endl,
               interconnect::kDebugLevel);
      return true;
    }

    // Fail conditions
    if (src_disabled || dest_disabled || pair_disabled) {
      return false;
    }
    if (src_has_user_typeid != dest_has_user_typeid) {
      return false;
    }
    if (src_has_user_typeid && dest_has_user_typeid &&
        (srcs_user_typeid[src] != dests_user_typeid[dest])) {
      return false;
    }
    if (!src_has_user_typeid ||
        !dest_has_user_typeid) {  // HERE adding in this condition.
      if (srcs_typeid[src] != dests_typeid[dest]) {
        return false;
      }
    }

    // Pass condition
    return true;
  }

  bool is_compatible_channels(IM_src_t *src, IM_dest_t *dest) {
    typename channels_middle_t::iterator it_dest = channels_middle.find(src);
    return (it_dest != channels_middle.end()) &&
           it_dest->second.find(dest) != it_dest->second.end();
  }

  unsigned int num_compatible_channels_from_src(IM_src_t *p) {
    return (channels_middle.find(p) == channels_middle.end())
               ? 0
               : channels_middle[p].size();
  }

  unsigned int num_compatible_channels_to_dest(IM_dest_t *p) {
    return (channels_valid_pairs_reverse.find(p) ==
            channels_valid_pairs_reverse.end())
               ? 0
               : channels_valid_pairs_reverse[p].size();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Route
  class Route {
   public:
    typedef std::vector<IM_dest_t *> route_vec_t;
    typedef typename route_vec_t::iterator iterator;

   protected:
    InterconnectBase<IM> &ic;

    route_vec_t route_vec;

   public:
    Route(InterconnectBase<IM> &ic_) : ic(ic_) {}

    void add_dest(idx_t dest_idx, dest_map_idx_t dest_map_idx = 0) {
      route_vec.push_back(ic.get_dest_from_idx(dest_idx, dest_map_idx));
    }

    iterator begin() { return route_vec.begin(); }

    iterator end() { return route_vec.end(); }

    int size() { return route_vec.size(); }
  };

  ////////////////////////////////////////////////////////////////////////////////
  // InputDecoder
  class InputDecoder : public sc_module, public Connections::Blocking_abs {
    SC_HAS_PROCESS(InputDecoder);

   protected:
    InterconnectBase<IM> &ic;
    IM_src_t *p;
    channels_begin_t &channels_begin;
    channels_middle_t &channels_middle;
    channels_end_t &channels_end;

   public:
    sc_in_clk clk;
    sc_in<bool> rst;

    explicit InputDecoder(InterconnectBase<IM> &ic_, IM_src_t *p_,
                          channels_begin_t &channels_begin_,
                          channels_middle_t &channels_middle_,
                          channels_end_t &channels_end_)
        : sc_module(sc_module_name(sc_gen_unique_name("input_decoder"))),
          ic(ic_),
          p(p_),
          channels_begin(channels_begin_),
          channels_middle(channels_middle_),
          channels_end(channels_end_),
          clk(sc_gen_unique_name("clk")),
          rst(sc_gen_unique_name("rst")) {

      SC_CTHREAD(input_dec_run, clk.pos());
      async_reset_signal_is(rst, false);
    }

    void input_dec_run() {
      IM data;

      // Reset input
      channels_begin[p]->ResetRead();

      // Check if we have valid outputs.
      switch (ic.num_compatible_channels_from_src(p)) {
        case 0:
          CDCOUT("Info: Source port "
                     << ic.get_src_name(p)
                     << " doesn't have any valid destinations. Disabling "
                        "thread for "
                        "this source port in the interconnect modeler."
                     << endl,
                 interconnect::kDebugLevel);
          return;
      }

      // Reset outputs.
      for (typename channels_middle_inner_t::iterator it_dest =
               channels_middle[p].begin();
           it_dest != channels_middle[p].end(); ++it_dest) {
        it_dest->second->ResetWrite();
      }

      while (1) {
        wait();

        IM data;
        if (channels_begin[p]->PopNB(data)) {
          data.set_src_idx(ic, p);
          typename IM::multicast_t multicast_dests = data.get_multicast(ic);
          for (typename IM::multicast_t::iterator it = multicast_dests.begin();
               it != multicast_dests.end(); ++it) {
            NVHLS_ASSERT_MSG(ic.is_compatible_channels(p, (*it)),
                             "Interconnect received a source mesage to an "
                             "invalid or incompatible destination.");
            channels_middle[p][(*it)]->Push(data);
          }
        }
      }
    }
  };  // end class InputDecoder

  ////////////////////////////////////////////////////////////////////////////////
  // InputBypass
  class InputBypass : public sc_module, public Connections::Blocking_abs {
    SC_HAS_PROCESS(InputBypass);

   protected:
    InterconnectBase<IM> &ic;
    IM_src_t *p;
    IM_dest_t *q;
    channels_begin_t &channels_begin;
    channels_middle_t &channels_middle;
    channels_end_t &channels_end;

   public:
    explicit InputBypass(InterconnectBase<IM> &ic_, IM_src_t *p_, IM_dest_t *q_,
                         channels_begin_t &channels_begin_,
                         channels_middle_t &channels_middle_,
                         channels_end_t &channels_end_)
        : sc_module(sc_module_name(sc_gen_unique_name("input_bypass"))),
          ic(ic_),
          p(p_),
          q(q_),
          channels_begin(channels_begin_),
          channels_middle(channels_middle_),
          channels_end(channels_end_) {

      CDCOUT("Info: Source port "
                 << ic.get_src_name(p)
                 << " only has one valid destination. Adding a bypass to this "
                    "source port in the interconnect modeler."
                 << endl,
             interconnect::kDebugLevel);

#ifdef CONNECTIONS_SIM_ONLY
      // SC_METHOD(do_bypass);
      declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                             SC_CURRENT_USER_MODULE, do_bypass);
      this->sensitive << channels_begin[p]->out_msg
                      << channels_begin[p]->out_val
                      << channels_middle[p][q]->in_rdy;

      channels_begin[p]->out_bound = true;
      channels_middle[p][q]->in_bound = true;

#endif
    }

    void do_bypass() {
      bool is_val = channels_begin[p]->out_val.read();

      // Set val and msg respectively
      channels_middle[p][q]->in_val.write(is_val);
      channels_begin[p]->out_rdy.write(channels_middle[p][q]->in_rdy.read());

      // Only convert if is_val (to prevent X's)
      if (is_val) {
        channels_middle[p][q]->in_msg.write(channels_begin[p]->out_msg.read());
      }
    }
  };  // end class InputBypass

  // OutputArbiter
  class OutputArbiter : public sc_module, public Connections::Blocking_abs {
    SC_HAS_PROCESS(OutputArbiter);

   protected:
    InterconnectBase<IM> &ic;
    IM_dest_t *p;
    channels_begin_t &channels_begin;
    channels_middle_t &channels_middle;
    channels_end_t &channels_end;
    channels_valid_pairs_reverse_t &channels_valid_pairs_reverse;
    cycle_count_channel_t &cycle_count_channel;

   public:
    sc_in_clk clk;
    sc_in<bool> rst;

    explicit OutputArbiter(
        InterconnectBase<IM> &ic_, IM_dest_t *p_,
        channels_begin_t &channels_begin_, channels_middle_t &channels_middle_,
        channels_end_t &channels_end_,
        channels_valid_pairs_reverse_t &channels_valid_pairs_reverse_,
        cycle_count_channel_t &cycle_count_channel_)
        : sc_module(sc_module_name(sc_gen_unique_name("output_arbiter"))),
          ic(ic_),
          p(p_),
          channels_begin(channels_begin_),
          channels_middle(channels_middle_),
          channels_end(channels_end_),
          channels_valid_pairs_reverse(channels_valid_pairs_reverse_),
          cycle_count_channel(cycle_count_channel_),
          clk(sc_gen_unique_name("clk")),
          rst(sc_gen_unique_name("rst")) {

      SC_CTHREAD(output_arb_run, clk.pos());
      async_reset_signal_is(rst, false);
    }

    void output_arb_run() {
      IM data;

      // Reset output
      channels_end[p]->ResetWrite();

      // Check if we have valid inputs.
      switch (ic.num_compatible_channels_to_dest(p)) {
        case 0:
          CDCOUT("Info: Destination port "
                     << ic.get_dest_name(p)
                     << " doesn't have any valid sources. Disabling thread for "
                        "this "
                        "destination port in the interconnect modeler."
                     << endl,
                 interconnect::kDebugLevel);
          return;
        case 1:
          CDCOUT("Warning: FIXME Optimization Destination port "
                     << ic.get_dest_name(p)
                     << " only has one source, could optimize." << endl,
                 interconnect::kDebugLevel);
          break;
      }

      for (typename channels_valid_pairs_reverse_inner_t::iterator it_src =
               channels_valid_pairs_reverse[p].begin();
           it_src != channels_valid_pairs_reverse[p].end(); ++it_src) {
        channels_middle[*it_src][p]->ResetRead();
        cycle_count_channel[*it_src][p] = 0;
      }

      // Start round robin at beginning.
      typename channels_valid_pairs_reverse_inner_t::iterator
          last_grant_it_src_begin = channels_valid_pairs_reverse[p].begin();
      typename channels_valid_pairs_reverse_inner_t::iterator
          last_grant_it_src_end = channels_valid_pairs_reverse[p].end();
      typename channels_valid_pairs_reverse_inner_t::iterator
          last_grant_it_src = last_grant_it_src_begin;

      while (1) {
        wait();

        typename channels_valid_pairs_reverse_inner_t::iterator it_src =
            last_grant_it_src;

        do {
          if (channels_middle[*it_src][p]->PopNB(data)) {
            channels_end[p]->Push(data);        // Push
            cycle_count_channel[*it_src][p]++;  // Monitor
            last_grant_it_src = it_src;         // Update
          }

          if (++it_src == last_grant_it_src_end) {
            it_src = last_grant_it_src_begin;
          }
        } while (it_src != last_grant_it_src);
      }  // while(1)
    }
  };  // end class OutputArbiter

  // OutputBypass
  class OutputBypass : public sc_module, public Connections::Blocking_abs {
    SC_HAS_PROCESS(OutputBypass);

   protected:
    InterconnectBase<IM> &ic;
    IM_dest_t *p;
    IM_src_t *q;
    channels_begin_t &channels_begin;
    channels_middle_t &channels_middle;
    channels_end_t &channels_end;
    channels_valid_pairs_reverse_t &channels_valid_pairs_reverse;
    cycle_count_channel_t &cycle_count_channel;

   public:
    sc_in_clk clk;
    sc_in<bool> rst;

    explicit OutputBypass(
        InterconnectBase<IM> &ic_, IM_dest_t *p_, IM_src_t *q_,
        channels_begin_t &channels_begin_, channels_middle_t &channels_middle_,
        channels_end_t &channels_end_,
        channels_valid_pairs_reverse_t &channels_valid_pairs_reverse_,
        cycle_count_channel_t &cycle_count_channel_)
        : sc_module(sc_module_name(sc_gen_unique_name("output_arb"))),
          ic(ic_),
          p(p_),
          q(q_),
          channels_begin(channels_begin_),
          channels_middle(channels_middle_),
          channels_end(channels_end_),
          channels_valid_pairs_reverse(channels_valid_pairs_reverse_),
          cycle_count_channel(cycle_count_channel_),
          clk(sc_gen_unique_name("clk")),
          rst(sc_gen_unique_name("rst")) {
      CDCOUT("Info: Destination port "
                 << ic.get_dest_name(p)
                 << " only has one valid source. Adding a bypass to this "
                    "destination port in the interconnect modeler."
                 << endl,
             interconnect::kDebugLevel);

#ifdef CONNECTIONS_SIM_ONLY
      // SC_METHOD(do_bypass);
      declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                             SC_CURRENT_USER_MODULE, do_bypass);
      this->sensitive << channels_middle[q][p]->out_msg
                      << channels_middle[q][p]->out_val
                      << channels_end[p]->in_rdy;

      channels_end[p]->in_bound = true;
      channels_middle[q][p]->out_bound = true;
#endif

      SC_CTHREAD(output_bypass_run, clk.pos());
      async_reset_signal_is(rst, false);
    }

    void do_bypass() {
      bool is_val = channels_middle[q][p]->out_val.read();

      // Set val and msg respectively
      channels_end[p]->in_val.write(is_val);
      channels_middle[q][p]->out_rdy.write(channels_end[p]->in_rdy.read());

      // Only convert if is_val (to prevent X's)
      if (is_val) {
        channels_end[p]->in_msg.write(channels_middle[q][p]->out_msg.read());
      }
    }

    void output_bypass_run() {
      cycle_count_channel[q][p] = 0;

      while (1) {
        wait();
        if (channels_end[p]->in_val.read() && channels_end[p]->in_rdy.read())
          cycle_count_channel[q][p]++;  // Monitor
      }                                 // while(1)
    }
  };  // end class OutputBypass

};  // end class InterconnectBase

template <unsigned int IC_ID>
class Interconnect : public InterconnectBase<interconnect::IM_t> {
 public:
  Interconnect() : InterconnectBase<interconnect::IM_t>(IC_ID) {

    // Register with static vector of interconnect objects.
    // Ensure that only one matches the IC_ID.
    if (interconnect::get_ICManager().registered_ic.find(IC_ID) !=
        interconnect::get_ICManager().registered_ic.end()) {
      NVHLS_ASSERT_MSG(0,
                       "An interconnect with this IC_ID is already registered");
    }
    interconnect::get_ICManager().registered_ic[IC_ID] = this;
  }

  explicit Interconnect(const char *name)
      : InterconnectBase<interconnect::IM_t>(name, IC_ID) {
    // Register with static vector of interconnect objects.
    // Ensure that only one matches the IC_ID.
    if (interconnect::get_ICManager().registered_ic.find(IC_ID) !=
        interconnect::get_ICManager().registered_ic.end()) {
      NVHLS_ASSERT_MSG(0,
                       "An interconnect with this IC_ID is already registered");
    }
    interconnect::get_ICManager().registered_ic[IC_ID] = this;
  }
};

template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
class InterconnectMessageSCLV : public nvhls_message {
 public:
  typedef InterconnectBase<InterconnectMessageSCLV<MaxDataWidth, MaxDestCount> >
      interconnect_t;
  typedef typename interconnect_t::route_t route_t;
  typedef std::vector<typename interconnect_t::IM_dest_t *> multicast_t;

  // For marshaller
  enum {
    width = Wrapped<sc_lv<MaxDataWidth> >::width +
            Wrapped<typename interconnect_t::idx_t>::width +
            Wrapped<sc_lv<MaxDestCount> >::width
  };

 protected:
  sc_lv<MaxDataWidth> msg_bits;

  typename interconnect_t::idx_t src_idx;

 public:
  sc_lv<MaxDestCount> dest_bits;

  // Need to provide default constructor to be compatible with use as a Message
  InterconnectMessageSCLV() : msg_bits(0), src_idx(-1), dest_bits(0) {}

  template <unsigned int Size>
  void Marshall(Marshaller<Size> &m) {
    m &msg_bits;
    m &src_idx;
    m &dest_bits;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Getters and Setters

  ////////////////////
  // Message

  template <typename Message>
  void set_msg(interconnect_t &ic, Message &msg_) {
    set_msg(msg_);
  }

  template <typename Message>
  void set_msg(Message &msg_) {
    // Ensure sized appropriately.
    NVHLS_ASSERT(Wrapped<Message>::width <= MaxDataWidth);
    // Convert from Message to general sc_lv type
    Marshaller<Wrapped<Message>::width> marshaller;
    Wrapped<Message> wm(msg_);
    wm.Marshall(marshaller);
    msg_bits.range(Wrapped<Message>::width - 1, 0) = marshaller.GetResult();
  }

  template <typename Message>
  void get_msg(interconnect_t &ic, Message &m_) const {
    get_msg(m_);
  }

  template <typename Message>
  void get_msg(Message &m_) const {
    // Ensure sized appropriately.
    NVHLS_ASSERT(Wrapped<Message>::width <= MaxDataWidth);
    // Convert from general sc_lv type to Message
    sc_lv<Wrapped<Message>::width> mbits;
    mbits = msg_bits.range(Wrapped<Message>::width - 1, 0);
    Marshaller<Wrapped<Message>::width> marshaller(mbits);
    Wrapped<Message> result;
    result.Marshall(marshaller);
    m_ = result.val;
  }

  ////////////////////
  // Source

  void set_src_idx(interconnect_t &ic, typename interconnect_t::IM_src_t *p) {
    src_idx = ic.find_src_idx(p);
  }

  typename interconnect_t::IM_src_t *get_src_from_idx(interconnect_t &ic) {
    return ic.get_src_from_idx(src_idx);
  }

  ////////////////////
  // Destination

  // get_route() to replace this
  multicast_t get_multicast(interconnect_t &ic) {
    multicast_t a;

    // Ensure sized appropriately
    NVHLS_ASSERT(ic.get_dest_size() <= MaxDestCount);

    for (unsigned int i = 0; i < ic.get_dest_size(); i++) {
      if (dest_bits[i] == 1) {
        a.push_back(ic.get_dest_from_idx(i));
      }
    }
    return a;
  }

  void set_route(interconnect_t &ic, route_t *route) {
    dest_bits = 0;
    for (typename route_t::iterator it = route->begin(); it != route->end();
         ++it) {
      dest_bits[ic.find_dest_idx(*it)] = 1;
    }
  }

  bool does_support_multicast() { return true; }
};

#endif  // ifdef CONNECTIONS_SIM_ONLY

template <typename Message>
class OutPortPin : public sc_module {
  SC_HAS_PROCESS(OutPortPin);

 public:
  Connections::OutBlocking<Message> &port;
  Connections::InBlocking<Message> pin;

  explicit OutPortPin(Connections::OutBlocking<Message> &port_)
      : sc_module(sc_module_name(sc_gen_unique_name("out_port_to_in_pin"))),
        port(port_) {

    // SC_METHOD(do_bypass);
    declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                           SC_CURRENT_USER_MODULE, do_bypass);
    this->sensitive << pin.msg << pin.val << port.rdy;

#ifdef CONNECTIONS_SIM_ONLY
    port.disable_spawn();
    pin.disable_spawn();
#endif
  }

  void do_bypass() {
    bool is_val = pin.val.read();

    // Set val and msg respectively
    port.val.write(is_val);
    pin.rdy.write(port.rdy.read());

    // Only convert if is_val (to prevent X's)
    if (is_val) {
      port.msg.write(pin.msg.read());
    }
  }
};  // class OutPortPin

template <typename Message>
class InPortPin : public sc_module {
  SC_HAS_PROCESS(InPortPin);

 public:
  Connections::InBlocking<Message> &port;
  Connections::OutBlocking<Message> pin;

  explicit InPortPin(Connections::InBlocking<Message> &port_)
      : sc_module(sc_module_name(sc_gen_unique_name("in_port_to_out_pin"))),
        port(port_) {

    // SC_METHOD(do_bypass);
    declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                           SC_CURRENT_USER_MODULE, do_bypass);
    this->sensitive << port.msg << port.val << pin.rdy;

#ifdef CONNECTIONS_SIM_ONLY
    port.disable_spawn();
    pin.disable_spawn();
#endif
  }

  void do_bypass() {
    bool is_val = port.val.read();

    // Set val and msg respectively
    pin.val.write(is_val);
    port.rdy.write(pin.rdy.read());

    // Only convert if is_val (to prevent X's)
    if (is_val) {
      pin.msg.write(port.msg.read());
    }
  }
};  // class InPortPin

template <typename Message>
class OutPortPin2 : public sc_module {
  SC_HAS_PROCESS(OutPortPin2);

 public:
  Connections::OutBlocking<Message> port;
  Connections::InBlocking<Message> pin;

  explicit OutPortPin2()
      : sc_module(sc_module_name(sc_gen_unique_name("out_port_to_in_pin"))) {

    // SC_METHOD(do_bypass);
    declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                           SC_CURRENT_USER_MODULE, do_bypass);
    this->sensitive << pin.msg << pin.val << port.rdy;

#ifdef CONNECTIONS_SIM_ONLY
    port.disable_spawn();
    pin.disable_spawn();
#endif
  }

  void do_bypass() {
    bool is_val = pin.val.read();

    // Set val and msg respectively
    port.val.write(is_val);
    pin.rdy.write(port.rdy.read());

// Only convert if is_val (to prevent X's)
#ifndef __SYNTHESIS__
    if (is_val) {
#endif
      port.msg.write(pin.msg.read());
#ifndef __SYNTHESIS__
    }
#endif
  }
};  // class OutPortPin2

template <typename Message>
class InPortPin2 : public sc_module {
  SC_HAS_PROCESS(InPortPin2);

 public:
  Connections::InBlocking<Message> port;
  Connections::OutBlocking<Message> pin;

  explicit InPortPin2()
      : sc_module(sc_module_name(sc_gen_unique_name("in_port_to_out_pin"))) {

    // SC_METHOD(do_bypass);
    declare_method_process(do_bypass_handle, sc_gen_unique_name("do_bypass"),
                           SC_CURRENT_USER_MODULE, do_bypass);
    this->sensitive << port.msg << port.val << pin.rdy;

#ifdef CONNECTIONS_SIM_ONLY
    port.disable_spawn();
    pin.disable_spawn();
#endif
  }

  void do_bypass() {
    bool is_val = port.val.read();

    // Set val and msg respectively
    pin.val.write(is_val);
    port.rdy.write(pin.rdy.read());

// Only convert if is_val (to prevent X's)
#ifndef __SYNTHESIS__
    if (is_val) {
#endif
      pin.msg.write(port.msg.read());
#ifndef __SYNTHESIS__
    }
#endif
  }
};  // class InPortPin2

template <typename Message>
InPortPin<Message> *new_port_pin(Connections::InBlocking<Message> &port_) {
  return new InPortPin<Message>(port_);
}

template <typename Message>
OutPortPin<Message> *new_port_pin(Connections::OutBlocking<Message> &port_) {
  return new OutPortPin<Message>(port_);
}

template <typename Message>
InPortPin2<Message> *new_port_pin2(Connections::InBlocking<Message> &port_) {
  auto *port_pin = new InPortPin2<Message>();
  port_pin->port(port_);
  return port_pin;
}

template <typename Message>
OutPortPin2<Message> *new_port_pin2(Connections::OutBlocking<Message> &port_) {
  auto *port_pin = new OutPortPin2<Message>();
  port_pin->port(port_);
  return port_pin;
}
};

#ifndef __SYNTHESIS__

namespace interconnect {

template <typename PART_TYPE,
          Connections::connections_port_t PortType = AUTO_PORT>
class InterconnectInterface {
 public:
  sc_in_clk clk;
  sc_in<bool> rst;

  // IC_t *parent_ic;
  InterconnectBase<IM_t> *parent_ic;
  unsigned int part_id;
  bool is_bound;

  std::map<Connections::Blocking_abs *, GenericDestHandlerIface<IM_t> *>
      generic_iface_dests;
  std::map<Connections::Blocking_abs *, MSG_ID_t> msg_id_dests;
  std::map<Connections::Blocking_abs *, unsigned int> port_id_dests;
  std::map<Connections::Blocking_abs *, std::string> port_name_dests;
  std::map<Connections::Blocking_abs *, width_t> type_width_dests;
  std::map<Connections::Blocking_abs *, const char *> type_name_dests;
  std::map<Connections::Blocking_abs *, const char *> msg_name_dests;

  std::map<Connections::Blocking_abs *, GenericSrcHandlerIface<IM_t> *>
      generic_iface_srcs;
  std::map<Connections::Blocking_abs *, MSG_ID_t> msg_id_srcs;
  std::map<Connections::Blocking_abs *, unsigned int> port_id_srcs;
  std::map<Connections::Blocking_abs *, std::string> port_name_srcs;
  std::map<Connections::Blocking_abs *, width_t> type_width_srcs;
  std::map<Connections::Blocking_abs *, const char *> type_name_srcs;
  std::map<Connections::Blocking_abs *, const char *> msg_name_srcs;

#ifdef CONNECTIONS_SIM_ONLY

  InterconnectInterface() : clk("clk"), rst("rst"), is_bound(false) {}

  explicit InterconnectInterface(const char *name)
      : clk("clk"), rst("rst"), is_bound(false) {}

  template <unsigned int BIND_ID, typename MSG_TYPE>
  void Bind(Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p,
            unsigned int port_id) {
    ICMToSpecialized<IM_t, MSG_TYPE> *ij =
        new ICMToSpecialized<IM_t, MSG_TYPE>(p);

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }

    generic_iface_dests[&p] = ij;
    msg_id_dests[&p] = MSG_TYPE::msg_id;
    port_id_dests[&p] = port_id;
    port_name_dests[&p] = port_name;
    type_width_dests[&p] = Wrapped<typename MSG_TYPE::data_t>::width;
    type_name_dests[&p] = typeid(typename MSG_TYPE::data_t).name();
    msg_name_dests[&p] = MSG_TYPE::name;

    if (is_bound) {
      Bind2_Dest(&p, msg_id_dests[&p], port_id_dests[&p], msg_name_dests[&p]);
    }
  }

  template <unsigned int BIND_ID, typename MSG_TYPE>
  void Bind(Connections::OutBlocking<interconnect::Msg<MSG_TYPE> > &p,
            unsigned int port_id) {
    ICMFromSpecialized<IM_t, MSG_TYPE> *is =
        new ICMFromSpecialized<IM_t, MSG_TYPE>(p);

    std::string port_name = p.val.name();
    if (port_name.substr(port_name.length() - 4, 4) == "_val") {
      port_name.erase(port_name.length() - 4, 4);
    }

    generic_iface_srcs[&p] = is;
    msg_id_srcs[&p] = MSG_TYPE::msg_id;
    port_id_srcs[&p] = port_id;
    port_name_srcs[&p] = port_name;
    type_width_srcs[&p] = Wrapped<typename MSG_TYPE::data_t>::width;
    type_name_srcs[&p] = typeid(typename MSG_TYPE::data_t).name();
    msg_name_srcs[&p] = MSG_TYPE::name;

    if (is_bound) {
      Bind2_Src(&p, msg_id_srcs[&p], port_id_srcs[&p], msg_name_srcs[&p]);
    }
  }

  template <unsigned int IC_ID>
  void do_parent_bind(Interconnect<IC_ID> &ic, const sc_object &parent,
                      const unsigned int &part_id_) {
    assert(!is_bound);

    parent_ic = &ic;
    part_id = part_id_;
    is_bound = true;

    ic.part_names[PART_TYPE::part_id] = PART_TYPE::name;
    ic.part_to_part_type[part_id_] = PART_TYPE::part_id;
    ic.part_inst_names[part_id_] = parent.name();

    NVHLS_ASSERT(parent.name());

    for (std::map<Connections::Blocking_abs *, MSG_ID_t>::iterator it =
             msg_id_dests.begin();
         it != msg_id_dests.end(); ++it) {
      // Bind
      Bind2_Dest(it->first, it->second, port_id_dests[it->first],
                 msg_name_dests[it->first]);
      ic.part_msgs_dests[PART_TYPE::part_id][it->second].insert(
          port_id_dests[it->first]);
    }
    for (std::map<Connections::Blocking_abs *, MSG_ID_t>::iterator it =
             msg_id_srcs.begin();
         it != msg_id_srcs.end(); ++it) {
      Bind2_Src(it->first, it->second, port_id_srcs[it->first],
                msg_name_srcs[it->first]);
      ic.part_msgs_srcs[PART_TYPE::part_id][it->second].insert(
          port_id_srcs[it->first]);
    }
  }

 protected:
  void Bind2_Dest(Connections::Blocking_abs *p, unsigned int msg_id,
                  unsigned int port_id, const char *msg_name) {
    parent_ic->Bind(*generic_iface_dests[p]->im_ptr, msg_id_dests[p],
                    type_width_dests[p], type_name_dests[p], msg_name);
    parent_ic->set_dest_name(generic_iface_dests[p]->im_ptr,
                             port_name_dests[p]);

    parent_ic->add_to_dest_directory(*generic_iface_dests[p]->im_ptr, part_id,
                                     msg_id, port_id);

    parent_ic->dests_block_abs[generic_iface_dests[p]->im_ptr] = p;
  }

  void Bind2_Src(Connections::Blocking_abs *p, unsigned int msg_id,
                 unsigned int port_id, const char *msg_name) {
    parent_ic->Bind(*generic_iface_srcs[p]->im_ptr, msg_id_srcs[p],
                    type_width_srcs[p], type_name_srcs[p], msg_name);
    parent_ic->set_src_name(generic_iface_srcs[p]->im_ptr, port_name_srcs[p]);

    parent_ic->srcs_block_abs[generic_iface_srcs[p]->im_ptr] = p;
  }

 public:
#else

  InterconnectInterface() {
    NVHLS_ASSERT_MSG(0, "Need an HLS compatible InterconnectInterface!");
  }
  template <unsigned int PORT_ID, typename MSG_TYPE>
  void Bind(Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p_) {
    NVHLS_ASSERT(0);
  }
  template <unsigned int PORT_ID, typename MSG_TYPE>
  void Bind(Connections::OutBlocking<interconnect::Msg<MSG_TYPE> > &p_) {
    NVHLS_ASSERT(0);
  }

#endif
};

// Enable bottom-up (versus top-down) binding.
// template <unsigned int BIND_ID, unsigned int IC_ID, unsigned int PART_TYPE,
// Connections::connections_port_t IfacePortType>
template <unsigned int BIND_ID, unsigned int IC_ID, typename PART_TYPE,
          Connections::connections_port_t IfacePortType>
void do_interconnect_to_interface_bind(
    Interconnect<IC_ID> &ic,
    interconnect::InterconnectInterface<PART_TYPE, IfacePortType> &ic_interface,
    const sc_object &parent, const unsigned int &part_id) {
  ic_interface.do_parent_bind(ic, parent, part_id);
}

template <class Dummy>
unsigned int get_real_dest_idx(InterconnectBase<IM_t> &ic,
                               const unsigned int &dest_idx,
                               const unsigned int &dest_map_idx) {
  return ic.get_real_dest_idx(dest_idx, dest_map_idx);
}
};

#else

namespace interconnect {
template <typename PART_TYPE,
          Connections::connections_port_t PortType = AUTO_PORT>
class InterconnectInterface {};
};

#endif

#endif  // #ifdef __INTERCONNECT_H__
