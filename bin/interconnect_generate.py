#
# Python module to use from interconnect_designer.py
#
# SPDX-FileCopyrightText: Copyright (c) 2020-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This file isn't used directly, instead it is a module for interconnect_designer.py.

import sys
import math
import re
import itertools
import json
from functools import reduce
from collections import defaultdict

# ------------------------------- Utils --------------------------------------

# Naturally sort for object names and other uses.
def natsorted(x):
    def natsort_key(i):
        arr_key = list()
        for j in re.split('(\d*\.?\d+)', i):
            if j.isdecimal():
                arr_key.append(float(j))
            else:
                arr_key.append(j.lower())
        return arr_key
    return sorted(x, key=natsort_key)

# Pull out all the message widths present in the YAML_DATA.
def get_all_msg_widths(YAML_DATA):
    # Find all widths.
    msg_type_widths = dict()
    for msg_info in YAML_DATA['groups'].values():
        if 'name' in msg_info and '_width' in msg_info:
            msg_type_widths[msg_info['name']] = msg_info['_width']
    return msg_type_widths


# Calculate manhattan distance for global wire
def mdist_global_wire(YAML_DATA, msg_name, c1, c2, w=None):
    global __global_wire_coors
    if '__global_wire_coors' not in globals():
        __global_wire_coors = []

    if not w:
        msg_widths = get_all_msg_widths(YAML_DATA)
        w = msg_widths[msg_name]
    __global_wire_coors.append( (c1, c2, w) )
    return manhattan_distance(c1,c2)

def global_wire_max_tracks_x():
    global __global_wire_coors
    if '__global_wire_coors' not in globals():
        __global_wire_coors = []

    if len(__global_wire_coors) == 0:
        return (0, 0.0)

    # determine minimum epsilon unit on which to iterate
    all_x = sorted(sum([[elem[0][0], elem[1][0]] for elem in __global_wire_coors], []))
    epsil = max([abs(all_x[i] - all_x[i+1]) for i in range(len(all_x)-1)])
    if epsil == 0:
        epsil = 0.001

    # iterate through coors and count max hits
    def is_within(xt, x1, x2):
        if x1 <= x2:
            return x1 <= xt and xt <= x2
        else:
            return x1 >= xt and xt >= x2

    def num_at_x(xt):
        global __global_wire_coors
        return sum([elem[2] for elem in __global_wire_coors if is_within(xt, elem[0][0], elem[1][0])])

    all_w = []
    all_w_x_coor = []
    xt = float(min(all_x))
    while xt <= float(max(all_x)):
        all_w.append(num_at_x(xt))
        all_w_x_coor.append(xt)
        xt += epsil
    max_w = max(all_w)
    max_w_x_coor = all_w_x_coor[all_w.index(max_w)]
    return (int(max(all_w)), max_w_x_coor)

def manhattan_distance(c1, c2):
    """Compute manhattan distance between two coordinates"""
    x1, y1 = c1
    x2, y2 = c2
    return abs(x1-x2)+abs(y1-y2)

def to_ranges(nums):
    nums = sorted(set(nums)) # Set to remove duplicates, sorted so we can iterate over consistently
    if len(nums) == 0:
        return
    range_start = nums[0] # Seed start
    last_number = range_start
    for current_number in nums[1:]:
        # Check if not contiguous
        if current_number != last_number + 1:
            yield range_start, last_number
            range_start = current_number
        last_number = current_number
    yield range_start, last_number         

def report_pins_of_inst(pin, inst_id, inst_name_to_part_id):
    global __pins_to_insts
    if '__pins_to_insts' not in globals():
        __pins_to_insts = {}

    found_insts = [x[0] for x in inst_name_to_part_id.items() if x[1] == inst_id]
    assert len(found_insts) != 0, "Couldn't find a name for inst_id {}".format(inst_id)
    assert len(found_insts) <= 1, "Found too many names for inst_id {}".format(inst_id)
    inst_name = found_insts[0]
    if pin in __pins_to_insts.keys():
        assert __pins_to_insts[pin] == inst_name, "Pin {pin} already assigned to {old} but now {new}".format(pin=pin, old=__pins_to_insts[pin], new=inst_name)
    else:
        __pins_to_insts[pin] = inst_name

# v---------- Interconnect Collate functions --------------v
def ic_add(ic_id, arr, x):
    arr.setdefault(ic_id, [])
    arr[ic_id].append(x)

def ic_collate_select(ic_id, arr, *block_tuple):
    global __glob_ic_collate_ic_id, __glob_ic_collate_arr, __glob_ic_collate_block_tuple, __glob_ic_collate

    if '__glob_ic_collate' not in globals():
        __glob_ic_collate = {}
    __glob_ic_collate.setdefault(ic_id, {})
    __glob_ic_collate[ic_id].setdefault(id(arr), [])

    __glob_ic_collate_ic_id = ic_id
    __glob_ic_collate_arr = __glob_ic_collate[ic_id][id(arr)]
    __glob_ic_collate_block_tuple = block_tuple

def ic_collate_add(body_string):
    global __glob_ic_collate_ic_id, __glob_ic_collate_arr, __glob_ic_collate_block_tuple
    __glob_ic_collate_arr.append(__glob_ic_collate_block_tuple + (body_string,))

def __ic_collate_recurse(out_s, arr, root_tuple=(), s_indent_level=0):
    # Filter to elements at this level
    this_arr = [x for x in arr if (len(x) - 1) == len(root_tuple) and x[0:len(root_tuple)] == root_tuple]
    # Filter to sub eleemnts
    sub_arr  = [x for x in arr if (len(x) - 1) > len(root_tuple) and x[0:len(root_tuple)] == root_tuple]

    # Write out any tuples at this level.
    for body_string in [x[-1] for x in this_arr]:
        out_s.append(s_indent_level*' ' + body_string)

    # Descend one level of tuples.
    for block_string in sorted(set([x[len(root_tuple)] for x in sub_arr])):
        # Begin block loop
        out_s.append(s_indent_level*' ' + "{block_string} {{".format(block_string=block_string))
        # Block body recusirve
        __ic_collate_recurse(out_s, sub_arr, root_tuple + (block_string,), s_indent_level=s_indent_level+4)
        # End block loop
        out_s.append(s_indent_level*' ' + "}}".format())

def ic_collate_recurse(ic_id, arr, *out_s_s):
    global __glob_ic_collate

    arr.setdefault(ic_id,[])

    if '__glob_ic_collate' not in globals() \
       or ic_id not in __glob_ic_collate.keys() \
       or id(arr) not in __glob_ic_collate[ic_id].keys():
        return

    # Always add strings to arr, optionally to other dicts in command line too,
    # so that we can add it to a super set
    for out_s in (arr,) + out_s_s:
        out_s.setdefault(ic_id,[])
        __ic_collate_recurse(out_s[ic_id], __glob_ic_collate[ic_id][id(arr)])



# ------------------------------- NoC --------------------------------------

class Flit:
    def create_flit(self, YAML_DATA, topo_name, topo_groups, supermsg_dest_maps, supermsg_src_maps, bus_width, joiner_splitter_id):
        # Find all widths.
        msg_type_widths = get_all_msg_widths(YAML_DATA)
        
        supermsg_master_dest_map = sorted(list( set( sum(supermsg_dest_maps.values(), list()) ) ))
        supermsg_master_insts = list()
        supermsg_dest_map_to_inst = list()
        for (part_name, inst_id, port_id) in supermsg_master_dest_map:
            if inst_id not in supermsg_master_insts:
                supermsg_master_insts.append(inst_id)
            supermsg_dest_map_to_inst.append(supermsg_master_insts.index(inst_id))

        max_msg_width = list()
        for msg_name in topo_groups:
            max_msg_width.append(int(msg_type_widths[msg_name]))

        s_declarations = str()
        s_declarations += "    class FLITMSG_{joiner_splitter_id};\n".format(joiner_splitter_id=joiner_splitter_id)


        a_supermsgs = list()
        a_supermsgs.append("IC_ADD_MESSAGE_TYPE(FLITMSG_{joiner_splitter_id}, FLITMSG_{joiner_splitter_id})".format(joiner_splitter_id=joiner_splitter_id))

        a_icg_msg_types = list()
        a_icg_msg_types.append( ("FLITMSG_{}".format(str(joiner_splitter_id)), "FLITMSG_{}".format(str(joiner_splitter_id))) )

        FLIT_ID_SIZE=2
        MCAST_DEST_SIZE = max(len(supermsg_master_dest_map),1) + 1 # +1 for NoP
        MSG_TYPE_SIZE = max(math.ceil(math.log2(len(topo_groups))),1)
        header_size=MSG_TYPE_SIZE + MCAST_DEST_SIZE + 1 # dest bits + msg type bits + 1 bit to indicate mcast
        FLIT_PAYLOAD_SIZE = bus_width - FLIT_ID_SIZE
        MSG_WIDTH=max(max_msg_width)
        TOTAL_MSG_WIDTH=header_size+MSG_WIDTH

        assert (FLIT_PAYLOAD_SIZE >= header_size), "Bus width must be at least {} to accomodate header!".format(header_size+2)

        num_flits=math.ceil(TOTAL_MSG_WIDTH/FLIT_PAYLOAD_SIZE)

        print("Info: Topo '{topo_name}' with bus width {bus_width} has a header size of {header_size} and packet data width of {max_msg_width}. Max number of {num_flits} flits in a packet." \
        .format(topo_name=topo_name,
                header_size=header_size,
                max_msg_width=MSG_WIDTH,
                bus_width=bus_width,
                num_flits=num_flits))
        
        s_definitions = """\
    template <>
    class Msg<interconnect_config_icg::msgs::FLITMSG_{joiner_splitter_id}> : public Flit<{bus_width}, 0, 0, 0, FlitId2bit, WormHole> {{}};
        """.format(joiner_splitter_id=joiner_splitter_id,
                   bus_width=bus_width)

        return (s_declarations, s_definitions, a_icg_msg_types)


    def create_joiner_flit(self, YAML_DATA, topo_options, topo_groups, part_name, supermsg_dest_maps, master_msg_list, local_msg_list, bus_width, joiner_splitter_id, grout=False):
        supermsg_master_dest_map = sorted(list( set( sum(supermsg_dest_maps.values(), list()) ) ))

        # Find all widths.
        msg_type_widths = dict()
        for msg_info in YAML_DATA['groups'].values():
            if 'name' in msg_info and '_width' in msg_info:
                msg_type_widths[msg_info['name']] = msg_info['_width']

        max_packet_size = 0

        a_joiner_instances = list()
        a_joiner_reset = list()
        a_joiner_transfernb = list()
        a_joiner_if_ladder = list()
        a_joiner_if_ladder2 = list()
        a_joiner_push_predicate = list()
        a_joiner_push_increment = list()
        for seq_id in range(len(master_msg_list)):
            (msg_name, port_id) = master_msg_list[seq_id]
            if msg_name not in local_msg_list:
                continue
            if grout:
                a_joiner_instances.append("Connections::InBuffered<interconnect::Msg<interconnect_config::msgs::{msg_name}>,1> in_port_{msg_name_lower}_{port_id};".format(msg_name=msg_name,
                                                                                                                                                    msg_name_lower=msg_name.lower(),
                                                                                                                                                    port_id=port_id))
                in_port_postfix = ""
            else:
                a_joiner_instances.append("Connections::CombinationalBufferedPorts<interconnect::Msg<interconnect_config::msgs::{msg_name}>,1,0> in_port_{msg_name_lower}_{port_id};".format(msg_name=msg_name,
                                                                                                                                                    msg_name_lower=msg_name.lower(),
                                                                                                                                                    port_id=port_id))
                in_port_postfix = "Read"

            a_joiner_reset.append("in_port_{msg_name_lower}_{port_id}.Reset{in_port_postfix}();".format(msg_name=msg_name,
                                                                                        msg_name_lower=msg_name.lower(),
                                                                                        port_id=port_id,
                                                                                        in_port_postfix=in_port_postfix))
            
            a_joiner_transfernb.append("in_port_{msg_name_lower}_{port_id}.TransferNB{in_port_postfix}();".format(msg_name=msg_name,
                                                                                                msg_name_lower=msg_name.lower(),
                                                                                                port_id=port_id,
                                                                                                in_port_postfix=in_port_postfix))

            if topo_options['router_module'] == 'WHVCSourceRouter':
                # TODO: Need to better refer to the Credit_ret_t, but it's buried in the specific router.
                a_joiner_instances.append("typedef NVUINTW(1) Credit_ret_t;") # TODO need to use proper type here
                a_joiner_instances.append("typedef NVUINTW(4) Credit_t;") # TODO need to know number of credits allowed here, based on max packet size. Probably tie this into its attached router.
                a_joiner_instances.append("Credit_t credit_reg_egress[1];") # 1 = number of virtual channels
                a_joiner_instances.append("Connections::In<Credit_ret_t> in_credit;")

                a_joiner_reset.append("in_credit.Reset();")
                a_joiner_reset.append("credit_reg_egress[0] = 2;") # TODO set this to correct value

                a_joiner_transfernb.append("""
                Credit_ret_t temp_credit;
                if(in_credit.PopNB(temp_credit)) {
                    credit_reg_egress[0] += temp_credit;
                    NVHLS_ASSERT(credit_reg_egress[0] > temp_credit); // Overflow detetion
                }

                // TODO: Need to increment credit_regs when sending etc
                // credit_reg_egress-- everytime we send a packet. Starts at buffer size.
                // credit_reg_ingress++ everytime we receive a packet. Starts at 0.
                """)

                a_joiner_push_predicate.append(" && credit_reg_egress[0] > 0")
                a_joiner_push_increment.append("--credit_reg_egress[0];")


            # Calculate number of flit iters for this message type
            # Assumes a 2 bit extra for flit id on every flit
            # This is not included in the head payload
            FLIT_ID_SIZE=2
            MCAST_DEST_SIZE = max(len(supermsg_master_dest_map),1) + 1 # +1 for NoP
            MSG_TYPE_SIZE = max(math.ceil(math.log2(len(topo_groups))),1)
            header_size=MSG_TYPE_SIZE + MCAST_DEST_SIZE + 1 # dest bits + msg type bits + 1 bit to indicate mcast
            FLIT_PAYLOAD_SIZE = bus_width - FLIT_ID_SIZE
            MSG_WIDTH=msg_type_widths[msg_name]
            TOTAL_MSG_WIDTH=header_size+MSG_WIDTH

            assert (bus_width-FLIT_ID_SIZE) >= header_size, "bus_width (excl. flit id size) of {FLIT_PAYLOAD_SIZE} needs to be greater than header payload size of {header_size}!" \
                .format(FLIT_PAYLOAD_SIZE=FLIT_PAYLOAD_SIZE, header_payload_size=header_size)
            
            num_flits=math.ceil(TOTAL_MSG_WIDTH/FLIT_PAYLOAD_SIZE)
            header_flit_data_size = min([FLIT_PAYLOAD_SIZE - header_size, MSG_WIDTH]) # how many message bits we can fit in header packet
            tail_flit_data_size = int(TOTAL_MSG_WIDTH % (bus_width-FLIT_ID_SIZE))

            assert num_flits >= 1
            max_packet_size = max([max_packet_size, num_flits])

            # Flag to indicate if we can get away with a single flit packet
            is_single = (num_flits == 1)

            # Flat to indicacte if we can include message bits in header flit
            has_header_flit_msg_bits = (header_flit_data_size > 0)
            has_tail_flit_msg_bits = (not is_single)
            if not has_tail_flit_msg_bits:
                tail_flit_data_size = 0

            # Borrowed from create_superflit
            local_to_master_dest_map_a = list()
            for v in supermsg_dest_maps[msg_name]:
                local_to_master_dest_map_a.append(supermsg_master_dest_map.index(v))

            # Need to decode port_id here (add as a function to SuperMsg, static const unsigned int lookup table? except,
            # we don't know the local dest_id here.. so we would need to decode that as well. Unless we add port id as both a separate
            # dest AND a separate message type? but then we cannot multiplex... so we do need to figure out a mechanism to decode it.
            #
            # Could we mask or add a bridge type?
            #
            # Probably cleanest to have a part_id sc_in (or jtag) that all partitions receive, and that will
            # tell splitter where to decode for the port_id. 
            #
            # Through a function in supermsg that given a part_id AND port_id (is_port_id(part_id, port_id)) it will return true or false.
            a_joiner_if_ladder.append("""\
    if (! in_port_{msg_name_lower}_{port_id}.Empty{in_port_postfix}()) {{
                        did_pop = true;
                        current_msg_type = {seq_id};
                    }}""".format(seq_id=seq_id,
                                msg_name=msg_name,
                                msg_name_lower=msg_name.lower(),
                                port_id=port_id,
                                MSG_WIDTH=MSG_WIDTH,
                                TOTAL_MSG_WIDTH=TOTAL_MSG_WIDTH,
                                is_single=is_single,
                                has_header_flit_msg_bits=has_header_flit_msg_bits,
                                header_flit_data_size=header_flit_data_size,
                                num_flits=num_flits,
                                tail_flit_data_size=tail_flit_data_size,
                                in_port_postfix=in_port_postfix) )

            a_joiner_if_ladder2.append("""\
    if (current_msg_type == {seq_id}) {{
                        // Set Flit id
                        if({is_single}) {{
                            flit.flit_id.set(FlitId2bit::SNGL);
                        }} else {{
                            if(current_flit_iter == 0) {{
                                flit.flit_id.set(FlitId2bit::HEAD);
                            }} else if(current_flit_iter == ({num_flits}-1)) {{
                                flit.flit_id.set(FlitId2bit::TAIL);
                            }} else {{
                                flit.flit_id.set(FlitId2bit::BODY);
                            }}
                        }}

                        // Payload information
                        IC_MESSAGE({msg_name}) msg_peek;
                        msg_peek = in_port_{msg_name_lower}_0.Peek{in_port_postfix}();

                        NVUINTW(Wrapped<interconnect_config::msgs::{msg_name}::data_t>::width) msg_peek_bits;
                        msg_peek_bits = TypeToNVUINT(msg_peek.get_msg());
                        // cout << sc_time_stamp() << " " << name() << " HERE2 PEEK current_flit_iter=" << current_flit_iter << hex << "  data= "<< msg_peek_bits.to_uint64() << dec << endl;

                        if(current_flit_iter == 0) {{
                            // Header flit information
                            NVUINTW({header_size}) header_bits=0;
                            NVUINTW({MSG_TYPE_SIZE}) msg_type_bits={seq_id};
                            NVUINTW({MCAST_DEST_SIZE}) mcast_dest_bits=0;
                            
                            // Add mcast route
                            static const unsigned int m[{num_local_dests}] = {{ {local_to_master_dest_map_s} }};
                            #pragma hls_unroll yes
                            for(unsigned int i=0; i < {num_local_dests}; ++i) {{
                                mcast_dest_bits[m[i]+1] = msg_peek.dest_bits[i] == 1 ? 1 : 0;
                            }}

                            // Set route type to mcast
                            header_bits[{MCAST_DEST_SIZE}] = 1;
                            // Set NoP destination to chip_id = 0
                            mcast_dest_bits[0] = 1;

                            header_bits = nvhls::set_slc(header_bits, msg_type_bits, {MCAST_DEST_SIZE}+1);
                            header_bits = nvhls::set_slc(header_bits, mcast_dest_bits, 0);

                            flit.data = nvhls::set_slc(flit.data, header_bits, 0);

                            if({has_header_flit_msg_bits}) {{
                                flit.data = nvhls::set_slc(flit.data, nvhls::get_slc<{has_header_flit_msg_bits} ? {header_flit_data_size} : 1>(msg_peek_bits, 0), {header_size});
                            }}
                        }} else if(current_flit_iter == ({num_flits}-1)) {{
                            flit.data = nvhls::set_slc(flit.data, nvhls::get_slc<{tail_flit_data_size}>(msg_peek_bits, {FLIT_PAYLOAD_SIZE}*(current_flit_iter-1) + {header_flit_data_size}), 0);
                        }} else {{
                            flit.data = nvhls::get_slc<{FLIT_PAYLOAD_SIZE}>(msg_peek_bits, {FLIT_PAYLOAD_SIZE}*(current_flit_iter-1) + {header_flit_data_size});
                        }}

                        if(current_flit_iter == ({num_flits}-1)) {{
                            // cout << sc_time_stamp() << " " << name() << " HERE2 Pushing current_flit_iter=" << current_flit_iter << " output msg_type=" << current_msg_type << " to=" << hex << msg_peek.dest_bits.to_uint64() << " data= "<< msg_peek_bits.to_uint64() << dec << endl;
                            in_port_{msg_name_lower}_0.Pop();
                            did_pop = false; // Handled the pop
                        }}
                    }}""".format(seq_id=seq_id,
                                msg_name=msg_name,
                                msg_name_lower=msg_name.lower(),
                                port_id=port_id,
                                is_single=("true" if is_single else "false"),
                                header_size=header_size,
                                MSG_TYPE_SIZE=MSG_TYPE_SIZE,
                                MCAST_DEST_SIZE=MCAST_DEST_SIZE,
                                has_header_flit_msg_bits=("true" if has_header_flit_msg_bits else "false"),
                                header_flit_data_size=header_flit_data_size,
                                num_flits=num_flits,
                                tail_flit_data_size=max([tail_flit_data_size, 1]),
                                bus_width=bus_width,
                                FLIT_PAYLOAD_SIZE=FLIT_PAYLOAD_SIZE,
                                num_local_dests=len(local_to_master_dest_map_a),
                                local_to_master_dest_map_s=', '.join([str(x) for x in local_to_master_dest_map_a]),
                                in_port_postfix=in_port_postfix) )

        s_joiner_instances = ''.join(['        {}\n'.format(x) for x in a_joiner_instances])
        s_joiner_reset = ''.join(['            {}\n'.format(x) for x in a_joiner_reset])
        s_joiner_transfernb = ''.join(['                {}\n'.format(x) for x in a_joiner_transfernb])
        s_joiner_if_ladder = '                ' + ' else '.join(a_joiner_if_ladder)
        s_joiner_if_ladder2 = '                ' + ' else '.join(a_joiner_if_ladder2)
        s_joiner_push_predicate = ''.join(a_joiner_push_predicate)
        s_joiner_push_increment = ''.join(['        {}\n'.format(x) for x in a_joiner_push_increment])
        s = """\
    #pragma hls_ungroup
    template <Connections::connections_port_t PortType = AUTO_PORT>
    class JoinerFlit_{part_name}_{joiner_splitter_id}  : public sc_module {{
        SC_HAS_PROCESS(JoinerFlit_{part_name}_{joiner_splitter_id});

        public:

        typedef typename interconnect_config_icg::msgs::FLITMSG_{joiner_splitter_id} MSG_TYPE;

        sc_in_clk clk;
        sc_in<bool> rst;

        Connections::Out<interconnect::Msg<MSG_TYPE>, PortType> out_port;

{s_joiner_instances}

        JoinerFlit_{part_name}_{joiner_splitter_id}() : sc_module(sc_module_name(sc_gen_unique_name("joiner"))) {{
            SC_CTHREAD(run, clk.pos());
            async_reset_signal_is(rst, false);
        }}


        // void PushOutFlit(Flit_t flit_reg) {{
        void PushOutFlit(interconnect::Msg<MSG_TYPE> flit_reg) {{
            CDCOUT(sc_time_stamp() << " " << name() << " Pushing Output Flit: " << flit_reg << endl, kDebugLevel);
            out_port.Push(flit_reg);
            CDCOUT(sc_time_stamp() << " " << name() << " Pushed Output Flit: " << flit_reg << endl, kDebugLevel);
        }}

        void run() {{
            // out_port.ResetWrite();
            out_port.Reset();

{s_joiner_reset}

            bool did_pop = false;
            int current_msg_type = -1; // -1 is reserved for no message
            unsigned int current_flit_iter = 0;

            #pragma hls_pipeline_init_interval 1
            #pragma pipeline_stall_mode flush
            while(1) {{
                wait();

                // New message to push
                interconnect::Msg<MSG_TYPE> im;

{s_joiner_transfernb}

                if(!did_pop) {{
#ifndef __SYNTHESIS__
                    NVHLS_ASSERT(current_msg_type == -1);
#endif
                    current_flit_iter = 0;

{s_joiner_if_ladder}
                }}

                // Push new message
                if(did_pop{s_joiner_push_predicate}) {{
                    //Flit_t flit;
                    interconnect::Msg<MSG_TYPE> flit;

                    // Always zero data in case we don't set upper bits for certain message types
                    flit.data = 0;

{s_joiner_if_ladder2}

                    // Push flit
                    // cout << sc_time_stamp() << " " << name() << " HERE Pushing msg_type=" << current_msg_type << " Flit<" << hex << flit.flit_id << "," << flit.data.to_uint64() << ">" << dec << endl;
                    PushOutFlit(flit);
                    // cout << sc_time_stamp() << " " << name() << " HERE Pushed!" << endl;

                    // Iterate
                    current_flit_iter++;
                    {s_joiner_push_increment}

                    // Iteration logic
                    if(!did_pop) {{
                        current_msg_type = -1;
                    }}
                }}
            }}
        }}
    }};""".format(part_name=part_name,
           joiner_splitter_id=joiner_splitter_id,
           s_joiner_instances=s_joiner_instances,
           s_joiner_reset=s_joiner_reset,
           s_joiner_transfernb=s_joiner_transfernb,
           s_joiner_if_ladder=s_joiner_if_ladder,
           s_joiner_if_ladder2=s_joiner_if_ladder2,
           s_joiner_push_predicate=s_joiner_push_predicate,
           s_joiner_push_increment=s_joiner_push_increment)
    
        return (s, max_packet_size)


    def create_splitter_flit(self, YAML_DATA, topo_options, topo_groups, part_name, supermsg_dest_maps, master_msg_list, local_msg_list, bus_width, joiner_splitter_id, grout=False):
        supermsg_master_dest_map = sorted(list( set( sum(supermsg_dest_maps.values(), list()) ) ))

        # Find all widths.
        msg_type_widths = dict()
        for msg_info in YAML_DATA['groups'].values():
            if 'name' in msg_info and '_width' in msg_info:
                msg_type_widths[msg_info['name']] = msg_info['_width']

        max_packet_size = 0

        a_splitter_instances = list()
        a_splitter_reset = list()
        a_splitter_transfernb = list()
        a_splitter_if_ladder = list()
        a_splitter_if_ladder2 = list()
        a_splitter_pop_increment = list()

        a_splitter_if_ladder2.append("""\
if (current_msg_type == -1) {{
                        // Header should be handled above
#ifndef __SYNTHESIS__
                        NVHLS_ASSERT(0);
#endif
                    }}""".format())

        for seq_id in range(len(master_msg_list)):
            (msg_name, port_id) = master_msg_list[seq_id]
            if msg_name not in local_msg_list:
                continue
            if grout:
                a_splitter_instances.append("Connections::OutBuffered<interconnect::Msg<interconnect_config::msgs::{msg_name}> , 2> out_port_{msg_name_lower}_{port_id};".format(msg_name=msg_name,
                                                                                                                                                    msg_name_lower=msg_name.lower(),
                                                                                                                                                    port_id=port_id))
                out_port_postfix=""
            else:
                a_splitter_instances.append("Connections::CombinationalBufferedPorts<interconnect::Msg<interconnect_config::msgs::{msg_name}> , 0, 2> out_port_{msg_name_lower}_{port_id};".format(msg_name=msg_name,
                                                                                                                                                    msg_name_lower=msg_name.lower(),
                                                                                                                                                    port_id=port_id))
                out_port_postfix="Write"
            a_splitter_reset.append("out_port_{msg_name_lower}_{port_id}.Reset{out_port_postfix}();".format(msg_name=msg_name,
                                                                                msg_name_lower=msg_name.lower(),
                                                                                port_id=port_id,
                                                                                out_port_postfix=out_port_postfix))
            a_splitter_transfernb.append("out_port_{msg_name_lower}_{port_id}.TransferNB{out_port_postfix}();".format(msg_name=msg_name,
                                                                                                      msg_name_lower=msg_name.lower(),
                                                                                                      port_id=port_id,
                                                                                                      out_port_postfix=out_port_postfix))

            if topo_options['router_module'] == 'WHVCSourceRouter':
                # TODO: Need to better refer to the Credit_ret_t, but it's buried in the specific router.
                a_splitter_instances.append("typedef NVUINTW(1) Credit_ret_t;") # TODO need to use proper type here
                a_splitter_instances.append("typedef NVUINTW(4) Credit_t;") # TODO need to know number of credits allowed here, based on max packet size. Probably tie this into it's attached router type.
                a_splitter_instances.append("Connections::Out<Credit_ret_t> out_credit;")
                a_splitter_instances.append("Credit_t credit_reg_ingress[1];") # 1 = number of virtual channels
                a_splitter_reset.append("out_credit.Reset();")
                a_splitter_reset.append("credit_reg_ingress[0] = 0;") # This is always 0, since we increment *up*.

                a_splitter_transfernb.append("""
                // send out credits
                if(credit_reg_ingress[0] > 0) {
                    Credit_ret_t temp = 1;
                    if(out_credit.PushNB(temp)) {
                        --credit_reg_ingress[0];
                    }
                }

                """)

                a_splitter_pop_increment.append("++credit_reg_ingress[0];")
                a_splitter_pop_increment.append("NVHLS_ASSERT(credit_reg_ingress[0] > 0); // Overflow detection")


            # Calculate number of flit iters for this message type
            # Assumes a 2 bit extra for flit id on every flit
            # This is not included in the head payload
            FLIT_ID_SIZE=2
            MCAST_DEST_SIZE = max(len(supermsg_master_dest_map),1) + 1 # +1 for NoP
            MSG_TYPE_SIZE = max(math.ceil(math.log2(len(topo_groups))),1)
            header_size=MSG_TYPE_SIZE + MCAST_DEST_SIZE + 1 # dest bits + msg type bits + 1 + 1 for mcast indicator bit
            FLIT_PAYLOAD_SIZE = bus_width - FLIT_ID_SIZE
            MSG_WIDTH=msg_type_widths[msg_name]
            TOTAL_MSG_WIDTH=header_size+MSG_WIDTH

            assert (bus_width-FLIT_ID_SIZE) >= header_size, "bus_width (excl. flit id size) of {FLIT_PAYLOAD_SIZE} needs to be greater than header payload size of {header_size}!" \
                .format(FLIT_PAYLOAD_SIZE=FLIT_PAYLOAD_SIZE, header_payload_size=header_size)
            
            num_flits=math.ceil(TOTAL_MSG_WIDTH/FLIT_PAYLOAD_SIZE)
            header_flit_data_size = min([FLIT_PAYLOAD_SIZE - header_size, MSG_WIDTH]) # how many message bits we can fit in header packet
            tail_flit_data_size = int(TOTAL_MSG_WIDTH % (bus_width-FLIT_ID_SIZE))

            assert num_flits >= 1
            max_packet_size = max([max_packet_size, num_flits])

            # Flag to indicate if we can get away with a single flit packet
            is_single = (num_flits == 1)

            # Flat to indicacte if we can include message bits in header flit
            has_header_flit_msg_bits = (header_flit_data_size > 0)
            has_tail_flit_msg_bits = (not is_single)
            if not has_tail_flit_msg_bits:
                tail_flit_data_size = 0

            # from create_supermsg
            local_to_master_dest_map_a = list()
            for v in supermsg_dest_maps[msg_name]:
                local_to_master_dest_map_a.append(supermsg_master_dest_map.index(v))

            s_splitter_pop_increment = ''.join(['        {}\n'.format(x) for x in a_splitter_pop_increment])

            # Need to decode port_id here (add as a function to SuperMsg, static const unsigned int lookup table? except,
            # we don't know the local dest_id here.. so we would need to decode that as well. Unless we add port id as both a separate
            # dest AND a separate message type? but then we cannot multiplex... so we do need to figure out a mechanism to decode it.
            #
            # Could we mask or add a bridge type?
            #
            # Probably cleanest to have a part_id sc_in (or jtag) that all partitions receive, and that will
            # tell splitter where to decode for the port_id.
            #
            # Through a function in supermsg that given a part_id AND port_id (is_port_id(part_id, port_id)) it will return true or false.
            a_splitter_if_ladder2.append("""\
if (current_msg_type == {seq_id}) {{
                        static const unsigned int m[{num_local_dests}] = {{ {local_to_master_dest_map_s} }};
                        if({is_single}) {{
#ifndef __SYNTHESIS__
                                NVHLS_ASSERT(flit.flit_id.isSingle());
#endif

                                if(out_port_{msg_name_lower}_0.Full{out_port_postfix}()) {{
                                    current_flit_iter--;
                                }} else {{
                                    full_msg_bits = nvhls::set_slc(full_msg_bits, nvhls::get_slc<{has_header_flit_msg_bits} ? {header_flit_data_size} : 1>(flit.data, {header_size}), 0);
                                    
                                    IC_MESSAGE({msg_name}) im = NVUINTToType<interconnect_config::msgs::{msg_name}::data_t>(nvhls::get_slc<Wrapped<interconnect_config::msgs::{msg_name}::data_t>::width>(full_msg_bits, 0));

                                    #pragma hls_unroll yes
                                    for(unsigned int i=0; i < {num_local_dests}; ++i) {{
                                        im.dest_bits[i] = mcast_dest_bits[m[i]+1] == 1 ? 1 : 0;
                                    }}

                                    // cout << sc_time_stamp() << " " << name() << " HERE2 Popping output msg_type=" << current_msg_type << " to=" << hex << im.dest_bits.to_uint64() << " data= "<< (nvhls::get_slc<Wrapped<interconnect_config::msgs::{msg_name}::data_t>::width>(full_msg_bits, 0)).to_uint64() << dec << endl;

                                    out_port_{msg_name_lower}_0.Push(im);
                                    // cout << sc_time_stamp() << " " << name() << " HERE Popped" << endl;
                                    current_msg_type = -1;
                                    in_port.Pop();
                                    {s_splitter_pop_increment}
                                }}
                        }} else {{
                            if(current_flit_iter == 0) {{
#ifndef __SYNTHESIS__
                                NVHLS_ASSERT(flit.flit_id.isHeader());
#endif
                                if({has_header_flit_msg_bits}) {{
                                    full_msg_bits = nvhls::set_slc(full_msg_bits, nvhls::get_slc<{has_header_flit_msg_bits} ? {header_flit_data_size} : 1>(flit.data, {header_size}), 0);
                                }}
                                in_port.Pop();
                                {s_splitter_pop_increment}
                            }} else if(current_flit_iter == ({num_flits}-1)) {{
#ifndef __SYNTHESIS__
                                NVHLS_ASSERT(flit.flit_id.isTail());
#endif
                                if(out_port_{msg_name_lower}_0.Full{out_port_postfix}()) {{
                                    current_flit_iter--;
                                }} else {{
                                    full_msg_bits = nvhls::set_slc(full_msg_bits, nvhls::get_slc<{tail_flit_data_size}>(flit.data, 0), {FLIT_PAYLOAD_SIZE}*(current_flit_iter-1) + {header_flit_data_size});
                                    
                                    IC_MESSAGE({msg_name}) im = NVUINTToType<interconnect_config::msgs::{msg_name}::data_t>(nvhls::get_slc<Wrapped<interconnect_config::msgs::{msg_name}::data_t>::width>(full_msg_bits, 0));

                                    #pragma hls_unroll yes
                                    for(unsigned int i=0; i < {num_local_dests}; ++i) {{
                                        im.dest_bits[i] = mcast_dest_bits[m[i]+1] == 1 ? 1 : 0;
                                    }}

                                    // cout << sc_time_stamp() << " " << name() << " HERE2 Popping output msg_type=" << current_msg_type << " to=" << hex << im.dest_bits.to_uint64() << " data= "<< (nvhls::get_slc<Wrapped<interconnect_config::msgs::{msg_name}::data_t>::width>(full_msg_bits, 0)).to_uint64() << dec << endl;
                                    out_port_{msg_name_lower}_0.Push(im);
                                    // cout << sc_time_stamp() << " " << name() << " HERE Pushed" << endl;
                                    current_msg_type = -1;
                                    in_port.Pop();
                                    {s_splitter_pop_increment}
                                }}
                            }} else {{
#ifndef __SYNTHESIS__
                                NVHLS_ASSERT(flit.flit_id.isBody());
#endif
                                full_msg_bits = nvhls::set_slc(full_msg_bits, flit.data, {FLIT_PAYLOAD_SIZE}*(current_flit_iter-1) + {header_flit_data_size});
                                in_port.Pop();
                                {s_splitter_pop_increment}
                            }}
                        }}
                    }}""".format(seq_id=seq_id,
                                msg_name=msg_name,
                                msg_name_lower=msg_name.lower(),
                                port_id=port_id,
                                is_single=("true" if is_single else "false"),
                                has_header_flit_msg_bits=("true" if has_header_flit_msg_bits else "false"),
                                header_size=header_size,
                                MSG_TYPE_SIZE=MSG_TYPE_SIZE,
                                MCAST_DEST_SIZE=MCAST_DEST_SIZE,
                                header_flit_data_size=header_flit_data_size,
                                num_flits=num_flits,
                                FLIT_PAYLOAD_SIZE=FLIT_PAYLOAD_SIZE,
                                tail_flit_data_size=max([tail_flit_data_size,1]),
                                bus_width=bus_width,
                                num_local_dests=len(local_to_master_dest_map_a),
                                local_to_master_dest_map_s=', '.join([str(x) for x in local_to_master_dest_map_a]),
                                out_port_postfix=out_port_postfix,
                                s_splitter_pop_increment=s_splitter_pop_increment))
        s_splitter_instances = ''.join(['        {}\n'.format(x) for x in a_splitter_instances])
        s_splitter_reset = ''.join(['            {}\n'.format(x) for x in a_splitter_reset])
        s_splitter_transfernb = ''.join(['                {}\n'.format(x) for x in a_splitter_transfernb])
        s_splitter_if_ladder = '                        ' + ' \n '.join(a_splitter_if_ladder)
        s_splitter_if_ladder2 = '                    ' + ' else '.join(a_splitter_if_ladder2)

        s = """\
    #pragma hls_ungroup
    template <Connections::connections_port_t PortType = AUTO_PORT>
    class SplitterFlit_{part_name}_{joiner_splitter_id} : public sc_module {{
        SC_HAS_PROCESS(SplitterFlit_{part_name}_{joiner_splitter_id});

        public:

        typedef typename interconnect_config_icg::msgs::FLITMSG_{joiner_splitter_id} MSG_TYPE;

        sc_in_clk clk;
        sc_in<bool> rst;

        Connections::InBuffered<interconnect::Msg<MSG_TYPE>, 1, PortType> in_port;

{s_splitter_instances}

        SplitterFlit_{part_name}_{joiner_splitter_id}() : sc_module(sc_module_name(sc_gen_unique_name("splitter_{part_name}"))) {{
            SC_CTHREAD(run, clk.pos());
            async_reset_signal_is(rst, false);
        }}

        void run() {{
            in_port.Reset();

{s_splitter_reset}

            int current_msg_type = -1; // -1 is reserved for no message
            unsigned int current_flit_iter = 0;
            NVUINTW({max_msg_width}) full_msg_bits;
            NVUINTW({MSG_TYPE_SIZE}) msg_type_bits;
            NVUINTW({MCAST_DEST_SIZE}) mcast_dest_bits;

            #pragma hls_pipeline_init_interval 1
            #pragma pipeline_stall_mode flush
            while(1) {{
                wait();
                in_port.TransferNB();

                // New message to push
                interconnect::Msg<MSG_TYPE> flit; // FIXME: We need to overload Msg<> for a supermsg type instead
                if (!in_port.Empty()) {{ 
                    // Pop new message
                    flit = in_port.Peek();
                    //flit = in_port.Pop();

    
                    // Header decode ladder here
                    if(current_msg_type == -1) {{
#ifndef __SYNTHESIS__                   
                        NVHLS_ASSERT(flit.flit_id.isHeader());
                        NVHLS_ASSERT(flit.data[{MCAST_DEST_SIZE}] == 1);
#endif
                        msg_type_bits = nvhls::get_slc<{MSG_TYPE_SIZE}>(flit.data, {MCAST_DEST_SIZE}+1);
                        mcast_dest_bits = nvhls::get_slc<{MCAST_DEST_SIZE}>(flit.data, 0);

                        current_msg_type = msg_type_bits;
                        current_flit_iter = 0;
// {s_splitter_if_ladder}
                    }}

                    //cout << sc_time_stamp() << " " << name() << " HERE Popped msg_type=" << current_msg_type << " Flit<" << hex << flit.flit_id << "," << flit.data.to_uint64() << ">" << dec << endl;

                    // Reverse mapping here
{s_splitter_if_ladder2}
                    if(current_msg_type != -1) {{
                        current_flit_iter++;
                    }}
                }}
{s_splitter_transfernb}
            }}
        }}
    }};
""".format(part_name=part_name,
           joiner_splitter_id=joiner_splitter_id,
           s_splitter_instances=s_splitter_instances,
           s_splitter_reset=s_splitter_reset,
           s_splitter_transfernb=s_splitter_transfernb,
           s_splitter_if_ladder=s_splitter_if_ladder,
           s_splitter_if_ladder2=s_splitter_if_ladder2,
           max_msg_width=max(msg_type_widths.values()),
           port_id=port_id,
           is_single=("true" if is_single else "false"),
           has_header_flit_msg_bits=("true" if has_header_flit_msg_bits else "false"),
           header_size=header_size,
           MSG_TYPE_SIZE=MSG_TYPE_SIZE,
           MCAST_DEST_SIZE=MCAST_DEST_SIZE,
           header_flit_data_size=header_flit_data_size,
           num_flits=num_flits,
           FLIT_PAYLOAD_SIZE=FLIT_PAYLOAD_SIZE,
           tail_flit_data_size=max([tail_flit_data_size,1]),
           bus_width=bus_width)

        return (s, max_packet_size)

# ------------------------------- Crossbar --------------------------------------

class SplitterJoiner:
    def create_supermsg(self, YAML_DATA, topo_groups, supermsg_dest_maps, supermsg_src_maps, joiner_splitter_id):
        # Find all widths.
        msg_type_widths = dict()
        for msg_info in YAML_DATA['groups'].values():
            if 'name' in msg_info and '_width' in msg_info:
                msg_type_widths[msg_info['name']] = msg_info['_width']
        

        supermsg_master_dest_map = sorted(list( set( sum(supermsg_dest_maps.values(), list()) ) ))
        supermsg_master_insts = list()
        supermsg_dest_map_to_inst = list()
        for (part_name, inst_id, port_id) in supermsg_master_dest_map:
            if inst_id not in supermsg_master_insts:
                supermsg_master_insts.append(inst_id)
            supermsg_dest_map_to_inst.append(supermsg_master_insts.index(inst_id))

        max_msg_width = list()
        for msg_name in topo_groups:
            max_msg_width.append(int(msg_type_widths[msg_name]))

        s_declarations = str()
        s_declarations += "    class SuperMsg_{joiner_splitter_id};\n".format(joiner_splitter_id=joiner_splitter_id)

        a_supermsgs = list()
        a_supermsgs.append("IC_ADD_MESSAGE_TYPE(SUPERMSG_{joiner_splitter_id}, SuperMsg_{joiner_splitter_id})".format(joiner_splitter_id=joiner_splitter_id))

        a_icg_msg_types = list()
        a_icg_msg_types.append( ("SUPERMSG_{}".format(str(joiner_splitter_id)), "SuperMsg_{}".format(str(joiner_splitter_id))) )

        s_supermsg_functions = str()

        s_supermsg_functions += """\
            sc_lv<{num_master_insts}> get_dest_insts() {{
                sc_lv<{num_master_insts}> v;
                static const unsigned int m[{num_dests}] = {{ {supermsg_dest_map_to_inst_s} }};

                #pragma hls_unroll yes
                for(unsigned int i=0; i < {num_master_insts}; ++i) {{
                    v[i] = 0;
                }}
                
                #pragma hls_unroll yes
                for(unsigned int i=0; i < {num_dests}; ++i) {{
                    v[m[i]] |= dest_bits[i];
                }}            
                return v;    
            }}\n""".format(num_master_insts=len(supermsg_master_insts),
                           num_dests=len(supermsg_master_dest_map),
                           supermsg_dest_map_to_inst_s=', '.join([str(x) for x in supermsg_dest_map_to_inst]))

        for seq_id in range(len(topo_groups)):
            msg_name = topo_groups[seq_id]

            local_to_master_dest_map_a = list()
            for v in supermsg_dest_maps[msg_name]:
                local_to_master_dest_map_a.append(supermsg_master_dest_map.index(v))

            s_supermsg_functions += """\
            void set_dest_map_per_msg(const interconnect::Msg<interconnect_config::msgs::{msg_name}> &msg_) {{
                static const unsigned int m[{num_local_dests}] = {{ {local_to_master_dest_map_s} }};
                #pragma hls_unroll yes
                for(unsigned int i=0; i < {num_local_dests}; ++i) {{
                    dest_bits[m[i]] = msg_.dest_bits[i];
                }}
                // dest_bits = msg_.dest_bits; // FIXME: Need to isolate range here... if ladder?
            }}
            void reverse_set_dest_map_per_msg(interconnect::Msg<interconnect_config::msgs::{msg_name}> &msg_) {{
                static const unsigned int m[{num_local_dests}] = {{ {local_to_master_dest_map_s} }};
                #pragma hls_unroll yes
                for(unsigned int i=0; i < {num_local_dests}; ++i) {{
                    msg_.dest_bits[i] = dest_bits[m[i]];
                }}
                // msg_.dest_bits = dest_bits; // FIXME: Need to isolate range here... if ladder?
            }}
            void set_msg_type_per_msg(const interconnect::Msg<interconnect_config::msgs::{msg_name}> &msg_) {{
                msg_type_bits = {seq_id};
            }}
            void reverse_set_msg_type_per_msg(const interconnect::Msg<interconnect_config::msgs::{msg_name}> &msg_) {{
#ifndef __SYNTHESIS__
                NVHLS_ASSERT(msg_type_bits == {seq_id});
#endif
            }}\n""".format(msg_name=msg_name,
                           msg_name_lower=msg_name.lower(),
                           seq_id=seq_id,
                           num_local_dests=len(local_to_master_dest_map_a),
                           local_to_master_dest_map_s=', '.join([str(x) for x in local_to_master_dest_map_a]))

        s_definitions = """\
    template <>
    class Msg<interconnect_config_icg::msgs::SUPERMSG_{joiner_splitter_id}> : public UnionMsgMulticast<{max_msg_bit}, {num_dests}, {num_msgs}> {{
        public:

{s_supermsg_functions}

            // Stub that needs to live at the leaf class
            template <typename Message>
            void set_msg(const interconnect::Msg<Message> &msg_) {{
                __set_msg(msg_);
                set_dest_map_per_msg(msg_);
                set_msg_type_per_msg(msg_);
            }}
            // Stub that needs to live at the leaf class
            template <typename Message>
            void get_msg(interconnect::Msg<Message> &msg_) {{
                __get_msg(msg_);
                reverse_set_dest_map_per_msg(msg_);
                reverse_set_msg_type_per_msg(msg_);
            }}
    }};
        """.format(max_msg_bit=max(max_msg_width),
                num_dests=len(supermsg_master_dest_map), 
                num_msgs=len(topo_groups),
                joiner_splitter_id=joiner_splitter_id,
                s_supermsg_functions=s_supermsg_functions)

        return (s_declarations, s_definitions, a_icg_msg_types, max(max_msg_width))



    def create_joiner(self, part_name, master_msg_list, local_msg_list, joiner_splitter_id, grout=False):
        a_joiner_instances = list()
        a_joiner_reset = list()
        a_joiner_transfernb = list()
        a_joiner_if_ladder = list()
        for seq_id in range(len(master_msg_list)):
            (msg_name, port_id) = master_msg_list[seq_id]
            if msg_name not in local_msg_list:
                continue
            if grout:
                a_joiner_instances.append("Connections::InBuffered<interconnect::Msg<interconnect_config::msgs::{msg_name}>,2> in_port_{msg_name_lower}_{port_id};".format(msg_name=msg_name,
                                                                                                                                                    msg_name_lower=msg_name.lower(),
                                                                                                                                                    port_id=port_id))
                in_port_postfix=""
            else:
                a_joiner_instances.append("Connections::CombinationalBufferedPorts<interconnect::Msg<interconnect_config::msgs::{msg_name}>,2,0> in_port_{msg_name_lower}_{port_id};".format(msg_name=msg_name,
                                                                                                                                                    msg_name_lower=msg_name.lower(),
                                                                                                                                                    port_id=port_id))
                in_port_postfix="Read"

            a_joiner_reset.append("in_port_{msg_name_lower}_{port_id}.Reset{in_port_postfix}();".format(msg_name=msg_name,
                                                                                           msg_name_lower=msg_name.lower(),
                                                                                            port_id=port_id,
                                                                                            in_port_postfix=in_port_postfix))
            
            a_joiner_transfernb.append("in_port_{msg_name_lower}_{port_id}.TransferNB{in_port_postfix}();".format(msg_name=msg_name,
                                                                                                      msg_name_lower=msg_name.lower(),
                                                                                                      port_id=port_id,
                                                                                                      in_port_postfix=in_port_postfix))

            # Need to decode port_id here (add as a function to SuperMsg, static const unsigned int lookup table? except,
            # we don't know the local dest_id here.. so we would need to decode that as well. Unless we add port id as both a separate
            # dest AND a separate message type? but then we cannot multiplex... so we do need to figure out a mechanism to decode it.
            #
            # Could we mask or add a bridge type?
            #
            # Probably cleanest to have a part_id sc_in (or jtag) that all partitions receive, and that will
            # tell splitter where to decode for the port_id. 
            #
            # Through a function in supermsg that given a part_id AND port_id (is_port_id(part_id, port_id)) it will return true or false.
            a_joiner_if_ladder.append("""\
if (! in_port_{msg_name_lower}_{port_id}.Empty{in_port_postfix}()) {{
                        IC_MESSAGE({msg_name}) m = in_port_{msg_name_lower}_0.Pop();
                        im.set_msg(m);
                        did_pop = true;
                        // cout << sc_time_stamp() << " " << name() << " HERE Joiner Popped {msg_name}!" << endl;
                    }}""".format(seq_id=seq_id,
                                 msg_name=msg_name,
                                 msg_name_lower=msg_name.lower(),
                                 port_id=port_id,
                                 in_port_postfix=in_port_postfix))

        s_joiner_instances = ''.join(['        {}\n'.format(x) for x in a_joiner_instances])
        s_joiner_reset = ''.join(['            {}\n'.format(x) for x in a_joiner_reset])
        s_joiner_transfernb = ''.join(['                {}\n'.format(x) for x in a_joiner_transfernb])
        s_joiner_if_ladder = '                ' + ' else '.join(a_joiner_if_ladder)
        s = """\
    #pragma hls_ungroup
    template <Connections::connections_port_t PortType = AUTO_PORT>
    class Joiner_{part_name}_{joiner_splitter_id}  : public sc_module {{
        SC_HAS_PROCESS(Joiner_{part_name}_{joiner_splitter_id});

        public:

        typedef typename interconnect_config_icg::msgs::SUPERMSG_{joiner_splitter_id} MSG_TYPE;

        sc_in_clk clk;
        sc_in<bool> rst;

        Connections::Out<interconnect::Msg<MSG_TYPE>, PortType> out_port;

{s_joiner_instances}

        Joiner_{part_name}_{joiner_splitter_id}() : sc_module(sc_module_name(sc_gen_unique_name("joiner_{part_name}"))) {{
            SC_CTHREAD(run, clk.pos());
            async_reset_signal_is(rst, false);
        }}

        void run() {{
            // out_port.ResetWrite();
            out_port.Reset();

{s_joiner_reset}

            // New message to push
            interconnect::Msg<MSG_TYPE> im;
            bool did_pop = false;
            #pragma hls_pipeline_init_interval 1
            #pragma pipeline_stall_mode flush
            while(1) {{
                wait();


{s_joiner_transfernb}

                if(!did_pop) {{
{s_joiner_if_ladder}
                }}

                // Push new message
                if(did_pop) {{
                    // For loop to handle multicast -> unicast (through brute force multicasting)
                    // FIXME: Only do this if sending to a unicast-only interconnect type
                    #pragma hls_unroll yes
                    for(int i = 0; i < interconnect::Msg<MSG_TYPE>::num_dests_; ++i) {{
                        // FIXME: need use get_dest_insts() instead of dest_bits and a way to clear dest insts to handle mulitple port_ids
                        //        since we don't need to send multiple packets if we are sending to multiple port_ids of the same inst.
                        if(im.dest_bits[i] == 1) {{
                            // Unflag this one destination, since we are sending it
                            im.dest_bits[i] = 0;

                            // create a copy of the message, and only set this one destination
                            interconnect::Msg<MSG_TYPE> im_copy = im;
                            im_copy.dest_bits = 0;
                            im_copy.dest_bits[i] = 1;

                            // Push packet
                            // cout << sc_time_stamp() << " " << name() << " HERE Joiner trying to push for each destination... i=" << i << endl;
                            out_port.Push(im_copy);
                            // cout << sc_time_stamp() << " " << name() << " HERE Joiner Pushed!" << endl;

                            // Only one push per cycle
                            break;
                        }}
                    }}

                    // End condition
                    did_pop = !(im.dest_bits == 0);
                }}
            }}
        }}
    }};""".format(part_name=part_name,
           joiner_splitter_id=joiner_splitter_id,
           s_joiner_instances=s_joiner_instances,
           s_joiner_reset=s_joiner_reset,
           s_joiner_transfernb=s_joiner_transfernb,
           s_joiner_if_ladder=s_joiner_if_ladder)
    
        return s


    def create_splitter(self, part_name, master_msg_list, local_msg_list, joiner_splitter_id, grout=False):
        a_splitter_instances = list()
        a_splitter_reset = list()
        a_splitter_transfernb = list()
        a_splitter_if_ladder = list()
        for seq_id in range(len(master_msg_list)):
            (msg_name, port_id) = master_msg_list[seq_id]
            if msg_name not in local_msg_list:
                continue
            if grout:
                a_splitter_instances.append("Connections::OutBuffered<interconnect::Msg<interconnect_config::msgs::{msg_name}>, 2> out_port_{msg_name_lower}_{port_id};".format(msg_name=msg_name,
                                                                                                                                                    msg_name_lower=msg_name.lower(),
                                                                                                                                                    port_id=port_id))
                out_port_postfix = ""
            else:
                a_splitter_instances.append("Connections::CombinationalBufferedPorts<interconnect::Msg<interconnect_config::msgs::{msg_name}> , 0, 2> out_port_{msg_name_lower}_{port_id};".format(msg_name=msg_name,
                                                                                                                                                    msg_name_lower=msg_name.lower(),
                                                                                                                                                    port_id=port_id))
                out_port_postfix = "Write"

            a_splitter_reset.append("out_port_{msg_name_lower}_{port_id}.Reset{out_port_postfix}();".format(msg_name=msg_name,
                                                                                msg_name_lower=msg_name.lower(),
                                                                                port_id=port_id,
                                                                                out_port_postfix=out_port_postfix))
            a_splitter_transfernb.append("out_port_{msg_name_lower}_{port_id}.TransferNB{out_port_postfix}();".format(msg_name=msg_name,
                                                                                                      msg_name_lower=msg_name.lower(),
                                                                                                      port_id=port_id,
                                                                                                      out_port_postfix=out_port_postfix))

            # Need to decode port_id here (add as a function to SuperMsg, static const unsigned int lookup table? except,
            # we don't know the local dest_id here.. so we would need to decode that as well. Unless we add port id as both a separate
            # dest AND a separate message type? but then we cannot multiplex... so we do need to figure out a mechanism to decode it.
            #
            # Could we mask or add a bridge type?
            #
            # Probably cleanest to have a part_id sc_in (or jtag) that all partitions receive, and that will
            # tell splitter where to decode for the port_id.
            #
            # Through a function in supermsg that given a part_id AND port_id (is_port_id(part_id, port_id)) it will return true or false.
            a_splitter_if_ladder.append("""\
if ((im.msg_type_bits == {seq_id}) && (!out_port_{msg_name_lower}_{port_id}.Full{out_port_postfix}())) {{
                    IC_MESSAGE({msg_name}) m;
                    im.get_msg(m);
                    out_port_{msg_name_lower}_{port_id}.Push(m);
                    in_port.Pop();
                    // cout << sc_time_stamp() << " " << name() << " HERE Splitter Pushed {msg_name}!" << endl;
}}
if ((im.msg_type_bits == {seq_id}) && (out_port_{msg_name_lower}_{port_id}.Full{out_port_postfix}())) {{
                    // cout << sc_time_stamp() << " " << name() << " HERE Splitter Trying Push {msg_name}!" << endl;

                }}""".format(seq_id=seq_id,
                             msg_name=msg_name,
                             msg_name_lower=msg_name.lower(),
                             port_id=port_id,
                             out_port_postfix=out_port_postfix))
        s_splitter_instances = ''.join(['        {}\n'.format(x) for x in a_splitter_instances])
        s_splitter_reset = ''.join(['            {}\n'.format(x) for x in a_splitter_reset])
        s_splitter_transfernb = ''.join(['            {}\n'.format(x) for x in a_splitter_transfernb])
        s_splitter_if_ladder = '                ' + ' \n '.join(a_splitter_if_ladder)

        s = """\
    #pragma hls_ungroup
    template <Connections::connections_port_t PortType = AUTO_PORT>
    class Splitter_{part_name}_{joiner_splitter_id} : public sc_module {{
        SC_HAS_PROCESS(Splitter_{part_name}_{joiner_splitter_id});

        public:

        typedef typename interconnect_config_icg::msgs::SUPERMSG_{joiner_splitter_id} MSG_TYPE;

        sc_in_clk clk;
        sc_in<bool> rst;

        Connections::InBuffered<interconnect::Msg<MSG_TYPE>, 1, PortType> in_port;

{s_splitter_instances}

        Splitter_{part_name}_{joiner_splitter_id}() : sc_module(sc_module_name(sc_gen_unique_name("splitter_{part_name}"))) {{
            SC_CTHREAD(run, clk.pos());
            async_reset_signal_is(rst, false);
        }}

        void run() {{
            in_port.Reset();

{s_splitter_reset}

            #pragma hls_pipeline_init_interval 1
            #pragma pipeline_stall_mode flush
            while(1) {{
                wait();
                in_port.TransferNB();

                // New message to push
                interconnect::Msg<MSG_TYPE> im; // FIXME: We need to overload Msg<> for a supermsg type instead
                if (!in_port.Empty()) {{ 
                // Pop new message
                im = in_port.Peek();
                // cout << sc_time_stamp() << " " << name() << " HERE Splitter popped!" << endl;

                // Reverse mapping here
{s_splitter_if_ladder}
                }}
{s_splitter_transfernb}
            }}
        }}
    }};
""".format(part_name=part_name,
           joiner_splitter_id=joiner_splitter_id,
           s_splitter_instances=s_splitter_instances,
           s_splitter_reset=s_splitter_reset,
           s_splitter_transfernb=s_splitter_transfernb,
           s_splitter_if_ladder=s_splitter_if_ladder)

        return s




class CrossbarGenerator:
    def __init__(self, YAML_DATA):
        pass

    def generate_msg_type(self, msg_name, dest_map):
        dest_map_entries = (12+5)*' '
        assert_check_a = list()
        for idx in range(len(dest_map)):
            (part_name, dest_map_id, port_id) = dest_map[idx]
            dest_map_entries += "if(dest.part_id == {dest_map_id} && dest.port_id == {port_id}) {{ idx = {idx}; }}\n            else ".format(dest_map_id=dest_map_id, port_id=port_id, idx=idx)
            assert_check_a.append("(dest.part_id == {dest_map_id} && dest.port_id == {port_id})".format(dest_map_id=dest_map_id, port_id=port_id))
        dest_map_entries += "{{ idx = 0; }}\n".format()
        assert_check_s = ('\n' + 24*' ' + ' || ').join(assert_check_a)

        s_dest_map = """\
        virtual inline unsigned int dest_map(const Destination &dest) {{
            // FIXME: Can we have this as a synthesizable assert?
#ifndef __SYNTHESIS__
            NVHLS_ASSERT(   {assert_check_s});
#endif // ifndef __SYNTHESIS__

            unsigned int idx;

{dest_map_entries}
            return idx;
        }}
        """.format(dest_map_entries=dest_map_entries,
                        assert_check_s=assert_check_s)

        s = """\n\
    template <>
    class Msg<interconnect_config::msgs::{msg_name}> : public LinkMsg<interconnect_config::msgs::{msg_name}, {dest_num}> {{
        public:

        static const unsigned int dest_count = {dest_num};
        typedef interconnect_config::msgs::{msg_name} MSG_TYPE;

        //// Constructor-Destructor 
        Msg() : LinkMsg<MSG_TYPE,dest_count>() {{}}

        Msg(const data_t &m) : LinkMsg<MSG_TYPE,dest_count>(m) {{}}

        template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
        explicit Msg(const InterconnectMessageSCLV<MaxDataWidth, MaxDestCount> &im) : LinkMsg<MSG_TYPE,dest_count>(im) {{}}

        Msg<MSG_TYPE>& operator=(const data_t &m) {{
            set_msg(m);
            return *this;
        }}
        Msg<MSG_TYPE>& operator<<(const Destination &dest) {{
            dest_bits[dest_map(dest)] = 1;
            return *this;
        }}
{s_dest_map}
    }};
        """.format(msg_name=msg_name,
                        dest_num=len(dest_map),
                        s_dest_map=s_dest_map)

        return s

    # This is topo-agnostic, move into a global section
    def generate_ic_interface(self, part_name, handler_instances, handler_constructors, handler_binds, handler_inits):
        handler_instances_s = str()
        for (handler_type, handler_template_args, handler_name) in handler_instances:
            handler_template_args = [str(i) for i in handler_template_args]
            handler_instances_s += 8*' ' + "{handler_type}<{handler_template_args}> {handler_name};\n".format(
                                                                                                            handler_type=handler_type,
                                                                                                            handler_template_args=', '.join(handler_template_args + ['PortType']),
                                                                                                            handler_name=handler_name)
        handler_constructors_s = str()
        for handler_command in handler_constructors:
            handler_constructors_s += 12*' ' + "{}\n".format(handler_command)

        handler_binds_s = str()
        for handler_bind in handler_binds:
            handler_binds_s += 0*' ' + "{}\n".format(handler_bind)

        handler_inits_s = str()
        for handler_init in handler_inits:
            handler_inits_s += ",\n" + 34*' ' + "{}".format(handler_init)

        s = """\
    template <Connections::connections_port_t PortType>
    class InterconnectInterface<interconnect_config::parts::{part_name}, PortType> {{
    public:
        sc_in_clk clk;
        sc_in<bool> rst;

        // Handlers
{handler_instances}


        InterconnectInterface() : clk(sc_gen_unique_name("ic_iface_{part_name_lower}_clk")),
                                  rst(sc_gen_unique_name("ic_iface_{part_name_lower}_rst")){handler_inits}
        {{

{handler_constructors}
        }}

        explicit InterconnectInterface(const char *name) : clk(sc_gen_unique_name("ic_iface_{part_name_lower}_clk")),
                                                           rst(sc_gen_unique_name("ic_iface_{part_name_lower}_rst")){handler_inits}
    {{

{handler_constructors}
        }}
{handler_binds}
    }};
    
        """.format(part_name=part_name,
                   part_name_lower=part_name.lower(),
                   handler_instances=handler_instances_s,
                   handler_constructors=handler_constructors_s,
                   handler_binds=handler_binds_s,
                   handler_inits=handler_inits_s)

        return s


class LinkGenerator:
    def __init__(self, YAML_DATA):
        pass

    def generate_msg_type(self, msg_name, dest_map):
        dest_map_entries = (12+5)*' '
        assert_check_a = list()
        for idx in range(len(dest_map)):
            (part_name, dest_map_id, port_id) = dest_map[idx]
            dest_map_entries += "if(dest.part_id == {dest_map_id} && dest.port_id == {port_id}) {{ idx = {idx}; }}\n            else ".format(dest_map_id=dest_map_id, port_id=port_id, idx=idx)
            assert_check_a.append("(dest.part_id == {dest_map_id} && dest.port_id == {port_id})".format(dest_map_id=dest_map_id, port_id=port_id))
        dest_map_entries += "{{ idx = 0; }}\n".format()
        assert_check_s = ('\n' + 24*' ' + ' || ').join(assert_check_a)

        s_dest_map = """\
        virtual inline unsigned int dest_map(const Destination &dest) {{
            // FIXME: Can we have this as a synthesizable assert?
#ifndef __SYNTHESIS__
            NVHLS_ASSERT(   {assert_check_s});
#endif // ifndef __SYNTHESIS__

            unsigned int idx;

{dest_map_entries}
            return idx;
        }}
        """.format(dest_map_entries=dest_map_entries,
                        assert_check_s=assert_check_s)

        s = """\n\
    template <>
    class Msg<interconnect_config::msgs::{msg_name}> : public LinkMsg<interconnect_config::msgs::{msg_name}, {dest_num}> {{
        static const unsigned int dest_count = {dest_num};
        typedef interconnect_config::msgs::{msg_name} MSG_TYPE;

        public:

        //// Constructor-Destructor 
        Msg() : LinkMsg<MSG_TYPE,dest_count>() {{}}

        Msg(const data_t &m) : LinkMsg<MSG_TYPE,dest_count>(m) {{}}

        template <unsigned int MaxDataWidth, unsigned int MaxDestCount>
        explicit Msg(const InterconnectMessageSCLV<MaxDataWidth, MaxDestCount> &im) : LinkMsg<MSG_TYPE,dest_count>(im) {{}}

        Msg<MSG_TYPE>& operator=(const data_t &m) {{
            set_msg(m);
            return *this;
        }}
        Msg<MSG_TYPE>& operator<<(const Destination &dest) {{
            dest_bits[dest_map(dest)] = 1;
            return *this;
        }}
{s_dest_map}
    }};
        """.format(msg_name=msg_name,
                        dest_num=len(dest_map),
                        s_dest_map=s_dest_map)

        return s


    # This is topo-agnostic, move into a global section
    def generate_ic_interface(self, part_name, handler_instances, handler_constructors, handler_binds, handler_inits):
        handler_instances_s = str()
        for (handler_type, handler_template_args, handler_name) in handler_instances:
            handler_template_args = [str(i) for i in handler_template_args]
            handler_instances_s += 8*' ' + "{handler_type}<{handler_template_args}> {handler_name};\n".format(
                                                                                                            handler_type=handler_type,
                                                                                                            handler_template_args=', '.join(handler_template_args + ['PortType']),
                                                                                                            handler_name=handler_name)
        handler_constructors_s = str()
        for handler_command in handler_constructors:
            handler_constructors_s += 12*' ' + "{}\n".format(handler_command)

        handler_binds_s = str()
        for handler_bind in handler_binds:
            handler_binds_s += 0*' ' + "{}\n".format(handler_bind)


        handler_inits_s = str()
        for handler_init in handler_inits:
            handler_inits_s += ",\n" + 34*' ' + "{}".format(handler_init)

        s = """\
    template <Connections::connections_port_t PortType>
    class InterconnectInterface<interconnect_config::parts::{part_name}, PortType> {{
    public:
        sc_in_clk clk;
        sc_in<bool> rst;

        // Handlers
{handler_instances}


        InterconnectInterface() : clk(sc_gen_unique_name("ic_iface_{part_name_lower}_clk")),
                                  rst(sc_gen_unique_name("ic_iface_{part_name_lower}_rst")){handler_inits}
        {{

{handler_constructors}
        }}

        explicit InterconnectInterface(const char *name) : clk(sc_gen_unique_name("ic_iface_{part_name_lower}_clk")),
                                                           rst(sc_gen_unique_name("ic_iface_{part_name_lower}_rst")){handler_inits}
        {{

{handler_constructors}
        }}
{handler_binds}
    }};
    
        """.format(part_name=part_name,
                   part_name_lower=part_name.lower(),
                   handler_instances=handler_instances_s,
                   handler_constructors=handler_constructors_s,
                   handler_binds=handler_binds_s,
                   handler_inits=handler_inits_s)

        return s


    def IM_src_map_gen(self):
        pass

    def IM_dest_map_gen(self):
        pass

    def partition_gen(self):
        # Add ports for link

        # Add sender for link

        # Add receiver for link

        pass

    def central_gen(self):
        # Link has no central module.
        pass

# agnostic to topology since topologies add to interconnect_instances and interconnect_binds as needed
def generate_interconnect(ic_id,
                          interconnect_instances,
                          interface_binds,
                          interconnect_constructors,
                          interconnect_constructor_inits,
                          grout_instances,
                          grout_constructors,
                          grout_constructor_inits):

    def indent_arr(indent_levels, a):
        return '\n'.join([indent_levels*' ' + x for x in a])

    def indent_init(indent_levels, a):
        return ''.join([',\n' + indent_levels*' ' + x for x in a])

    grout_instances = ["sc_in_clk clk;",
                        "sc_in<bool> rst;"] + grout_instances

    grout_constructor_inits = ["clk(sc_gen_unique_name(\"clk\"))",
                                "rst(sc_gen_unique_name(\"rst\"))"] + grout_constructor_inits

    interconnect_constructors = ["grout_{}.clk(clk);".format(ic_id),
                                    "grout_{}.rst(rst);".format(ic_id)] + interconnect_constructors

    interface_binds_s = str()
    for part_name in set([x[0] for x in interface_binds]):
        unique_part_ids = set(sum([list(x[1]) for x in interface_binds if x[0] == part_name], []))
        unique_part_ids_s = []
        for (range_start, range_end) in to_ranges(sorted(unique_part_ids)):
            unique_part_ids_s.append("(part_id_ >= {range_start} && part_id_ <= {range_end})".format(range_start=range_start, range_end=range_end))
        part_assert_s = "            NVHLS_ASSERT( {} );".format('\n                         || '.join(unique_part_ids_s))
        bind_body_s = '\n'.join(["          {}".format(x[2]) for x in interface_binds if x[0] == part_name])
        interface_binds_s += """\n\
        template <Connections::connections_port_t PortType>
        void InterfaceBind(interconnect::InterconnectInterface<interconnect_config::parts::{part_name}, PortType> &ic_interface, const unsigned int &part_id_) {{
    #ifndef __SYNTHESIS__
            // Only bind during __SYNTHESIS__ (actual __SYNTHESIS__), not _SYNTHESIS_ in co-sim.
            ic_interface.clk(clk);
            ic_interface.rst(rst);

            // FIXME: can we do an OVL assert for this?
{part_assert_s} 
    #endif
{bind_body_s}
        }}\n""".format(part_name=part_name, part_assert_s=part_assert_s, bind_body_s=bind_body_s)


    s = """\
}}; // namespace interconnect

class Grout_{ic_id} : public sc_module {{
    public:

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Ports and Channels

{grout_instances}

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructors

    explicit Grout_{ic_id}(sc_module_name name)
        : sc_module(name){grout_constructor_inits}
    {{
{grout_constructors}
    }}
}};


#endif

#if !defined(IC_GEN_TOP_ONLY) && !defined(__INTERCONNECT_GEN_BOT_{ic_id}_HPP__)
#define __INTERCONNECT_GEN_BOT_{ic_id}_HPP__

#define NVHLS_VERIFY_BLOCKS (Grout_{ic_id})
#include <nvhls_verify.h>
#undef IC_GEN_TOP_ONLY

namespace interconnect {{
    #pragma hls_ungroup
    template <>
    class Interconnect<{ic_id}> {{
        public:

        ////////////////////////////////////////////////////////////////////////////////////////////////
        // Ports and Channels

        sc_in_clk clk;
        sc_in<bool> rst;

        NVHLS_DESIGN(Grout_{ic_id}) grout_{ic_id};

{interconnect_instances}

        ////////////////////////////////////////////////////////////////////////////////////////////////
        // Constructors

        void SharedConstructor() {{
{interconnect_constructors}
        }}

        Interconnect()
            : clk(sc_gen_unique_name("clk")),
              rst(sc_gen_unique_name("rst")),
              grout_{ic_id}("grout_{ic_id}"){interconnect_constructor_inits}
        {{
            SharedConstructor();
        }}

        explicit Interconnect(const char *name)
            : clk(sc_gen_unique_name("clk")),
              rst(sc_gen_unique_name("rst")),
              grout_{ic_id}("grout_{ic_id}"){interconnect_constructor_inits}
        {{
            SharedConstructor();
        }}


        ////////////////////////////////////////////////////////////////////////////////////////////////
        // Interface bindings
{interface_binds_s}
        ////////////////////////////////////////////////////////////////////////////////////////////////
        // Unsupported functions in INTERCONNECT_GEN
        
        template <typename T>
        void Bind(Connections::OutBlocking<T> &p) {{
            NVHLS_ASSERT(0);
        }}
        template <typename T>
        void Bind(Connections::InBlocking<T> &p) {{
            NVHLS_ASSERT(0);
        }}
        template <typename T>
        void add_to_dest_directory(Connections::InBlocking<T> &p, unsigned int part_id, unsigned int msg_id, unsigned int port_id=0, bool allow_duplicate=false) {{
            NVHLS_ASSERT(0);
        }}
        template <typename MSG_TYPE>
        void add_to_dest_directory(Connections::InBlocking<interconnect::Msg<MSG_TYPE> > &p, unsigned int part_id, unsigned int port_id=0, bool allow_duplicate=false) {{
            NVHLS_ASSERT(0);
        }}
        
#ifndef INTERCONNECT_GEN
        void disable_pair_of_ports(Connections::Blocking_abs &p1, Connections::Blocking_abs &p2) {{
            NVHLS_ASSERT(0);
        }}
#endif

        void pretty_print() {{}}
        void print_statistics() {{}}
    }};

#endif

#if !defined(__INTERCONNECT_GEN_TOP_{ic_id}_HPP__)
#define __INTERCONNECT_GEN_TOP_{ic_id}_HPP__
    
    """.format(ic_id=ic_id,
                  interconnect_instances = indent_arr(8, interconnect_instances),
                  interface_binds_s=interface_binds_s,
                  interconnect_constructors = indent_arr(12, interconnect_constructors),
                  interconnect_constructor_inits = indent_init(14, interconnect_constructor_inits),
                  grout_instances = indent_arr(4, grout_instances),
                  grout_constructors = indent_arr(8, grout_constructors),
                  grout_constructor_inits = indent_init(10, grout_constructor_inits))

    return s


def do_generate(YAML_DATA, IC_DATA, f, rpt, NOC_IS_NOP=False):
    # For each group:
    # - Create three sets of code:
    #    1. Partition-level module
    #    2. Central module
    #    3. Top-level code block
    #
    # For a link, this looks something like:
    # - Partition-level module: Sender and Receiver
    # - Central module: N/A
    # - Top-level code block: interconnections between modules only
    #
     # For a link, this looks something like:
    # - Partition-level module: Sender and Receiver
    # - Central module: N/A
    # - Top-level code block: interconnections between modules only
    #

    # Strings to accumulate generate results.
    part_types = list()
    handler_instances = dict()
    handler_constructors = dict()
    handler_binds = dict()
    handler_inits = dict()
    msg_types_s = str()
    ic_types = list()

    interface_binds = dict()

    grout_constructors = dict()
    grout_constructors_pre = dict()
    grout_constructors_post = dict()
    grout_constructor_inits = dict()
    grout_instances = dict()

    interconnect_constructors = dict()
    interconnect_constructors_pre = dict()
    interconnect_constructors_post = dict()
    interconnect_constructor_inits = dict()
    interconnect_instances = dict()

    supermsg_idx = 0
    super_msg_decls_s = str()
    super_msg_types_s = str()
    icg_config_msg_types_a = list()
    super_msg_classes_s = str()

    global_wire_manifest = list()

    ################################################################################################################
    # These would be more globals, for a per-partition setting.

    def add_part_type(part_type):
        if part_type not in part_types:
            part_types.append(part_type)
        handler_instances.setdefault(part_type, list())
        handler_constructors.setdefault(part_type, list())
        handler_binds.setdefault(part_type, list())
        handler_inits.setdefault(part_type, list())

    def add_ic_type(ic_id):
        if ic_id not in ic_types:
            ic_types.append(ic_id)

    # v--------------------- Move to Link class --------------------v

    def link_add_receiver(ic_type, part_type, msg_type, port_ids, num_srcs, num_dests, inst_name_base, grout=False):
        add_ic_type(ic_type)
        add_part_type(part_type)

        if num_srcs > 0:
            if grout:
                b_s = 12*' '
                for port_id in port_ids:
                    inst_name = inst_name_base + "_" + str(port_id)
                    handler_instances[part_type].append( ("Connections::In", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_type)], inst_name) )
                    b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                b_s += "{{}}".format()
            else:
                b_s = 12*' '
                for port_id in port_ids:
                    inst_name = inst_name_base + "_" + str(port_id)
                    handler_instances[part_type].append( ("LinkReceiver", ["interconnect_config::msgs::{}".format(msg_type), num_srcs], inst_name) )
                    handler_constructors[part_type] += ["{}.clk(clk);".format(inst_name), "{}.rst(rst);".format(inst_name)]
                    b_s += "if(port_id == {port_id}) {{ {inst_name}.Bind(p); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                b_s += "{{}}".format()
        else:
            # if no senders, just connect up to a Combinational to prevent unbound errors
            b_s = 12*' '
            for port_id in port_ids:
                inst_name = inst_name_base + "_" + str(port_id)
                handler_instances[part_type].append( ("Connections::Combinational", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_type)], inst_name) )
                handler_instances[part_type].append( ("Connections::DummySource", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_type)], inst_name + "_src") )
                handler_inits[part_type].append( "{inst_name}(\"{inst_name}\")".format(inst_name=inst_name + "_src") )
                handler_constructors[part_type] += ["{inst_name}_src.clk(clk); {inst_name}_src.rst(rst); {inst_name}_src.out({inst_name});".format(inst_name=inst_name) ]
                b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
            b_s += "{{}}".format()           

        handler_binds[part_type].append("""\
        template <unsigned int BIND_ID>
        void Bind(Connections::InBlocking<interconnect::Msg<interconnect_config::msgs::{msg_type}> > &p, unsigned int port_id) {{
{b_s}
        }}""".format(msg_type=msg_type, inst_name=inst_name, b_s=b_s))


    def link_add_sender(ic_type, part_type, msg_type, port_ids, num_srcs, num_dests, inst_name_base, grout=False):
        add_ic_type(ic_type)
        add_part_type(part_type)

        if num_dests > 0:
            if grout:
                b_s = 12*' '
                for port_id in port_ids:
                    inst_name = inst_name_base + "_" + str(port_id)
                    handler_instances[part_type].append( ("Connections::Out", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_type)], inst_name) )
                    b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                b_s += "{{}}".format()
            else:
                b_s = 12*' '
                for port_id in port_ids:
                    inst_name = inst_name_base + "_" + str(port_id)
                    handler_instances[part_type].append( ("LinkSender", ["interconnect_config::msgs::{}".format(msg_type), num_dests], inst_name) )
                    handler_constructors[part_type] += ["{}.clk(clk);".format(inst_name), "{}.rst(rst);".format(inst_name)]
                    b_s += "if(port_id == {port_id}) {{ {inst_name}.Bind(p); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                b_s += "{{}}".format()
        else:
            b_s = 12*' '
            for port_id in port_ids:
                inst_name = inst_name_base + "_" + str(port_id)
                handler_instances[part_type].append( ("Connections::Combinational", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_type)], inst_name) )
                handler_instances[part_type].append( ("Connections::DummySink", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_type)], inst_name + "_sink") )
                handler_inits[part_type].append( "{inst_name}(\"{inst_name}\")".format(inst_name=inst_name + "_sink") )
                handler_constructors[part_type] += ["{inst_name}_sink.clk(clk); {inst_name}_sink.rst(rst); {inst_name}_sink.in({inst_name});".format(inst_name=inst_name)]
                b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
            b_s += "{{}}".format()


        handler_binds[part_type].append("""\
        template <unsigned int BIND_ID>
        void Bind(Connections::OutBlocking<interconnect::Msg<interconnect_config::msgs::{msg_type}> > &p, unsigned int port_id) {{
{b_s}
        }}""".format(msg_type=msg_type, inst_name=inst_name, b_s=b_s))
    
    # ^--------------------- Move to Link class --------------------^
    # v--------------------- Move to Crossbar class --------------------v

    def crossbar_add_receiver(ic_type, part_type, msg_type, port_ids, num_srcs, num_dests, inst_name_base):
        add_ic_type(ic_type)
        add_part_type(part_type)

        b_s = 12*' '
        for port_id in port_ids:
            inst_name = inst_name_base + "_" + str(port_id)
            handler_instances[part_type].append( ("Connections::In", ["IC_MESSAGE({})".format(msg_type)], inst_name) )
            b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
        b_s += "{{}}".format()

        handler_binds[part_type].append("""\
        template <unsigned int BIND_ID>
        void Bind(Connections::InBlocking<interconnect::Msg<interconnect_config::msgs::{msg_type}> > &p, unsigned int port_id) {{
{b_s}
        }}""".format(msg_type=msg_type, inst_name=inst_name, b_s=b_s))

    def crossbar_add_sender(ic_type, part_type, msg_type, port_ids, num_srcs, num_dests, inst_name_base):
        add_ic_type(ic_type)
        add_part_type(part_type)

        b_s = 12*' '
        for port_id in port_ids:
            inst_name = inst_name_base + "_" + str(port_id)
            handler_instances[part_type].append( ("Connections::Out", ["IC_MESSAGE({})".format(msg_type)], inst_name) )
            b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
        b_s += "{{}}".format()

        handler_binds[part_type].append("""\
        template <unsigned int BIND_ID>
        void Bind(Connections::OutBlocking<interconnect::Msg<interconnect_config::msgs::{msg_type}> > &p, unsigned int port_id) {{
{b_s}
        }}""".format(msg_type=msg_type, inst_name=inst_name, b_s=b_s))


    # ^--------------------- Move to Crossbar class --------------------^
    # v--------------------- Global  --------------------v

    def gen_ic_iface_bind(ic_type):
        s = """\
    // Enable top-down (versus bottom-up) binding.
    //   template <unsigned int BIND_ID, unsigned int part_id, Connections::connections_port_t ICPortType, Connections::connections_port_t IfacePortType>
    //   void do_interconnect_to_interface_bind(Interconnect<{ic_type}, ICPortType> &ic,
    // FIXME: Need to add PortType parameter to IC soon --- ^
    template <unsigned int BIND_ID, typename PART_TYPE, Connections::connections_port_t IfacePortType>
    void do_interconnect_to_interface_bind(Interconnect<{ic_type}> &ic,
                                           interconnect::InterconnectInterface<PART_TYPE, IfacePortType> &ic_interface,
                                           const sc_object &parent,
                                           const unsigned int &part_id_) {{
        ic.InterfaceBind(ic_interface, part_id_);
    }}
        """.format(ic_type=ic_type)
        return s


    # For a named message type, create a list of partitions (ports) that has a dest port.
    # 
    def gen_msg_dest_map(IC_DATA, msg_name):
        dest_map = list()
        for (part_name, part) in IC_DATA['parts'].items():
            if msg_name in part['dest_ports']:
                for inst_id in part['insts']:
                    for port_id in part['dest_ports'][msg_name]:
                        dest_map.append( (part_name, inst_id, port_id) )
        return dest_map


    # For a named message type, create a list of partitions (ports) that has a src port.
    # 
    def gen_msg_src_map(IC_DATA, msg_name):
        src_map = list()
        for (part_name, part) in IC_DATA['parts'].items():
            if msg_name in part['src_ports']:
                for inst_id in part['insts']:
                    for port_id in part['src_ports'][msg_name]:
                        src_map.append( (part_name, inst_id, port_id) )
        return src_map

    def is_compatible_channels(IC_DATA, msg_name, src_part, dest_part):
        # Do a lookup in channels, ensure that it exists.
        assert "channels" in IC_DATA["channels"]
        for channel in IC_DATA["channels"].values():
            if channel["msg_name"] != msg_name:
                continue
            src_name = channel["src_name"]
            dest_name = channel["dest_name"]

            src_match = re.match(r"$" + re.escape(src_part) + r"\..*", src_name)
            dest_match = re.match(r"$" + re.escape(dest_part) + r"\..*", dest_name)

            if src_match and dest_match:
                return True

    def map_unique_parts(IC_DATA, src_or_dest_map):
        return set([x[0] for x in src_or_dest_map])

    def map_unique_parts_ports(IC_DATA, src_or_dest_map):
        s = dict()
        for part_name in map_unique_parts(IC_DATA, src_or_dest_map):
            s[part_name] = set()
            for x in src_or_dest_map:
                if x[0] == part_name:
                    s[part_name].add(x[2])
        return s

    # ^--------------------- Global  --------------------^

    ################################################################################################################

    # Find all msg types that aren't in a group
    not_connected_msg_types = set(IC_DATA['msgs'].keys())
    for topo in YAML_DATA['topologies'].values():
        for msg_name in topo['groups']:
            if msg_name in not_connected_msg_types:
                not_connected_msg_types.remove(msg_name)

            
    # Generate for stub sub-type. Added as a special link subgroup called NC
    YAML_DATA['topologies']['__internal_NC'] = dict()
    YAML_DATA['topologies']['__internal_NC']['type'] = 'link'
    YAML_DATA['topologies']['__internal_NC']['groups'] = not_connected_msg_types
    YAML_DATA['topologies']['__internal_NC']['options'] = dict()

    # Inst ids per partition type (global to topologies)
    # Could probably also pull this from IC_DATA directly?
    dest_map_per_iface = dict()
    src_map_per_iface = dict()
    interface_dest_binds_per_iface = dict()
    interface_src_binds_per_iface = dict()

    def add_collated_combinationals(base_name, a_combinationals, postfix="", msg_config_loc="interconnect_config", override_ic_msg_str = None):
        if postfix:
            postfix = "_" + postfix
        for msg_name in a_combinationals.keys():
            assert a_combinationals[msg_name] > 0

            if not override_ic_msg_str:
                ic_msg_str = "interconnect::Msg<{}>".format(msg_config_loc + "::msgs::" + msg_name)
            else:
                ic_msg_str = override_ic_msg_str

            ic_collate_select(ic_id, interconnect_instances)
            ic_collate_add("Connections::Combinational<{ic_msg_str} > {base_name}_{msg_name}{postfix}[{num}];"
                                        .format(base_name=base_name,
                                                msg_name=msg_name,
                                                postfix=postfix,
                                                ic_id=ic_id,
                                                ic_msg_str=ic_msg_str,
                                                num = a_combinationals[msg_name]))

            ic_collate_select(ic_id, interconnect_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=a_combinationals[msg_name]) )
            ic_collate_add("grout_{ic_id}.{base_name}_{msg_name}{postfix}[i]({base_name}_{msg_name}{postfix}[i]);"
                                                            .format(base_name=base_name,
                                                                    msg_name=msg_name,
                                                                    postfix=postfix,
                                                                    ic_id=ic_id))

    def add_collated_retimings(base_name, a_retiming_stages, msg_config_loc="interconnect_config", override_ic_msg_str = None):
        for (msg_name, retiming_stages, extra_capacity) in a_retiming_stages.keys():
            a_retiming_stage = a_retiming_stages[(msg_name, retiming_stages, extra_capacity)]
            assert len(a_retiming_stage) > 0

            if not override_ic_msg_str:
                ic_msg_str = "interconnect::Msg<{}>".format(msg_config_loc + "::msgs::" + msg_name)
            else:
                ic_msg_str = override_ic_msg_str
            
            ic_collate_select(ic_id, grout_instances)
            ic_collate_add("interconnect::RetimingStages<{ic_msg_str}, {retiming_stages}, {extra_capacity}> *{base_name}_{msg_name}_{retiming_stages}_{extra_capacity}_retime[{num}];"
                                                .format(base_name=base_name,
                                                        ic_msg_str=ic_msg_str,
                                                        msg_name=msg_name,
                                                        retiming_stages=retiming_stages,
                                                        extra_capacity=extra_capacity,
                                                        num=len(a_retiming_stage)))

            # optimization if a_retiming_stage is already sequential.
            skip_retiming_lookup = a_retiming_stage == list(range(len(a_retiming_stage)))
            retiming_lookup_str = "{base_name}_{msg_name}_{retiming_stages}_{extra_capacity}_retime_lookup".format(base_name=base_name,
                                                                                                                    msg_name=msg_name,
                                                                                                                    retiming_stages=retiming_stages,
                                                                                                                    extra_capacity=extra_capacity)
            if not skip_retiming_lookup:
                ic_collate_select(ic_id, grout_constructors_pre)
                ic_collate_add("static const unsigned int {retiming_lookup_str}[{num}] = {{{arr_lookup}}};"
                                                                .format(retiming_lookup_str=retiming_lookup_str,
                                                                        num=len(a_retiming_stage),
                                                                        arr_lookup=', '.join([str(x) for x in a_retiming_stage])) )

            ic_collate_select(ic_id, grout_constructors_pre, "for(int i=0; i < {num}; ++i)".format(num=len(a_retiming_stage)) )
            ic_collate_add("{base_name}_{msg_name}_{retiming_stages}_{extra_capacity}_retime[i] = new interconnect::RetimingStages<{ic_msg_str}, {retiming_stages}, {extra_capacity}>(sc_gen_unique_name(\"retiming_stages\"));"
                                                            .format(base_name=base_name,
                                                                    ic_msg_str=ic_msg_str,
                                                                    msg_name=msg_name,
                                                                    retiming_stages=retiming_stages,
                                                                    extra_capacity=extra_capacity))

            ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=len(a_retiming_stage)) )

            if retiming_stages or extra_capacity:
                ic_collate_add("{base_name}_{msg_name}_{retiming_stages}_{extra_capacity}_retime[i]->clk(clk);"
                                                                .format(base_name=base_name,
                                                                        msg_name=msg_name,
                                                                        retiming_stages=retiming_stages,
                                                                        extra_capacity=extra_capacity))
                ic_collate_add("{base_name}_{msg_name}_{retiming_stages}_{extra_capacity}_retime[i]->rst(rst);"
                                                                .format(base_name=base_name,
                                                                        msg_name=msg_name,
                                                                        retiming_stages=retiming_stages,
                                                                        extra_capacity=extra_capacity))

            ic_collate_add("{base_name}_{msg_name}_{retiming_stages}_{extra_capacity}_retime[i]->in_port({base_name}_{msg_name}[{retiming_lookup_str}]);"
                                                            .format(base_name=base_name,
                                                                    msg_name=msg_name,
                                                                    retiming_stages=retiming_stages,
                                                                    extra_capacity=extra_capacity,
                                                                    retiming_lookup_str=(retiming_lookup_str + "[i]") if not skip_retiming_lookup else "i"))
            ic_collate_add("{base_name}_{msg_name}_{retiming_stages}_{extra_capacity}_retime[i]->out_port({base_name}_{msg_name}_retimed[{retiming_lookup_str}]);"
                                                            .format(base_name=base_name,
                                                                    msg_name=msg_name,
                                                                    retiming_stages=retiming_stages,
                                                                    extra_capacity=extra_capacity,
                                                                    retiming_lookup_str=(retiming_lookup_str + "[i]") if not skip_retiming_lookup else "i"))

    # Grab each interconnect topo from YAML_DATA.
    print()
    for topo_name,topo in YAML_DATA['topologies'].items():
        topo_groups = topo['groups']
        topo_type = topo['type']
        topo_options = topo['options']
        
        if len(topo_groups) == 0:
            print("Info: Skipping topo '%s' because no associated groups." % topo_name)
            continue

        inst_name_to_part_id = dict()
        for part_data in IC_DATA["parts"].values():
            for i in range(len(part_data["insts"])):
                part_id = part_data["insts"][i]
                inst_name = part_data["inst_names"][i]
                inst_name_to_part_id[inst_name] = part_id

        inst_coors = dict()
        for (inst_name, inst_data) in YAML_DATA['units'].items():
            if inst_name in inst_name_to_part_id:
                inst_coors[inst_name_to_part_id[inst_name]] = (inst_data['x_coor'], inst_data['y_coor'])
            else:
                print("Warning: No coordinates for '%s'" % inst_name)

        print("Info: Generating for topo '%s'" % topo_name)

        ############################# Link (Directly Connected Wires) #############################
        # if it matches the link topology type
        if topo_type.lower() == 'link':
            grout = True

            link_gen = LinkGenerator(YAML_DATA)
            ic_id = int(IC_DATA['ic_id'])

            # Generate message handler for each message in topology
            for msg_name in topo_groups:
                dest_map = gen_msg_dest_map(IC_DATA, msg_name)
                src_map = gen_msg_src_map(IC_DATA, msg_name)

                # Add the message type
                msg_types_s += link_gen.generate_msg_type(msg_name, dest_map)

                # Add to a per-partition IC interfaces list
                for (part_name, port_ids) in map_unique_parts_ports(IC_DATA, dest_map).items():
                    link_add_receiver(ic_id, part_name, msg_name, port_ids, len(src_map), len(dest_map), "recv_{}".format(msg_name.lower()), grout=grout )
                for (part_name, port_ids) in map_unique_parts_ports(IC_DATA, src_map).items():
                    link_add_sender(ic_id, part_name, msg_name, port_ids, len(src_map), len(dest_map), "send_{}".format(msg_name.lower()), grout=grout )

                # Record inst_id mapping on a per-interface basis
                # Since these messages are part of the same topology group, and this is on a
                # per-interface (part_type) basis, they should be the same among all message types
                # in the group.
                #
                # And record number of sources and dests (per port_id type)
                #
                # -- Destination map
                for part_name in map_unique_parts(IC_DATA,dest_map):
                    if part_name not in dest_map_per_iface:
                        dest_map_per_iface[part_name] = sorted(set( [x[1] for x in dest_map if x[0] == part_name] ))
                    else:
                        assert dest_map_per_iface[part_name] == sorted(set( [x[1] for x in dest_map if x[0] == part_name] ))
                # -- Source map
                for part_name in map_unique_parts(IC_DATA,src_map):
                    if part_name not in src_map_per_iface:
                        src_map_per_iface[part_name] = sorted(set( [x[1] for x in src_map if x[0] == part_name] ))
                    else:
                        assert src_map_per_iface[part_name] == sorted(set( [x[1] for x in src_map if x[0] == part_name] ))

                # Add top level connections
                if len(src_map)*len(dest_map) > 0:
                    if grout:
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("interconnect::LinkSenderPort<interconnect_config::msgs::{msg_name}, {num_dest}> *link_ic_isend_{msg_name}[{num_src}];"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))
                        ic_collate_add("interconnect::LinkReceiverPort<interconnect_config::msgs::{msg_name}, {num_src}> *link_ic_irecv_{msg_name}[{num_dest}];"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))

                        # *** Src map
                        for i in range(len(src_map)):
                            ic_collate_select(ic_id, grout_constructors_pre)
                            ic_collate_add("link_ic_isend_{msg_name}[{i}] = new interconnect::LinkSenderPort<interconnect_config::msgs::{msg_name}, {num_dest}>();"
                                                                .format(i=i, msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))

                        block_string = "for(int i=0; i < {num}; ++i)".format(num=len(src_map))
                        ic_collate_select(ic_id, grout_constructors_post, block_string)
                        ic_collate_add("link_ic_isend_{msg_name}[i]->clk(clk);"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))
                        ic_collate_add("link_ic_isend_{msg_name}[i]->rst(rst);"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))
                        ic_collate_add("link_ic_isend_{msg_name}[i]->Bind(link_ic_send_{msg_name}[i]);"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))
                        
                        ic_collate_select(ic_id, grout_constructors_post, block_string, "for(int j=0; j < {num_dest}; ++j)".format(num_dest=len(dest_map)) )
                        ic_collate_add("link_ic_isend_{msg_name}[i]->out_ports[j](link_ic_{msg_name}[{num_src}*j + i]);"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))


                        # *** Dest map
                        for i in range(len(dest_map)):
                            ic_collate_select(ic_id, grout_constructors_pre)
                            ic_collate_add("link_ic_irecv_{msg_name}[{i}] = new interconnect::LinkReceiverPort<interconnect_config::msgs::{msg_name}, {num_src}>();"
                                                                .format(i=i, msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))

                        block_string = "for(int i=0; i < {num}; ++i)".format(num=len(dest_map))
                        ic_collate_select(ic_id, grout_constructors_post, block_string)
                        ic_collate_add("link_ic_irecv_{msg_name}[i]->clk(clk);"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))
                        ic_collate_add("link_ic_irecv_{msg_name}[i]->rst(rst);"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))
                        ic_collate_add("link_ic_irecv_{msg_name}[i]->Bind(link_ic_recv_{msg_name}[i]);"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))
                   
                        ic_collate_select(ic_id, grout_constructors_post, block_string,  "for(int j=0; j < {num_dest}; ++j)".format(num_dest=len(dest_map)) )
                        ic_collate_add("link_ic_irecv_{msg_name}[i]->in_ports[j](link_ic_{msg_name}_retimed[{num_src}*i + j]);"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))

                        # *** Grout hookups
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("Connections::In<IC_MESSAGE({msg_name})> link_ic_send_{msg_name}[{num_src}];"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))
                        ic_collate_add("Connections::Out<IC_MESSAGE({msg_name})> link_ic_recv_{msg_name}[{num_dest}];"
                                                            .format(msg_name=msg_name, num_src=len(src_map), num_dest=len(dest_map)))
                        for i in range(len(src_map)):
                            report_pins_of_inst("link_ic_send_{msg_name}[{i}]".format(msg_name=msg_name, i=i), src_map[i][1], inst_name_to_part_id)
                        for i in range(len(dest_map)):
                            report_pins_of_inst("link_ic_recv_{msg_name}[{i}]".format(msg_name=msg_name, i=i), dest_map[i][1], inst_name_to_part_id)

                        ic_collate_add("Connections::Combinational<IC_MESSAGE({msg_name})> link_ic_{msg_name}[{num}];"
                                                            .format(msg_name=msg_name, num=len(src_map)*len(dest_map)))
                        ic_collate_add("Connections::Combinational<IC_MESSAGE({msg_name})> link_ic_{msg_name}_retimed[{num}];"
                                                            .format(msg_name=msg_name, num=len(src_map)*len(dest_map)))

                    else:
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("Connections::In<IC_MESSAGE({msg_name})> link_ic_{msg_name}[{num}];"
                                                            .format(msg_name=msg_name, num=len(src_map)*len(dest_map)))
                        ic_collate_add("Connections::Out<IC_MESSAGE({msg_name})> link_ic_{msg_name}_retimed[{num}];"
                                                            .format(msg_name=msg_name, num=len(src_map)*len(dest_map)))


                    if grout:
                        add_collated_combinationals("link_ic_send", {msg_name: len(src_map)})
                        add_collated_combinationals("link_ic_recv", {msg_name: len(dest_map)})
                    else:
                        add_collated_combinationals("link_ic", {msg_name: len(src_map)*len(dest_map)})
                        add_collated_combinationals("link_ic", {msg_name: len(src_map)*len(dest_map)}, "retimed")
                    

                    # retiming stages is a dict of template parameters -> indexes
                    a_retiming_stages = dict()
                    for dest_idx in range(len(dest_map)):
                        (dest_part_name, dest_inst_id, dest_port_id) = dest_map[dest_idx]
                        for src_idx in range(len(src_map)):
                            (src_part_name, src_inst_id, src_port_id) = src_map[src_idx]
                            m_dist=mdist_global_wire(YAML_DATA, msg_name, inst_coors[src_inst_id], inst_coors[dest_inst_id])

                            retiming_stages=max(math.ceil(m_dist/topo_options['wire_prop_speed'] - 1), 0) + int(topo_options.get('extra_latency',0))
                            extra_capacity = int(topo_options.get('extra_capacity', 0)/2)

                            idx = len(src_map)*dest_idx + src_idx

                            a_retiming_stages.setdefault((msg_name, retiming_stages, extra_capacity),list())
                            a_retiming_stages[(msg_name, retiming_stages, extra_capacity)].append(idx)
                            global_wire_manifest.append({'name': 'link_ic_{msg_name}_{idx}'.format(msg_name=msg_name, idx=idx), 'src_idx': src_inst_id, 'dest_idx': dest_inst_id, 'bus_width': get_all_msg_widths(YAML_DATA)[msg_name], 'wire_prop_speed': topo_options['wire_prop_speed']})
                    add_collated_retimings("link_ic", a_retiming_stages)
                                                                
                if len(dest_map) > 0:
                    this_src_idx_offset = 0
                    for (src_part_name, src_port_ids) in map_unique_parts_ports(IC_DATA, src_map).items():
                        interface_src_binds_per_iface.setdefault(src_part_name, list())
                        if grout:
                            for src_port_id in src_port_ids:
                                interface_src_binds_per_iface[src_part_name].append( (
                                    "",
                                    "ic_interface.send_{msg_name_lower}_{port_id}(link_ic_send_{msg_name}[(this_src_idx + {this_src_idx_offset})]);"
                                        .format(msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=src_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_src_idx_offset=this_src_idx_offset)
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_src_idx_offset += len(src_map_per_iface[src_part_name])
                        else:
                            for src_port_id in src_port_ids:
                                interface_src_binds_per_iface[src_part_name].append( (
                                    "for(int dest_idx=0; dest_idx < {}; ++dest_idx)".format(len(dest_map)),
                                    "ic_interface.send_{msg_name_lower}_{port_id}.out_ports[dest_idx](link_ic_{msg_name}[{num_srcs}*dest_idx + (this_src_idx + {this_src_idx_offset})]);"
                                        .format(msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=src_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_src_idx_offset=this_src_idx_offset)
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_src_idx_offset += len(src_map_per_iface[src_part_name])

                if len(src_map) > 0:
                    this_dest_idx_offset = 0
                    for (dest_part_name, dest_port_ids) in map_unique_parts_ports(IC_DATA, dest_map).items():
                        interface_dest_binds_per_iface.setdefault(dest_part_name, list())
                        if grout:
                            for dest_port_id in dest_port_ids:
                                interface_dest_binds_per_iface[dest_part_name].append( (
                                    "",
                                    "ic_interface.recv_{msg_name_lower}_{port_id}(link_ic_recv_{msg_name}[(this_dest_idx + {this_dest_idx_offset})]);"
                                        .format(msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=dest_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_dest_idx_offset=this_dest_idx_offset)
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_dest_idx_offset += len(dest_map_per_iface[dest_part_name])
                        else:
                            for dest_port_id in dest_port_ids:
                                interface_dest_binds_per_iface[dest_part_name].append( (
                                    "for(int src_idx=0; src_idx < {}; ++src_idx)".format(len(src_map)),
                                    "ic_interface.recv_{msg_name_lower}_{port_id}.in_ports[src_idx](link_ic_{msg_name}_retimed[{num_srcs}*(this_dest_idx + {this_dest_idx_offset}) + src_idx]);"
                                        .format(msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=dest_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_dest_idx_offset=this_dest_idx_offset)
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_dest_idx_offset += len(dest_map_per_iface[dest_part_name])

        ################################## Crossbar ##################################
        elif topo_type.lower() == 'crossbar':
            grout = True
            # grout = False

            link_gen = CrossbarGenerator(YAML_DATA)
            ic_id = int(IC_DATA['ic_id'])

            # Flag if this topo group will be super messaged
            # Always supermsg, since the unicast -> multicast right now is in the joiner/splitter
            is_supermsg = True
            
            # Add super message related classes
            if is_supermsg:
                supermsg_dest_parts = dict()
                supermsg_src_parts = dict()
                supermsg_dest_maps = dict()
                supermsg_src_maps = dict()

                # key = part_type, value = list of messages to support in splitter/joiner
                supermsg_local_dest_msgs_per_part_type = dict()
                supermsg_local_src_msgs_per_part_type = dict()

                for msg_name in topo_groups:
                    dest_map = gen_msg_dest_map(IC_DATA, msg_name)
                    src_map = gen_msg_src_map(IC_DATA, msg_name)

                    supermsg_dest_maps[msg_name] = dest_map
                    supermsg_src_maps[msg_name] = src_map

                    # Record inst_id mapping on a per-interface basis
                    # Since these messages are part of the same topology group, and this is on a
                    # per-interface (part_type) basis, they should be the same among all message types
                    # in the group.
                    #
                    # And record number of sources and dests (per port_id type)
                    #
                    # -- Destination map
                    for part_name in map_unique_parts(IC_DATA,dest_map):
                        if part_name not in dest_map_per_iface:
                            dest_map_per_iface[part_name] = sorted(set( [x[1] for x in dest_map if x[0] == part_name] ))
                        else:
                            assert dest_map_per_iface[part_name] == sorted(set( [x[1] for x in dest_map if x[0] == part_name] ))
                    # -- Source map
                    for part_name in map_unique_parts(IC_DATA,src_map):
                        if part_name not in src_map_per_iface:
                            src_map_per_iface[part_name] = sorted(set( [x[1] for x in src_map if x[0] == part_name] ))
                        else:
                            assert src_map_per_iface[part_name] == sorted(set( [x[1] for x in src_map if x[0] == part_name] ))

                    # Record unique source/dest partitions to add splitter joiner to
                    for part_name in map_unique_parts(IC_DATA, dest_map):
                        supermsg_dest_parts[part_name] = sorted(list(set([x[1] for x in dest_map if x[0] == part_name])))
                        supermsg_local_dest_msgs_per_part_type.setdefault(part_name, set())
                        supermsg_local_dest_msgs_per_part_type[part_name].add(msg_name)
                    for part_name in map_unique_parts(IC_DATA, src_map):
                        supermsg_src_parts[part_name] = sorted(list(set([x[1] for x in src_map if x[0] == part_name])))
                        supermsg_local_src_msgs_per_part_type.setdefault(part_name, set())
                        supermsg_local_src_msgs_per_part_type[part_name].add(msg_name)

                    # Add the binds to the splitter/joiner
                    # dests
                    for (part_name, port_ids) in map_unique_parts_ports(IC_DATA, dest_map).items():
                        add_ic_type(ic_id)
                        add_part_type(part_name)

                        if grout:
                            b_s = 12*' '
                            for port_id in port_ids:
                                inst_name_base =  "recv_{}".format(msg_name.lower())
                                inst_name = inst_name_base + "_" + str(port_id)
                                handler_instances[part_name].append( ("Connections::In", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_name)], inst_name) )
                                b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                            b_s += "{{}}".format()
                        else:
                            b_s = 12*' '
                            for port_id in port_ids:
                                inst_name = "splitter_{supermsg_idx}.out_port_{msg_name_lower}_{port_id}".format(supermsg_idx=supermsg_idx,
                                                                                                            msg_name_lower=msg_name.lower(),
                                                                                                            port_id=port_id)
                                b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                            b_s += "{{}}".format()

                        handler_binds[part_name].append("""\
        template <unsigned int BIND_ID>
        void Bind(Connections::InBlocking<interconnect::Msg<interconnect_config::msgs::{msg_type}> > &p, unsigned int port_id) {{
{b_s}
        }}""".format(msg_type=msg_name, inst_name=inst_name, b_s=b_s))

                    # Add the binds to the splitter/joiner
                    # srcs
                    for (part_name, port_ids) in map_unique_parts_ports(IC_DATA, src_map).items():
                        add_ic_type(ic_id)
                        add_part_type(part_name)

                        if grout:
                            b_s = 12*' '
                            for port_id in port_ids:
                                inst_name_base =  "send_{}".format(msg_name.lower())
                                inst_name = inst_name_base + "_" + str(port_id)
                                handler_instances[part_name].append( ("Connections::Out", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_name)], inst_name) )
                                b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                            b_s += "{{}}".format()
                        else:
                            b_s = 12*' '
                            for port_id in port_ids:
                                inst_name = "joiner_{supermsg_idx}.in_port_{msg_name_lower}_{port_id}".format(supermsg_idx=supermsg_idx,
                                                                                                            msg_name_lower=msg_name.lower(),
                                                                                                            port_id=port_id)
                                b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                            b_s += "{{}}".format()

                        handler_binds[part_name].append("""\
        template <unsigned int BIND_ID>
        void Bind(Connections::OutBlocking<interconnect::Msg<interconnect_config::msgs::{msg_type}> > &p, unsigned int port_id) {{
{b_s}
        }}""".format(msg_type=msg_name, inst_name=inst_name, b_s=b_s))


                # Add the joiner and splittter instances, and bindings
                if grout:
                    __idx_offset = 0
                    for part_name in sorted(supermsg_dest_parts.keys()):
                        inst_name="splitter_{supermsg_idx}_{part_name}".format(supermsg_idx=supermsg_idx, part_name=part_name)
                        num=len(supermsg_dest_parts[part_name])
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("interconnect::Splitter_{part_name}_{supermsg_idx}<> *{inst_name}[{num}];"
                                                        .format(part_name=part_name,
                                                                supermsg_idx=supermsg_idx,
                                                                inst_name=inst_name,
                                                                num=num))
                        ic_collate_select(ic_id, grout_constructors_pre, "for(int i=0; i < {num}; ++i)".format(num=num))
                        ic_collate_add("{inst_name}[i] = new interconnect::Splitter_{part_name}_{supermsg_idx}<>();"
                                        .format(part_name=part_name,
                                                supermsg_idx=supermsg_idx,
                                                inst_name=inst_name))
                        ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=num))
                        ic_collate_add("{inst_name}[i]->clk(clk);".format(inst_name=inst_name))
                        ic_collate_add("{inst_name}[i]->rst(rst);".format(inst_name=inst_name))
                        ic_collate_add("{inst_name}[i]->in_port(crossbar_ic_recv_{msg_name}_retimed[i + {this_dest_idx_offset}]);"
                                        .format(inst_name=inst_name,
                                                msg_name="SUPERMSG_{}".format(supermsg_idx),
                                                this_dest_idx_offset=__idx_offset))
                        __idx_offset += num

                        for sub_msg_name in supermsg_local_dest_msgs_per_part_type[part_name]:
                            ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=num))
                            ic_collate_add("{inst_name}[i]->out_port_{msg_name_lower}_{port_id}(link_ic_recv_{part_name}_{msg_name}[i]);"
                                                        .format(inst_name=inst_name,
                                                                part_name=part_name,
                                                                msg_name=sub_msg_name,
                                                                msg_name_lower=sub_msg_name.lower(),
                                                                port_id=0))

                    __idx_offset = 0
                    for part_name in sorted(supermsg_src_parts.keys()):
                        inst_name="joiner_{supermsg_idx}_{part_name}".format(supermsg_idx=supermsg_idx, part_name=part_name)
                        num=len(supermsg_src_parts[part_name])
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("interconnect::Joiner_{part_name}_{supermsg_idx}<> *{inst_name}[{num}];"
                                                        .format(part_name=part_name,
                                                                supermsg_idx=supermsg_idx,
                                                                inst_name=inst_name,
                                                                num=num))
                        ic_collate_select(ic_id, grout_constructors_pre, "for(int i=0; i < {num}; ++i)".format(num=num))
                        ic_collate_add("{inst_name}[i] = new interconnect::Joiner_{part_name}_{supermsg_idx}<>();"
                                        .format(part_name=part_name,
                                                supermsg_idx=supermsg_idx,
                                                inst_name=inst_name))
                        ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=num))
                        ic_collate_add("{inst_name}[i]->clk(clk);".format(inst_name=inst_name))
                        ic_collate_add("{inst_name}[i]->rst(rst);".format(inst_name=inst_name))
                        ic_collate_add("{inst_name}[i]->out_port(crossbar_ic_send_{msg_name}[i + {this_src_idx_offset}]);"
                                        .format(inst_name=inst_name,
                                                msg_name="SUPERMSG_{}".format(supermsg_idx),
                                                this_src_idx_offset=__idx_offset))
                        __idx_offset += num

                        for sub_msg_name in supermsg_local_src_msgs_per_part_type[part_name]:
                            ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=num))
                            ic_collate_add("{inst_name}[i]->in_port_{msg_name_lower}_{port_id}(link_ic_send_{part_name}_{msg_name}[i]);"
                                                            .format(inst_name=inst_name,
                                                                    part_name=part_name,
                                                                    msg_name=sub_msg_name,
                                                                    msg_name_lower=sub_msg_name.lower(),
                                                                    port_id=0))

                else:
                    for part_name in sorted(supermsg_dest_parts.keys()):
                        add_ic_type(ic_id)
                        add_part_type(part_name)
                        inst_name="splitter_{}".format(supermsg_idx)
                        handler_instances[part_name].append( ("Splitter_{part_name}_{supermsg_idx}".format(part_name=part_name, supermsg_idx=supermsg_idx), [], inst_name) )
                        handler_constructors[part_name] += ["{}.clk(clk);".format(inst_name), "{}.rst(rst);".format(inst_name)]

                    for part_name in sorted(supermsg_src_parts.keys()):
                        add_ic_type(ic_id)
                        add_part_type(part_name)
                        inst_name="joiner_{}".format(supermsg_idx)
                        handler_instances[part_name].append( ("Joiner_{part_name}_{supermsg_idx}".format(part_name=part_name, supermsg_idx=supermsg_idx), [], inst_name) )
                        handler_constructors[part_name] += ["{}.clk(clk);".format(inst_name), "{}.rst(rst);".format(inst_name)]

                # Need to pull out port_ids here
                # port_id really should be a destination list thing, not a message type thing
                # 0 should be from a global counter for super messages

                (super_msg_decls_s_, super_msg_types_s_, icg_config_msg_types_a_, supermsg_msg_width) = SplitterJoiner().create_supermsg(YAML_DATA, topo_groups, supermsg_dest_maps, supermsg_src_maps, supermsg_idx)

                super_msg_decls_s += super_msg_decls_s_ + '\n'
                super_msg_types_s += super_msg_types_s_ + '\n'
                icg_config_msg_types_a += icg_config_msg_types_a_

                # once we encode part_id as an sc_in that splitter can use, we can have the real port_id list
                # and have splitter decode dest_bits properly
                #
                # Through a function in supermsg that given a part_id AND port_id (is_port_id(part_id, port_id)) it will return true or false.
                for (part_name, local_msg_list) in supermsg_local_dest_msgs_per_part_type.items():
                    super_msg_classes_s += SplitterJoiner().create_splitter(part_name, [(x, 0) for x in topo_groups], local_msg_list, supermsg_idx, grout=grout) + '\n\n'
                for (part_name, local_msg_list) in supermsg_local_src_msgs_per_part_type.items():
                    super_msg_classes_s += SplitterJoiner().create_joiner(part_name, [(x, 0) for x in topo_groups], local_msg_list, supermsg_idx, grout=grout) + '\n\n'


                for msg_name in topo_groups:
                    dest_map = gen_msg_dest_map(IC_DATA, msg_name)
                    src_map = gen_msg_src_map(IC_DATA, msg_name)

                    # Add the message type
                    msg_types_s += link_gen.generate_msg_type(msg_name, dest_map)

                # Add top level connections
                msg_name = "SUPERMSG_{}".format(supermsg_idx)
                num_dests = sum([len(x) for x in supermsg_dest_parts.values()])
                num_srcs = sum([len(x) for x in supermsg_src_parts.values()])
                ic_collate_select(ic_id, grout_instances)
                ic_collate_add("interconnect::CrossbarICTop<interconnect_config_icg::msgs::{msg_name}, {num_srcs}, {num_dests}, 1, 1> crossbar_ic_{msg_name};"
                                                        .format(msg_name=msg_name, num_srcs=num_srcs, num_dests=num_dests))
                ic_collate_select(ic_id, grout_constructors)
                ic_collate_add("crossbar_ic_{msg_name}.clk(clk);".format(msg_name=msg_name))
                ic_collate_add("crossbar_ic_{msg_name}.rst(rst);".format(msg_name=msg_name))
                if num_srcs > 0:
                    ic_collate_select(ic_id, grout_instances)
                    if grout:
                        ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > crossbar_ic_send_{msg_name}[{num_srcs}];" \
                                                            .format(msg_name=msg_name, num_srcs=num_srcs, num_dests=num_dests))
                    else:
                        ic_collate_add("Connections::In<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > crossbar_ic_send_{msg_name}[{num_srcs}];" \
                                                            .format(msg_name=msg_name, num_srcs=num_srcs, num_dests=num_dests))
                    ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > crossbar_ic_send_{msg_name}_retimed[{num_srcs}];" \
                                                        .format(msg_name=msg_name, num_srcs=num_srcs, num_dests=num_dests))
                    ic_collate_select(ic_id, grout_constructors)
                    ic_collate_add("for (int i=0; i < {num_srcs}; ++i) {{ crossbar_ic_{msg_name}.in_ports[i](crossbar_ic_send_{msg_name}_retimed[i]); }}" \
                                                            .format(msg_name=msg_name, num_srcs=num_srcs, num_dests=num_dests))
                    if not grout:
                        add_collated_combinationals("crossbar_ic_send", {msg_name: num_srcs}, "", "interconnect_config_icg")
                if num_dests > 0:
                    ic_collate_select(ic_id, grout_instances)
                    ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > crossbar_ic_recv_{msg_name}[{num_dests}];" \
                                                        .format(msg_name=msg_name, num_srcs=num_srcs, num_dests=num_dests))
                    if grout:
                        ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > crossbar_ic_recv_{msg_name}_retimed[{num_dests}];" \
                                                            .format(msg_name=msg_name, num_srcs=num_srcs, num_dests=num_dests))
                    else:
                        ic_collate_add("Connections::Out<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > crossbar_ic_recv_{msg_name}_retimed[{num_dests}];" \
                                                            .format(msg_name=msg_name, num_srcs=num_srcs, num_dests=num_dests))
                    ic_collate_select(ic_id, grout_constructors)
                    ic_collate_add("for (int i=0; i < {num_dests}; ++i) {{ crossbar_ic_{msg_name}.out_ports[i](crossbar_ic_recv_{msg_name}[i]); }}" \
                                                            .format(msg_name=msg_name, num_srcs=num_srcs, num_dests=num_dests))
                    if not grout:
                        add_collated_combinationals("crossbar_ic_recv", {msg_name: num_dests}, "retimed", "interconnect_config_icg")

                idx = 0
                a_retiming_stages = dict()
                for (src_part_name, src_port_ids) in sorted([ (x, [0]) for x in supermsg_src_parts.keys() ]):
                    for inst_id in sorted(supermsg_src_parts[src_part_name]):
                        m_dist=mdist_global_wire(YAML_DATA, msg_name, inst_coors[inst_id], (topo_options['crossbar_x_coor'], topo_options['crossbar_y_coor']), w=supermsg_msg_width)
                        extra_capacity = int(topo_options.get('extra_capacity', 0)/2)
                        retiming_stages=max(math.ceil(m_dist/topo_options['wire_prop_speed'] - 1), 0)

                        a_retiming_stages.setdefault((msg_name, retiming_stages, extra_capacity),list())
                        a_retiming_stages[(msg_name, retiming_stages, extra_capacity)].append(idx)
                        global_wire_manifest.append({'name': 'crossbar_ic_send_{msg_name}_{idx}'.format(msg_name=msg_name, idx=idx), 'src_coors': inst_coors[inst_id], 'dest_coors': (topo_options['crossbar_x_coor'], topo_options['crossbar_y_coor']), 'bus_width': supermsg_msg_width, 'wire_prop_speed': topo_options['wire_prop_speed']})

                        idx += 1
                add_collated_retimings("crossbar_ic_send", a_retiming_stages, "interconnect_config_icg")


                idx = 0
                a_retiming_stages = dict()
                for (dest_part_name, dest_port_ids) in sorted([ (x, [0]) for x in supermsg_dest_parts.keys() ]):
                    for inst_id in sorted(supermsg_dest_parts[dest_part_name]):
                        m_dist=mdist_global_wire(YAML_DATA, msg_name, inst_coors[inst_id], (topo_options['crossbar_x_coor'], topo_options['crossbar_y_coor']), w=supermsg_msg_width)
                        retiming_stages=max(math.ceil(m_dist/topo_options['wire_prop_speed'] - 1), 0) + int(topo_options.get('extra_latency',0))
                        extra_capacity = topo_options.get('extra_capacity', 0)

                        a_retiming_stages.setdefault((msg_name, retiming_stages, extra_capacity),list())
                        a_retiming_stages[(msg_name, retiming_stages, extra_capacity)].append(idx)
                        global_wire_manifest.append({'name': 'crossbar_ic_recv_{msg_name}_{idx}'.format(msg_name=msg_name, idx=idx), 'src_coors': (topo_options['crossbar_x_coor'], topo_options['crossbar_y_coor']), 'dest_coors': inst_coors[inst_id], 'bus_width': supermsg_msg_width, 'wire_prop_speed': topo_options['wire_prop_speed']})

                        idx += 1
                add_collated_retimings("crossbar_ic_recv", a_retiming_stages, "interconnect_config_icg")

                # Interface source and destination binds
                if num_dests > 0:
                    if grout:
                        this_src_idx_offset = defaultdict(lambda: defaultdict(lambda: 0))
                    else:
                        this_src_idx_offset = 0

                    # This needs to accomodate port_ids in the future
                    for (src_part_name, src_port_ids) in sorted([ (x, [0]) for x in supermsg_src_parts.keys() ]):
                        interface_src_binds_per_iface.setdefault(src_part_name, list())
                        if grout:
                            for src_port_id in src_port_ids:
                                for sub_msg_name in supermsg_local_src_msgs_per_part_type[src_part_name]:
                                    interface_src_binds_per_iface[src_part_name].append( (
                                        "",
                                        "ic_interface.send_{msg_name_lower}_{port_id}(link_ic_send_{part_name}_{msg_name}[(this_src_idx + {this_src_idx_offset})]);"
                                            .format(msg_name=sub_msg_name,
                                                    msg_name_lower=sub_msg_name.lower(),
                                                    port_id=src_port_id,
                                                    num_dests=len(dest_map),
                                                    num_srcs=len(src_map),
                                                    part_name=src_part_name,
                                                    this_src_idx_offset=this_src_idx_offset[src_part_name][sub_msg_name])
                                    ) )
                                    # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                    this_src_idx_offset[src_part_name][sub_msg_name] += len(src_map_per_iface[src_part_name])
                        else:
                            for src_port_id in src_port_ids:
                                interface_src_binds_per_iface[src_part_name].append( (
                                    "",
                                    "ic_interface.joiner_{supermsg_idx}.out_port(crossbar_ic_send_{msg_name}[this_src_idx + {this_src_idx_offset}]);"
                                        .format(supermsg_idx=supermsg_idx, msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=src_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_src_idx_offset=this_src_idx_offset)
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_src_idx_offset += len(src_map_per_iface[src_part_name])

                if num_srcs > 0:
                    if grout:
                        this_dest_idx_offset = defaultdict(lambda: defaultdict(lambda: 0))
                    else:
                        this_dest_idx_offset = 0

                    # This needs to accomodate port_ids in the future
                    for (dest_part_name, dest_port_ids) in sorted([ (x, [0]) for x in supermsg_dest_parts.keys() ]):
                        interface_dest_binds_per_iface.setdefault(dest_part_name, list())
                        if grout:
                            for dest_port_id in dest_port_ids:
                                for sub_msg_name in supermsg_local_dest_msgs_per_part_type[dest_part_name]:
                                    interface_dest_binds_per_iface[dest_part_name].append( (
                                        "",
                                        "ic_interface.recv_{msg_name_lower}_{port_id}(link_ic_recv_{part_name}_{msg_name}[(this_dest_idx + {this_dest_idx_offset})]);"
                                            .format(msg_name=sub_msg_name,
                                                    msg_name_lower=sub_msg_name.lower(),
                                                    port_id=dest_port_id,
                                                    num_dests=len(dest_map),
                                                    num_srcs=len(src_map),
                                                    part_name=dest_part_name,
                                                    this_dest_idx_offset=this_dest_idx_offset[dest_part_name][sub_msg_name])
                                    ) )
                                    # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                    this_dest_idx_offset[dest_part_name][sub_msg_name] += len(dest_map_per_iface[dest_part_name])
                        else:
                            for dest_port_id in dest_port_ids:
                                interface_dest_binds_per_iface[dest_part_name].append( (
                                    "",
                                    "ic_interface.splitter_{supermsg_idx}.in_port(crossbar_ic_recv_{msg_name}_retimed[this_dest_idx + {this_dest_idx_offset}]);"
                                        .format(supermsg_idx=supermsg_idx, msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=dest_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_dest_idx_offset=this_dest_idx_offset)
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_dest_idx_offset += len(dest_map_per_iface[dest_part_name])

                if grout:
                    for msg_name in topo_groups:
                        for src_part_name in this_src_idx_offset.keys():
                            if msg_name not in this_src_idx_offset[src_part_name].keys():
                                continue
                            ic_collate_select(ic_id, interconnect_instances)
                            ic_collate_add("Connections::Combinational<IC_MESSAGE({msg_name})> link_ic_send_{part_name}_{msg_name}[{num}];"
                                                                    .format(msg_name=msg_name,
                                                                            part_name=src_part_name,
                                                                            num=this_src_idx_offset[src_part_name][msg_name]))
                            ic_collate_select(ic_id, grout_instances)
                            ic_collate_add("Connections::In<IC_MESSAGE({msg_name})> link_ic_send_{part_name}_{msg_name}[{num}];"
                                                                    .format(msg_name=msg_name,
                                                                            part_name=src_part_name,
                                                                            num=this_src_idx_offset[src_part_name][msg_name]))
                            
                            assert len(supermsg_src_maps[msg_name]) == this_src_idx_offset[src_part_name][msg_name]
                            for i in range(len(supermsg_src_maps[msg_name])):
                                report_pins_of_inst("link_ic_send_{part_name}_{msg_name}[{i}]".format(msg_name=msg_name,
                                                                                                      part_name=src_part_name,
                                                                                                      num=this_src_idx_offset[src_part_name][msg_name],
                                                                                                      i=i), supermsg_src_maps[msg_name][i][1], inst_name_to_part_id)

                            ic_collate_select(ic_id, interconnect_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=this_src_idx_offset[src_part_name][msg_name]) )
                            ic_collate_add("grout_{ic_id}.{base_name}_{part_name}_{msg_name}{postfix}[i]({base_name}_{part_name}_{msg_name}{postfix}[i]);"
                                                                    .format(base_name="link_ic_send",
                                                                            msg_name=msg_name,
                                                                            part_name=src_part_name,
                                                                            postfix="",
                                                                            ic_id=ic_id))
                        for dest_part_name in this_dest_idx_offset.keys():
                            if msg_name not in this_dest_idx_offset[dest_part_name].keys():
                                continue
                            ic_collate_select(ic_id, interconnect_instances)
                            ic_collate_add("Connections::Combinational<IC_MESSAGE({msg_name})> link_ic_recv_{part_name}_{msg_name}[{num}];"
                                                                    .format(msg_name=msg_name,
                                                                            part_name=dest_part_name,
                                                                            num=this_dest_idx_offset[dest_part_name][msg_name]))
                            ic_collate_select(ic_id, grout_instances)
                            ic_collate_add("Connections::Out<IC_MESSAGE({msg_name})> link_ic_recv_{part_name}_{msg_name}[{num}];"
                                                                    .format(msg_name=msg_name,
                                                                    part_name=dest_part_name,
                                                                    num=this_dest_idx_offset[dest_part_name][msg_name]))

                            assert len(supermsg_dest_maps[msg_name]) == this_dest_idx_offset[dest_part_name][msg_name]
                            for i in range(len(supermsg_dest_maps[msg_name])):
                                report_pins_of_inst("link_ic_recv_{part_name}_{msg_name}[{i}]".format(msg_name=msg_name,
                                                                                                      part_name=dest_part_name,
                                                                                                      num=this_dest_idx_offset[dest_part_name][msg_name],
                                                                                                      i=i), supermsg_dest_maps[msg_name][i][1], inst_name_to_part_id)

                            ic_collate_select(ic_id, interconnect_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=this_dest_idx_offset[dest_part_name][msg_name]) )
                            ic_collate_add("grout_{ic_id}.{base_name}_{part_name}_{msg_name}{postfix}[i]({base_name}_{part_name}_{msg_name}{postfix}[i]);"
                                                                    .format(base_name="link_ic_recv",
                                                                            msg_name=msg_name,
                                                                            part_name=dest_part_name,
                                                                            postfix="",
                                                                            ic_id=ic_id))

                                                            
            else:                   
                for msg_name in topo_groups:
                    dest_map = gen_msg_dest_map(IC_DATA, msg_name)
                    src_map = gen_msg_src_map(IC_DATA, msg_name)

                    # Add the message type
                    msg_types_s += link_gen.generate_msg_type(msg_name, dest_map)

                    # Generate pass-through to go up to global interconnect
                    # Add to a per-partition IC interfaces list
                    for (part_name, port_ids) in map_unique_parts_ports(IC_DATA, dest_map).items():
                        crossbar_add_receiver(ic_id, part_name, msg_name, port_ids, len(src_map), len(dest_map), "recv_{}".format(msg_name.lower()) )
                    for (part_name, port_ids) in map_unique_parts_ports(IC_DATA, src_map).items():
                        crossbar_add_sender(ic_id, part_name, msg_name, port_ids, len(src_map), len(dest_map), "send_{}".format(msg_name.lower()) )

                    # Record inst_id mapping on a per-interface basis
                    # Since these messages are part of the same topology group, and this is on a
                    # per-interface (part_type) basis, they should be the same among all message types
                    # in the group.
                    #
                    # And record number of sources and dests (per port_id type)
                    #
                    # -- Destination map
                    for part_name in map_unique_parts(IC_DATA,dest_map):
                        if part_name not in dest_map_per_iface:
                            dest_map_per_iface[part_name] = sorted(set( [x[1] for x in dest_map if x[0] == part_name] ))
                        else:
                            assert dest_map_per_iface[part_name] == sorted(set( [x[1] for x in dest_map if x[0] == part_name] ))
                    # -- Source map
                    for part_name in map_unique_parts(IC_DATA,src_map):
                        if part_name not in src_map_per_iface:
                            src_map_per_iface[part_name] = sorted(set( [x[1] for x in src_map if x[0] == part_name] ))
                        else:
                            assert src_map_per_iface[part_name] == sorted(set( [x[1] for x in src_map if x[0] == part_name] ))

                    # Add top level connections
                    ic_collate_select(ic_id, grout_instances)
                    ic_collate_add("interconnect::CrossbarICTop<interconnect_config::msgs::{msg_name}, {num_srcs}, {num_dests}, 1, 1> crossbar_ic_{msg_name};"
                                                            .format(msg_name=msg_name, num_srcs=len(src_map), num_dests=len(dest_map)))
                    ic_collate_select(ic_id, grout_constructors)
                    ic_collate_add("crossbar_ic_{msg_name}.clk(clk);".format(msg_name=msg_name))
                    ic_collate_add("crossbar_ic_{msg_name}.rst(rst);".format(msg_name=msg_name))
                    if len(src_map) > 0:
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("Connections::In<IC_MESSAGE({msg_name})> crossbar_ic_send_{msg_name}[{num_srcs}];"
                                                            .format(msg_name=msg_name, num_srcs=len(src_map), num_dests=len(dest_map)))
                        ic_collate_select(ic_id, grout_constructors)
                        ic_collate_add("for (int i=0; i < {num_srcs}; ++i) {{ crossbar_ic_{msg_name}.in_ports[i](crossbar_ic_send_{msg_name}[i]); }}"
                                                                .format(msg_name=msg_name, num_srcs=len(src_map), num_dests=len(dest_map)))
                        add_collated_combinationals("crossbar_ic_send", {msg_name: num_srcs})
                    if len(dest_map) > 0:
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("Connections::Out<IC_MESSAGE({msg_name})> crossbar_ic_recv_{msg_name}[{num_dests}];"
                                                            .format(msg_name=msg_name, num_srcs=len(src_map), num_dests=len(dest_map)))
                        ic_collate_select(ic_id, grout_constructors)
                        ic_collate_add("for (int i=0; i < {num_dests}; ++i) {{ crossbar_ic_recv_{msg_name}.out_ports[i](crossbar_ic_recv_{msg_name}[i]); }}"
                                                                .format(msg_name=msg_name, num_srcs=len(src_map), num_dests=len(dest_map)))
                        add_collated_combinationals("crossbar_ic_recv", {msg_name: num_dests})

                    if len(dest_map) > 0:
                        this_src_idx_offset = 0
                        for (src_part_name, src_port_ids) in map_unique_parts_ports(IC_DATA, src_map).items():
                            interface_src_binds_per_iface.setdefault(src_part_name, list())
                            for src_port_id in src_port_ids:
                                interface_src_binds_per_iface[src_part_name].append( (
                                    "",
                                    "ic_interface.send_{msg_name_lower}_{port_id}(crossbar_ic_send_{msg_name}[this_src_idx + {this_src_idx_offset}]);"
                                        .format(msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=src_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_src_idx_offset=this_src_idx_offset)
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_src_idx_offset += len(src_map_per_iface[src_part_name])

                    if len(src_map) > 0:
                        this_dest_idx_offset = 0
                        for (dest_part_name, dest_port_ids) in map_unique_parts_ports(IC_DATA, dest_map).items():
                            interface_dest_binds_per_iface.setdefault(dest_part_name, list())
                            for dest_port_id in dest_port_ids:
                                interface_dest_binds_per_iface[dest_part_name].append( (
                                    "",
                                    "ic_interface.recv_{msg_name_lower}_{port_id}(crossbar_ic_recv_{msg_name}[this_dest_idx + {this_dest_idx_offset}]);"
                                        .format(msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=dest_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_dest_idx_offset=this_dest_idx_offset)
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_dest_idx_offset += len(dest_map_per_iface[dest_part_name])
            if is_supermsg:
                supermsg_idx += 1

        ################################## NoC ##################################
        elif topo_type.lower() == 'noc':
            grout = True
            # grout = False

            link_gen = CrossbarGenerator(YAML_DATA) # Reuse the crossbar generator for NoC
            ic_id = int(IC_DATA['ic_id'])

            supermsg_dest_parts = dict()
            supermsg_src_parts = dict()
            supermsg_dest_maps = dict()
            supermsg_src_maps = dict()

            # key = part_type, value = list of messages to support in splitter/joiner
            supermsg_local_dest_msgs_per_part_type = dict()
            supermsg_local_src_msgs_per_part_type = dict()

            for msg_name in topo_groups:
                dest_map = gen_msg_dest_map(IC_DATA, msg_name)
                src_map = gen_msg_src_map(IC_DATA, msg_name)

                supermsg_dest_maps[msg_name] = dest_map
                supermsg_src_maps[msg_name] = src_map
            
                # Record inst_id mapping on a per-interface basis
                # Since these messages are part of the same topology group, and this is on a
                # per-interface (part_type) basis, they should be the same among all message types
                # in the group.
                #
                # And record number of sources and dests (per port_id type)
                #
                # -- Destination map
                for part_name in map_unique_parts(IC_DATA,dest_map):
                    if part_name not in dest_map_per_iface:
                        dest_map_per_iface[part_name] = sorted(set( [x[1] for x in dest_map if x[0] == part_name] ))
                    else:
                        assert dest_map_per_iface[part_name] == sorted(set( [x[1] for x in dest_map if x[0] == part_name] ))
                # -- Source map
                for part_name in map_unique_parts(IC_DATA,src_map):
                    if part_name not in src_map_per_iface:
                        src_map_per_iface[part_name] = sorted(set( [x[1] for x in src_map if x[0] == part_name] ))
                    else:
                        assert src_map_per_iface[part_name] == sorted(set( [x[1] for x in src_map if x[0] == part_name] ))

                # Record unique source/dest partitions to add splitter joiner to
                for part_name in map_unique_parts(IC_DATA, dest_map):
                    supermsg_dest_parts[part_name] = sorted(list(set([x[1] for x in dest_map if x[0] == part_name])))
                    supermsg_local_dest_msgs_per_part_type.setdefault(part_name, set())
                    supermsg_local_dest_msgs_per_part_type[part_name].add(msg_name)
                for part_name in map_unique_parts(IC_DATA, src_map):
                    supermsg_src_parts[part_name] = sorted(list(set([x[1] for x in src_map if x[0] == part_name])))
                    supermsg_local_src_msgs_per_part_type.setdefault(part_name, set())
                    supermsg_local_src_msgs_per_part_type[part_name].add(msg_name)

                # Add the binds to the splitter/joiner
                for (part_name, port_ids) in map_unique_parts_ports(IC_DATA, dest_map).items():
                    add_ic_type(ic_id)
                    add_part_type(part_name)

                    if grout:
                        b_s = 12*' '
                        for port_id in port_ids:
                            inst_name_base =  "recv_{}".format(msg_name.lower())
                            inst_name = inst_name_base + "_" + str(port_id)
                            handler_instances[part_name].append( ("Connections::In", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_name)], inst_name) )
                            b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                        b_s += "{{}}".format()
                    else:
                        b_s = 12*' '
                        for port_id in port_ids:
                            inst_name = "splitterflit_{supermsg_idx}.out_port_{msg_name_lower}_{port_id}".format(supermsg_idx=supermsg_idx,
                                                                                                            msg_name_lower=msg_name.lower(),
                                                                                                            port_id=port_id)
                            b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                        b_s += "{{}}".format()

                    handler_binds[part_name].append("""\
        template <unsigned int BIND_ID>
        void Bind(Connections::InBlocking<interconnect::Msg<interconnect_config::msgs::{msg_type}> > &p, unsigned int port_id) {{
{b_s}
        }}""".format(msg_type=msg_name, inst_name=inst_name, b_s=b_s))

                # Add the binds to the splitter/joiner
                for (part_name, port_ids) in map_unique_parts_ports(IC_DATA, src_map).items():
                    add_ic_type(ic_id)
                    add_part_type(part_name)

                    if grout:
                        b_s = 12*' '
                        for port_id in port_ids:
                            inst_name_base =  "send_{}".format(msg_name.lower())
                            inst_name = inst_name_base + "_" + str(port_id)
                            handler_instances[part_name].append( ("Connections::Out", ["interconnect::Msg<interconnect_config::msgs::{}> ".format(msg_name)], inst_name) )
                            b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                        b_s += "{{}}".format()
                    else:
                        b_s = 12*' '
                        for port_id in port_ids:
                            inst_name = "joinerflit_{supermsg_idx}.in_port_{msg_name_lower}_{port_id}".format(supermsg_idx=supermsg_idx,
                                                                                                            msg_name_lower=msg_name.lower(),
                                                                                                            port_id=port_id)
                            b_s += "if(port_id == {port_id}) {{ p({inst_name}); }}\n            else ".format(port_id=port_id, inst_name=inst_name)
                        b_s += "{{}}".format()

                    handler_binds[part_name].append("""\
        template <unsigned int BIND_ID>
        void Bind(Connections::OutBlocking<interconnect::Msg<interconnect_config::msgs::{msg_type}> > &p, unsigned int port_id) {{
{b_s}
        }}""".format(msg_type=msg_name, inst_name=inst_name, b_s=b_s))


            if grout:
                __idx_offset = 0
                for part_name in sorted(supermsg_dest_parts.keys()):
                    inst_name="splitterflit_{supermsg_idx}_{part_name}".format(supermsg_idx=supermsg_idx, part_name=part_name)
                    num=len(supermsg_dest_parts[part_name])
                    ic_collate_select(ic_id, grout_instances)
                    ic_collate_add("interconnect::SplitterFlit_{part_name}_{supermsg_idx}<> *{inst_name}[{num}];"
                                                    .format(part_name=part_name,
                                                            supermsg_idx=supermsg_idx,
                                                            inst_name=inst_name,
                                                            num=num))
                    ic_collate_select(ic_id, grout_constructors_pre)
                    for i in range(num):
                        ic_collate_add("{inst_name}[{i}] = new interconnect::SplitterFlit_{part_name}_{supermsg_idx}<>();"
                                        .format(part_name=part_name,
                                                supermsg_idx=supermsg_idx,
                                                inst_name=inst_name,
                                                i=i))
                    ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=num))
                    ic_collate_add("{inst_name}[i]->clk(clk);".format(inst_name=inst_name))
                    ic_collate_add("{inst_name}[i]->rst(rst);".format(inst_name=inst_name))
                    ic_collate_add("{inst_name}[i]->in_port(routers_{supermsg_idx}_lport_link_egress_{msg_name}_retimed[i + {this_dest_idx_offset}]);"
                                    .format(inst_name=inst_name,
                                            msg_name="FLITMSG_{}".format(supermsg_idx),
                                            supermsg_idx=supermsg_idx,
                                            this_dest_idx_offset=__idx_offset))
                    if topo_options['router_module'] == 'WHVCSourceRouter':
                        ic_collate_add("{inst_name}[i]->out_credit(routers_{supermsg_idx}_lport_credit_ingress_{msg_name}[i + {this_dest_idx_offset}]);"
                                    .format(inst_name=inst_name,
                                            msg_name="FLITMSG_{}".format(supermsg_idx),
                                            supermsg_idx=supermsg_idx,
                                            this_dest_idx_offset=__idx_offset))
                    __idx_offset += num

                    for sub_msg_name in supermsg_local_dest_msgs_per_part_type[part_name]:
                        ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=num))
                        ic_collate_add("{inst_name}[i]->out_port_{msg_name_lower}_{port_id}(link_ic_recv_{part_name}_{msg_name}[i]);"
                                                    .format(inst_name=inst_name,
                                                            part_name=part_name,
                                                            msg_name=sub_msg_name,
                                                            msg_name_lower=sub_msg_name.lower(),
                                                            port_id=0))

                __idx_offset = 0
                for part_name in sorted(supermsg_src_parts.keys()):
                    inst_name="joinerflit_{supermsg_idx}_{part_name}".format(supermsg_idx=supermsg_idx, part_name=part_name)
                    num=len(supermsg_src_parts[part_name])
                    ic_collate_select(ic_id, grout_instances)
                    ic_collate_add("interconnect::JoinerFlit_{part_name}_{supermsg_idx}<> *{inst_name}[{num}];"
                                                    .format(part_name=part_name,
                                                            supermsg_idx=supermsg_idx,
                                                            inst_name=inst_name,
                                                            num=num))
                    ic_collate_select(ic_id, grout_constructors_pre)
                    for i in range(num):
                        ic_collate_add("{inst_name}[{i}] = new interconnect::JoinerFlit_{part_name}_{supermsg_idx}<>();"
                                        .format(part_name=part_name,
                                                supermsg_idx=supermsg_idx,
                                                inst_name=inst_name,
                                                i=i))
                    ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=num))
                    ic_collate_add("{inst_name}[i]->clk(clk);".format(inst_name=inst_name))
                    ic_collate_add("{inst_name}[i]->rst(rst);".format(inst_name=inst_name))
                    ic_collate_add("{inst_name}[i]->out_port(routers_{supermsg_idx}_lport_link_ingress_{msg_name}[i + {this_src_idx_offset}]);"
                                    .format(inst_name=inst_name,
                                            msg_name="FLITMSG_{}".format(supermsg_idx),
                                            supermsg_idx=supermsg_idx,
                                            this_src_idx_offset=__idx_offset))
                    if topo_options['router_module'] == 'WHVCSourceRouter':
                         ic_collate_add("{inst_name}[i]->in_credit(routers_{supermsg_idx}_lport_credit_egress_{msg_name}_retimed[i + {this_src_idx_offset}]);"
                                    .format(inst_name=inst_name,
                                            msg_name="FLITMSG_{}".format(supermsg_idx),
                                            supermsg_idx=supermsg_idx,
                                            this_src_idx_offset=__idx_offset))               
                    __idx_offset += num

                    for sub_msg_name in supermsg_local_src_msgs_per_part_type[part_name]:
                        ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=num))
                        ic_collate_add("{inst_name}[i]->in_port_{msg_name_lower}_{port_id}(link_ic_send_{part_name}_{msg_name}[i]);"
                                                        .format(inst_name=inst_name,
                                                                part_name=part_name,
                                                                msg_name=sub_msg_name,
                                                                msg_name_lower=sub_msg_name.lower(),
                                                                port_id=0))


            else:
                # Add the joiner and splittter instances, and bindings
                for part_name in sorted(supermsg_dest_parts.keys()):
                    add_ic_type(ic_id)
                    add_part_type(part_name)
                    inst_name="splitterflit_{}".format(supermsg_idx)
                    handler_instances[part_name].append( ("SplitterFlit_{part_name}_{supermsg_idx}".format(part_name=part_name, supermsg_idx=supermsg_idx), [], inst_name) )
                    handler_constructors[part_name] += ["{}.clk(clk);".format(inst_name), "{}.rst(rst);".format(inst_name)]

                for part_name in sorted(supermsg_src_parts.keys()):
                    add_ic_type(ic_id)
                    add_part_type(part_name)
                    inst_name="joinerflit_{}".format(supermsg_idx)
                    handler_instances[part_name].append( ("JoinerFlit_{part_name}_{supermsg_idx}".format(part_name=part_name, supermsg_idx=supermsg_idx), [], inst_name) )
                    handler_constructors[part_name] += ["{}.clk(clk);".format(inst_name), "{}.rst(rst);".format(inst_name)]

            # Need to pull out port_ids here
            # port_id really should be a destination list thing, not a message type thing
            # 0 should be from a global counter for super messages

            (super_msg_decls_s_, super_msg_types_s_, icg_config_msg_types_a_) = Flit().create_flit(YAML_DATA, topo_name, topo_groups, supermsg_dest_maps, supermsg_src_maps, topo_options['bus_width'], supermsg_idx)

            super_msg_decls_s += super_msg_decls_s_ + '\n'
            super_msg_types_s += super_msg_types_s_ + '\n'
            icg_config_msg_types_a += icg_config_msg_types_a_

            # Once we encode part_id as an sc_in that splitter can use, we can have the real port_id list
            # and have splitter decode dest_bits properly
            # 
            # Through a function in supermsg that given a part_id AND port_id (is_port_id(part_id, port_id)) it will return true or false.
            max_packet_size = 0
            for (part_name, local_msg_list) in supermsg_local_dest_msgs_per_part_type.items():
                (s, max_packet_size_) = Flit().create_splitter_flit(YAML_DATA, topo_options, topo_groups, part_name, supermsg_dest_maps, [(x, 0) for x in topo_groups], local_msg_list, topo_options['bus_width'], supermsg_idx, grout=grout)
                super_msg_classes_s += s + '\n\n'
                max_packet_size = max([max_packet_size, max_packet_size_])
            for (part_name, local_msg_list) in supermsg_local_src_msgs_per_part_type.items():
                (s, max_packet_size_) = Flit().create_joiner_flit(YAML_DATA, topo_options, topo_groups, part_name, supermsg_dest_maps, [(x, 0) for x in topo_groups], local_msg_list, topo_options['bus_width'], supermsg_idx, grout=grout)
                super_msg_classes_s += s + '\n\n'
                max_packet_size = max([max_packet_size, max_packet_size_])


            for msg_name in topo_groups:
                dest_map = gen_msg_dest_map(IC_DATA, msg_name)
                src_map = gen_msg_src_map(IC_DATA, msg_name)

                # Add the message type
                msg_types_s += link_gen.generate_msg_type(msg_name, dest_map)

            # Add top level connections
            msg_name = "FLITMSG_{}".format(supermsg_idx)
            num_dests = sum([len(x) for x in supermsg_dest_parts.values()])
            num_srcs = sum([len(x) for x in supermsg_src_parts.values()])

            all_parts = dict()
            for (part_name, part_insts) in supermsg_dest_parts.items():
                for part_inst in part_insts:
                    all_parts[part_inst] = part_name
            for (part_name, part_insts) in supermsg_src_parts.items():
                for part_inst in part_insts:
                    all_parts[part_inst] = part_name
            all_parts_enum = sorted(all_parts.keys()) # enumerated so we can inst id's in sequence
        
            # Determine topo areas
            topo_min_coor=[float(min(i)) for i in zip(*inst_coors.values())]
            topo_max_coor=[float(max(i)) for i in zip(*inst_coors.values())]
            topo_center_coor = [sum(i)/2.0 for i in zip(topo_min_coor, topo_max_coor)]

            # print(topo_min_coor)
            # print(topo_max_coor)
            # print(topo_center_coor)

            num_routers_x = math.floor( (topo_max_coor[0] - topo_min_coor[0])/topo_options['router_spacing'] + 1 )
            num_routers_y = math.floor( (topo_max_coor[1] - topo_min_coor[1])/topo_options['router_spacing'] + 1 )
            num_routers = num_routers_x*num_routers_y

            router_coors = list()
            router_coor_y = topo_center_coor[1] - topo_options['router_spacing']*(num_routers_y-1)/2.0
            for y_idx in range(num_routers_y):
                router_coor_x = topo_center_coor[0] - topo_options['router_spacing']*(num_routers_x-1)/2.0
                for x_idx in range(num_routers_x):
                    router_coors += [(router_coor_x, router_coor_y)]
                    # incr the coordinates
                    router_coor_x += topo_options['router_spacing']
                router_coor_y += topo_options['router_spacing']

            # Given router coors, for each inst find the closest router, and add to its lport
            router_lport_insts = dict()
            for idx in range(num_routers):
                router_lport_insts.setdefault(idx, [])

            inst_assigned_router = dict()
            for (inst_id, inst_coor) in inst_coors.items():
                # skip partition ids not in topology
                if inst_id not in all_parts:
                    continue

                # sets criteria to be closest router
                CLOSEST_ROUTER_EPSILON=0.05

                # Sort list of routers by distance
                closest_router = sorted(range(len(router_coors)), key=lambda x: manhattan_distance(router_coors[x], inst_coor))

                # Select all routers within epsilon of closet router
                closest_router = [x for x in closest_router \
                                  if manhattan_distance(router_coors[x], inst_coor) \
                                        <= (1.0+CLOSEST_ROUTER_EPSILON)*manhattan_distance(router_coors[closest_router[0]], inst_coor)]

                # sort remaining list by fewer number of lports
                closest_router = sorted(closest_router, key=lambda x: len(router_lport_insts[x]))

                # assign to router
                router_lport_insts[closest_router[0]].append(inst_id)
                inst_assigned_router[inst_id] = closest_router[0]

            # v-- taken from create_supermsg and elsewhere in code. Refactor...
            supermsg_master_src_map = sorted(list( set( sum(supermsg_src_maps.values(), list()) ) ))
            supermsg_master_dest_map = sorted(list( set( sum(supermsg_dest_maps.values(), list()) ) ))
            supermsg_master_insts = list()
            for (part_name, inst_id, port_id) in supermsg_master_dest_map:
                if inst_id not in supermsg_master_insts:
                    supermsg_master_insts.append(inst_id)

            # jtag initialization
            if topo_options['router_module'] in ['HybridRouter', 'WHVCRouter_LUTRouting']:
                ic_collate_select(ic_id, grout_constructors)
                ic_collate_add("chip_id_{supermsg_idx}_jtag.write(0);".format(supermsg_idx=supermsg_idx))

            this_src_idx_offset = 0
            this_src_dummy_offset = 0
            this_dest_idx_offset = 0
            this_dest_dummy_offset = 0
            for i in range(num_routers):
                # move these functions central at some point
                def inst_id_index_in_supermsg_map(supermsg_map, inst_id):
                    for supermsg_idx in range(len(supermsg_map)):
                        if supermsg_map[supermsg_idx][1] == inst_id:
                            return supermsg_idx
                    return None

                def rport_idx_of_dir(router_dir_mask, d):
                    idx_end = None
                    if d.lower() == 'n':
                        idx_end = 0
                    elif d.lower() == 'e':
                        idx_end = 1
                    elif d.lower() == 's':
                        idx_end = 2
                    elif d.lower() == 'w':
                        idx_end = 3
                    else:
                        assert False, "Unknown router direction {}!".format(d)
                    assert router_dir_mask[idx_end], "Router doesn't support direction {}!".format(d)
                    return sum(router_dir_mask[0:idx_end])

                # This implements XY routing
                def what_is_dir(start_coor, end_coor):
                    x_dist = end_coor[0] - start_coor[0]
                    y_dist = end_coor[1] - start_coor[1]

                    # return 'x' to mark that they are at the same spot
                    if abs(x_dist) == 0 and abs(y_dist) == 0:
                        return 'x'

                    if x_dist > 0:
                      return 'e'
                    elif x_dist < 0:
                      return 'w'
                    elif y_dist > 0:
                      return 'n'
                    else:
                      return 's'

                # Implements what_is_dir but for routing
                def what_is_dir_route_west_first(start_coor, end_coor):
                    x_dist = end_coor[0] - start_coor[0]
                    y_dist = end_coor[1] - start_coor[1]

                    # return 'x' to mark that they are at the same spot
                    if end_coor[0] <= start_coor[0] or end_coor[1] == start_coor[1]:
                        return what_is_dir(start_coor, end_coor)

                    if end_coor[1] > start_coor[1]:
                        return 'n'
                    else:
                        return 's'

                # Implements what_is_dir but for routing
                # This seems to match XY routing? Probably since we can only go w or e, not s also.
                def what_is_dir_route_north_last(start_coor, end_coor):
                    x_dist = end_coor[0] - start_coor[0]
                    y_dist = end_coor[1] - start_coor[1]

                    # return 'x' to mark that they are at the same spot
                    if end_coor[0] == start_coor[0] or end_coor[1] >= start_coor[1]:
                        return what_is_dir(start_coor, end_coor)

                    if end_coor[0] < start_coor[0]:
                        return 'w'
                    else:
                        return 'e' 

                # Implements what_is_dir but for routing
                def what_is_dir_route_negative_first(start_coor, end_coor):
                    x_dist = end_coor[0] - start_coor[0]
                    y_dist = end_coor[1] - start_coor[1]

                    # return 'x' to mark that they are at the same spot
                    if x_dist < 0:
                        return 'w'
                    elif y_dist < 0:
                        return 's'
                    elif x_dist > 0:
                        return 'e'
                    elif y_dist > 0:
                        return 'n'
                    else:
                        return 'x'

                def what_is_dir_route(start_coor, end_coor):
                    return what_is_dir(start_coor, end_coor)
                    #return what_is_dir_route_west_first(start_coor, end_coor)
                    #return what_is_dir_route_north_last(start_coor, end_coor)
                    #return what_is_dir_route_negative_first(start_coor, end_coor)

                # Find closest router from inst (inst could be a router_coor) that is in the desired direction. -1 if not found.
                def what_is_closest_router(inst_coor, d):
                    closest_router = -1
                    for i in range(len(router_coors)):
                        if what_is_dir(inst_coor, router_coors[i]) == d and (closest_router == -1 or manhattan_distance(router_coors[i], inst_coor) < manhattan_distance(router_coors[closest_router], inst_coor)):
                            closest_router = i
                    return closest_router

                router_coor = router_coors[i]

                #router_dir_mask = (0, 1, 0, 1) # mask for which of (N, E, S, W) ports are populated. Allowed values are 0 or 1 (False or True)
                router_dir_mask = (what_is_closest_router(router_coor, 'n') != -1,
                                   what_is_closest_router(router_coor, 'e') != -1,
                                   what_is_closest_router(router_coor, 's') != -1,
                                   what_is_closest_router(router_coor, 'w') != -1)

                num_l_ports = max(len(router_lport_insts[i]),1)
                num_r_ports = sum(router_dir_mask)
                num_ports = num_l_ports + num_r_ports
                maxpktsize = max_packet_size
                buffersize = maxpktsize + 8
                nop_dests = 1
                noc_dests = num_dests
                mcast_noc_dests = noc_dests
                ic_collate_select(ic_id, grout_instances)
                if topo_options['router_module'] == 'HybridRouter':
                    ic_collate_add("interconnect::HybridRouter<{num_l_ports}, {num_r_ports}, {buffersize}, interconnect::Msg<interconnect_config_icg::msgs::{msg_name}>, {nop_dests}, {noc_dests}, {mcast_noc_dests}, {maxpktsize}> router_{i}_{msg_name};".format(
                                                                                                    i=i,
                                                                                                    ic_id=ic_id,
                                                                                                    num_l_ports=num_l_ports,
                                                                                                    num_r_ports=num_r_ports,
                                                                                                    buffersize=buffersize,
                                                                                                    nop_dests=nop_dests,
                                                                                                    noc_dests=noc_dests,
                                                                                                    mcast_noc_dests=mcast_noc_dests,
                                                                                                    maxpktsize=maxpktsize,
                                                                                                    msg_name=msg_name))
                elif topo_options['router_module'] == 'WHVCRouter_LUTRouting':
                    # Note, max hops is unused for LUT routing, so we set it to -1 to indicate invalid.
                    ic_collate_add("interconnect::WHVCRouter<{num_l_ports}, {num_r_ports}, 1, {buffersize}, -1, LUTRouting, {nop_dests}, {noc_dests}, {mcast_noc_dests}, interconnect::Msg<interconnect_config_icg::msgs::{msg_name}>, {maxpktsize}> router_{i}_{msg_name};".format(
                                                                                                    i=i,
                                                                                                    ic_id=ic_id,
                                                                                                    num_l_ports=num_l_ports,
                                                                                                    num_r_ports=num_r_ports,
                                                                                                    buffersize=buffersize,
                                                                                                    nop_dests=nop_dests,
                                                                                                    noc_dests=noc_dests,
                                                                                                    mcast_noc_dests=mcast_noc_dests,
                                                                                                    maxpktsize=maxpktsize,
                                                                                                    msg_name=msg_name))
                elif topo_options['router_module'] == 'WHVCSourceRouter':
                    credit_t = "typename interconnect::WHVCSourceRouter<{num_l_ports}, {num_r_ports}, 1, {buffersize}, interconnect::Msg<interconnect_config_icg::msgs::{msg_name}>, {max_hops}>::Credit_ret_t".format(
                                                                                ic_id=ic_id,
                                                                                num_l_ports=num_l_ports,
                                                                                num_r_ports=num_r_ports,
                                                                                buffersize=buffersize,
                                                                                max_hops=noc_dests,
                                                                                msg_name=msg_name)
                    ic_collate_add("interconnect::WHVCSourceRouter<{num_l_ports}, {num_r_ports}, 1, {buffersize}, interconnect::Msg<interconnect_config_icg::msgs::{msg_name}>, {max_hops}> router_{i}_{msg_name};".format(
                                                                                i=i,
                                                                                ic_id=ic_id,
                                                                                num_l_ports=num_l_ports,
                                                                                num_r_ports=num_r_ports,
                                                                                buffersize=buffersize,
                                                                                max_hops=noc_dests,
                                                                                msg_name=msg_name))
                else:
                    sys.stderr.write("Error: Illegal router module for {}\n".format(str(topo_options['router_module'])))
                    sys.stderr.write("Exiting...\n")
                    sys.exit(-1)
                ic_collate_select(ic_id, grout_constructor_inits)
                ic_collate_add("router_{i}_{msg_name}(sc_gen_unique_name(\"router_{i}_{msg_name}\"))".format(i=i, msg_name=msg_name))

                ic_collate_select(ic_id, grout_constructors)
                ic_collate_add("router_{i}_{msg_name}.clk(clk);".format(i=i, msg_name=msg_name))
                ic_collate_add("router_{i}_{msg_name}.rst(rst);".format(i=i, msg_name=msg_name))
                if topo_options['router_module'] in ['HybridRouter', 'WHVCRouter_LUTRouting']:
                    if NOC_IS_NOP:
                        ic_collate_add("router_{i}_{msg_name}.chip_id_jtag(chip_id_{supermsg_idx}_jtag);".format(i=i, msg_name=msg_name, supermsg_idx=supermsg_idx)) # jtag
                    else:
                        ic_collate_add("router_{i}_{msg_name}.noc2_id_jtag(chip_id_{supermsg_idx}_jtag);".format(i=i, msg_name=msg_name, supermsg_idx=supermsg_idx)) # jtag

                    ic_collate_select(ic_id, grout_instances)
                    ic_collate_add("sc_signal<NVUINTW({num_ports})> router_{i}_{msg_name}_NoCRouteLUT_jtag[{noc_dests}];".format(i=i, msg_name=msg_name, supermsg_idx=supermsg_idx, num_ports=num_ports, noc_dests=noc_dests))
                    ic_collate_add("sc_signal<NVUINTW({num_ports})> router_{i}_{msg_name}_NoPRouteLUT_jtag[{nop_dests}];".format(i=i, msg_name=msg_name, supermsg_idx=supermsg_idx, num_ports=num_ports, nop_dests=nop_dests))

                    ic_collate_select(ic_id, grout_constructors)
                    ic_collate_add("for(int i=0; i < {noc_dests}; ++i) {{ router_{i}_{msg_name}.NoCRouteLUT_jtag[i]( router_{i}_{msg_name}_NoCRouteLUT_jtag[i] ); }}".format(i=i, msg_name=msg_name, supermsg_idx=supermsg_idx, noc_dests=noc_dests))
                    if NOC_IS_NOP:
                        ic_collate_add("for(int i=0; i < {nop_dests}; ++i) {{ router_{i}_{msg_name}.NoPRouteLUT_jtag[i]( router_{i}_{msg_name}_NoPRouteLUT_jtag[i] ); }}".format(i=i, msg_name=msg_name, supermsg_idx=supermsg_idx, nop_dests=nop_dests))
                    else:
                        ic_collate_add("for(int i=0; i < {nop_dests}; ++i) {{ router_{i}_{msg_name}.NoC2RouteLUT_jtag[i]( router_{i}_{msg_name}_NoPRouteLUT_jtag[i] ); }}".format(i=i, msg_name=msg_name, supermsg_idx=supermsg_idx, nop_dests=nop_dests))

                # local port
                for lport_idx in range(num_l_ports):
                    fake_l_ports = len(router_lport_insts[i]) < num_l_ports
                    if not fake_l_ports:
                        part_inst = router_lport_insts[i][lport_idx]
                        part_name = all_parts[part_inst]

                    ic_collate_select(ic_id, grout_constructors)

                    if not fake_l_ports and part_name in supermsg_src_parts.keys(): # for a given part_inst check if it is included in dests
                        ic_collate_add("router_{i}_{msg_name}.in_port[{lport_idx}](routers_{supermsg_idx}_lport_link_ingress_{msg_name}_retimed[{a_idx}]);".format(supermsg_idx=supermsg_idx,
                                                                                                                                                        i=i,
                                                                                                                                                        msg_name=msg_name,
                                                                                                                                                        lport_idx=lport_idx,
                                                                                                                                                        a_idx=inst_id_index_in_supermsg_map(supermsg_master_src_map, part_inst)))
                        if topo_options['router_module'] == 'WHVCSourceRouter':
                            ic_collate_add("router_{i}_{msg_name}.in_credit[{lport_idx}](routers_{supermsg_idx}_lport_credit_ingress_{msg_name}_retimed[{a_idx}]);".format(supermsg_idx=supermsg_idx,
                                                                                                                                    i=i,
                                                                                                                                    msg_name=msg_name,
                                                                                                                                    lport_idx=lport_idx,
                                                                                                                                    a_idx=inst_id_index_in_supermsg_map(supermsg_master_src_map, part_inst)))
                        this_src_idx_offset += 1
                    else:
                        ic_collate_add("router_{i}_{msg_name}.in_port[{lport_idx}](routers_{supermsg_idx}_lport_dummy_ingress[{a_idx}]);".format(supermsg_idx=supermsg_idx,
                                                                                                                                                        i=i,
                                                                                                                                                        msg_name=msg_name,
                                                                                                                                                        lport_idx=lport_idx,
                                                                                                                                                        a_idx=this_src_dummy_offset))
                        if topo_options['router_module'] == 'WHVCSourceRouter':
                            ic_collate_add("router_{i}_{msg_name}.in_credit[{lport_idx}](routers_{supermsg_idx}_lport_credit_dummy_ingress[{a_idx}]);".format(supermsg_idx=supermsg_idx,
                                                                                                                                                            i=i,
                                                                                                                                                            msg_name=msg_name,
                                                                                                                                                            lport_idx=lport_idx,
                                                                                                                                                            a_idx=this_src_dummy_offset))
                        this_src_dummy_offset += 1

                    if not fake_l_ports and part_name in supermsg_dest_parts.keys(): # for a given part_inst check if it is included in dests
                        ic_collate_add("router_{i}_{msg_name}.out_port[{lport_idx}](routers_{supermsg_idx}_lport_link_egress_{msg_name}[{a_idx}]);".format(supermsg_idx=supermsg_idx,
                                                                                                                                                        i=i,
                                                                                                                                                        msg_name=msg_name,
                                                                                                                                                        lport_idx=lport_idx,
                                                                                                                                                        a_idx=inst_id_index_in_supermsg_map(supermsg_master_dest_map, part_inst)))
                        if topo_options['router_module'] == 'WHVCSourceRouter':
                            ic_collate_add("router_{i}_{msg_name}.out_credit[{lport_idx}](routers_{supermsg_idx}_lport_credit_egress_{msg_name}[{a_idx}]);".format(supermsg_idx=supermsg_idx,
                                                                                                                                                            i=i,
                                                                                                                                                            msg_name=msg_name,
                                                                                                                                                            lport_idx=lport_idx,
                                                                                                                                                            a_idx=inst_id_index_in_supermsg_map(supermsg_master_dest_map, part_inst)))
                        this_dest_idx_offset += 1
                    else:
                        ic_collate_add("router_{i}_{msg_name}.out_port[{lport_idx}](routers_{supermsg_idx}_lport_dummy_egress[{a_idx}]);".format(supermsg_idx=supermsg_idx, i=i, msg_name=msg_name, lport_idx=lport_idx, a_idx=this_dest_dummy_offset))
                        if topo_options['router_module'] == 'WHVCSourceRouter':
                            ic_collate_add("router_{i}_{msg_name}.out_port[{lport_idx}](routers_{supermsg_idx}_lport_dummy_egress[{a_idx}]);".format(supermsg_idx=supermsg_idx, i=i, msg_name=msg_name, lport_idx=lport_idx, a_idx=this_dest_dummy_offset))
                        this_dest_dummy_offset += 1


                # Remote ports
                
                retiming_stages=max(math.ceil(topo_options['router_spacing']/topo_options['wire_prop_speed'] - 1), 0) # rport retiming
                extra_capacity = 0 # extra capacity to end only

                for dir_mask_idx in range(len(router_dir_mask)):
                    (d, d_named, d_opposite) = [('n', 'north', 'south'),
                                                ('e', 'east', 'west'),
                                                ('s', 'south', 'north'),
                                                ('w', 'west', 'east')][dir_mask_idx]

                    if router_dir_mask[dir_mask_idx]:
                        # create the rport links between routers
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("interconnect::RetimingStages<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}>, {retiming_stages}, {extra_capacity}> router_{i}_{msg_name}_rport_link_egress_{d_named}_retime;" \
                                                            .format(i=i,
                                                                    msg_name=msg_name,
                                                                    retiming_stages=retiming_stages,
                                                                    extra_capacity=extra_capacity,
                                                                    d_named=d_named ))

                        if topo_options['router_module'] == 'WHVCSourceRouter':
                            ic_collate_add("interconnect::RetimingStages<{credit_t}, {retiming_stages}, {extra_capacity}> router_{i}_{msg_name}_rport_credit_egress_{d_named}_retime;" \
                                                            .format(i=i,
                                                                    msg_name=msg_name,
                                                                    retiming_stages=retiming_stages,
                                                                    extra_capacity=extra_capacity,
                                                                    d_named=d_named,
                                                                    credit_t=credit_t ))

                        ic_collate_select(ic_id, grout_constructors)
                        if retiming_stages or extra_capacity:
                            ic_collate_add("router_{i}_{msg_name}_rport_link_egress_{d_named}_retime.clk(clk);"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            d_named=d_named ))
                            ic_collate_add("router_{i}_{msg_name}_rport_link_egress_{d_named}_retime.rst(rst);"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            d_named=d_named ))
                            global_wire_manifest.append({'name': 'router_{i}_{msg_name}_rport_link_egress_{d_named}'.format(i=i, msg_name=msg_name, d_named=d_named), 'src_coors': router_coor, 'dest_coors': router_coors[what_is_closest_router(router_coor, d)], 'bus_width': topo_options['bus_width'], 'wire_prop_speed': topo_options['wire_prop_speed']})
                            if topo_options['router_module'] == 'WHVCSourceRouter':
                                ic_collate_add("router_{i}_{msg_name}_rport_credit_egress_{d_named}_retime.clk(clk);"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            d_named=d_named ))
                                ic_collate_add("router_{i}_{msg_name}_rport_credit_egress_{d_named}_retime.rst(rst);"
                                                                        .format(i=i,
                                                                                msg_name=msg_name,
                                                                                d_named=d_named ))
                                global_wire_manifest.append({'name': 'router_{i}_{msg_name}_rport_credit_egress_{d_named}'.format(i=i, msg_name=msg_name, d_named=d_named), 'src_coors': router_coor, 'dest_coors': router_coors[what_is_closest_router(router_coor, d)], 'bus_width': topo_options['bus_width'], 'wire_prop_speed': topo_options['wire_prop_speed']})

                        ic_collate_add("router_{i}_{msg_name}_rport_link_egress_{d_named}_retime.in_port(router_{i}_{msg_name}_rport_link_egress_{d_named});"
                                                                .format(i=i,
                                                                        msg_name=msg_name,
                                                                        d_named=d_named ))
                        ic_collate_add("router_{i}_{msg_name}_rport_link_egress_{d_named}_retime.out_port(router_{i}_{msg_name}_rport_link_egress_{d_named}_retimed);"
                                                                .format(i=i,
                                                                        msg_name=msg_name,
                                                                        d_named=d_named ))
                        if topo_options['router_module'] == 'WHVCSourceRouter':
                            ic_collate_add("router_{i}_{msg_name}_rport_credit_egress_{d_named}_retime.in_port(router_{i}_{msg_name}_rport_credit_egress_{d_named});"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            d_named=d_named ))
                            ic_collate_add("router_{i}_{msg_name}_rport_credit_egress_{d_named}_retime.out_port(router_{i}_{msg_name}_rport_credit_egress_{d_named}_retimed);"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            d_named=d_named ))

                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > router_{i}_{msg_name}_rport_link_egress_{d_named};" \
                                                            .format(i=i,
                                                                    num_routers=num_routers,
                                                                    msg_name=msg_name,
                                                                    d_named=d_named ))
                        ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > router_{i}_{msg_name}_rport_link_egress_{d_named}_retimed;" \
                                                            .format(i=i,
                                                                    num_routers=num_routers,
                                                                    msg_name=msg_name,
                                                                    d_named=d_named ))
                        if topo_options['router_module'] == 'WHVCSourceRouter':
                            ic_collate_add("Connections::Combinational<{credit_t}> router_{i}_{msg_name}_rport_credit_egress_{d_named};" \
                                                                .format(i=i,
                                                                        num_routers=num_routers,
                                                                        msg_name=msg_name,
                                                                        d_named=d_named,
                                                                        credit_t=credit_t))
                            ic_collate_add("Connections::Combinational<{credit_t}> router_{i}_{msg_name}_rport_credit_egress_{d_named}_retimed;" \
                                                                .format(i=i,
                                                                        num_routers=num_routers,
                                                                        msg_name=msg_name,
                                                                        d_named=d_named,
                                                                        credit_t=credit_t))

                        ic_collate_select(ic_id, grout_constructors)
                        ic_collate_add("router_{i}_{msg_name}.out_port[{rport_idx}+{num_l_ports}](router_{i}_{msg_name}_rport_link_egress_{d_named});"
                                                                .format(i=i,
                                                                        msg_name=msg_name,
                                                                        d_named=d_named,
                                                                        num_l_ports=num_l_ports,
                                                                        rport_idx=rport_idx_of_dir(router_dir_mask, d) ))
                        ic_collate_add("router_{i}_{msg_name}.in_port[{rport_idx}+{num_l_ports}](router_{closet_router_idx}_{msg_name}_rport_link_egress_{d_opposite}_retimed);"
                                                                .format(i=i,
                                                                        msg_name=msg_name,
                                                                        num_l_ports=num_l_ports,
                                                                        d_opposite=d_opposite,
                                                                        closet_router_idx=what_is_closest_router(router_coor, d),
                                                                        rport_idx=rport_idx_of_dir(router_dir_mask, d) ))
                        mdist_global_wire(YAML_DATA, msg_name, router_coor, router_coors[what_is_closest_router(router_coor, d)], w=topo_options['bus_width'])
                        if topo_options['router_module'] == 'WHVCSourceRouter':
                            ic_collate_add("router_{i}_{msg_name}.out_credit[{rport_idx}+{num_l_ports}](router_{i}_{msg_name}_rport_credit_egress_{d_named});"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            d_named=d_named,
                                                                            num_l_ports=num_l_ports,
                                                                            rport_idx=rport_idx_of_dir(router_dir_mask, d) ))
                            ic_collate_add("router_{i}_{msg_name}.in_credit[{rport_idx}+{num_l_ports}](router_{closet_router_idx}_{msg_name}_rport_credit_egress_{d_opposite}_retimed);"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            num_l_ports=num_l_ports,
                                                                            d_opposite=d_opposite,
                                                                            closet_router_idx=what_is_closest_router(router_coor, d),
                                                                            rport_idx=rport_idx_of_dir(router_dir_mask, d) ))                                

                if topo_options['router_module'] in ['HybridRouter', 'WHVCRouter_LUTRouting']:
                    # NoC dests
                    port_masks_arr = []
                    for j in range(noc_dests):
                        inst_id = supermsg_master_dest_map[j][1]
                        # if part_inst in supermsg_master_insts and supermsg_master_insts.index(part_inst) == j:
                        # if this router is response for this destination...
                        if inst_id in router_lport_insts[i]:
                            port_masks_arr.append(1 << router_lport_insts[i].index(inst_id))
                        else:
                            d = what_is_dir_route(router_coor, router_coors[inst_assigned_router[inst_id]] )
                            port_masks_arr.append(1 << (rport_idx_of_dir(router_dir_mask, d)+num_l_ports))

                    ic_collate_select(ic_id, grout_constructors_pre)
                    ic_collate_add("static const unsigned int router_{i}_{msg_name}_NoCRouteLUT_jtag_lookup[{num}] = {{{port_masks_str}}};"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            num=len(port_masks_arr),
                                                                            port_masks_str=', '.join([str(x) for x in port_masks_arr])))
                    
                    ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=len(port_masks_arr)) )
                    ic_collate_add("router_{i}_{msg_name}_NoCRouteLUT_jtag[i].write(router_{i}_{msg_name}_NoCRouteLUT_jtag_lookup[i]);"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            num=len(port_masks_arr)))
                                                                            
                    # NoP dests
                    # always local for this chip... so we just set a nop rport destination of 1
                    ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=nop_dests) )
                    ic_collate_add("router_{i}_{msg_name}_NoPRouteLUT_jtag[i].write({port_mask});"
                                                                    .format(i=i,
                                                                            msg_name=msg_name,
                                                                            num=len(port_masks_arr),
                                                                            port_mask=1 << (0+num_l_ports)))

            if topo_options['router_module'] in ['HybridRouter', 'WHVCRouter_LUTRouting']:
                # create chip_id
                ic_collate_select(ic_id, grout_instances)
                ic_collate_add("sc_signal<NVUINTW(1)> chip_id_{supermsg_idx}_jtag;".format(supermsg_idx=supermsg_idx))

            # Local ports
            if this_src_idx_offset > 0:
                ic_collate_select(ic_id, grout_instances)
                if grout:
                    ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > routers_{supermsg_idx}_lport_link_ingress_{msg_name}[{num_dests}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_dests=this_src_idx_offset,
                                                    msg_name=msg_name))       
                    if topo_options['router_module'] == 'WHVCSourceRouter':
                        ic_collate_add("Connections::Combinational<{credit_t}> routers_{supermsg_idx}_lport_credit_ingress_{msg_name}[{num_dests}];" \
                                                .format(supermsg_idx=supermsg_idx,
                                                        num_routers=num_routers,
                                                        num_dests=this_src_idx_offset,
                                                        msg_name=msg_name,
                                                        credit_t=credit_t))                         
                else:
                    ic_collate_add("Connections::In<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > routers_{supermsg_idx}_lport_link_ingress_{msg_name}[{num_dests}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_dests=this_src_idx_offset,
                                                    msg_name=msg_name))
                    add_collated_combinationals("routers_{supermsg_idx}_lport_link_ingress".format(supermsg_idx=supermsg_idx), {msg_name: this_src_idx_offset}, "", "interconnect_config_icg")
                    if topo_options['router_module'] == 'WHVCSourceRouter':
                        ic_collate_add("Connections::In<{credit_t}> routers_{supermsg_idx}_lport_credit_ingress_{msg_name}[{num_dests}];" \
                                                .format(supermsg_idx=supermsg_idx,
                                                        num_routers=num_routers,
                                                        num_dests=this_src_idx_offset,
                                                        msg_name=msg_name,
                                                        credit_t=credit_t))
                        add_collated_combinationals("routers_{supermsg_idx}_lport_credit_ingress".format(supermsg_idx=supermsg_idx), {msg_name: this_src_idx_offset}, "", "interconnect_config_icg")
                ic_collate_select(ic_id, grout_instances)
                ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > routers_{supermsg_idx}_lport_link_ingress_{msg_name}_retimed[{num_dests}];" \
                                        .format(supermsg_idx=supermsg_idx,
                                                num_routers=num_routers,
                                                num_dests=this_src_idx_offset,
                                                msg_name=msg_name))
                if topo_options['router_module'] == 'WHVCSourceRouter':
                    ic_collate_add("Connections::Combinational<{credit_t}> routers_{supermsg_idx}_lport_credit_ingress_{msg_name}_retimed[{num_dests}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_dests=this_src_idx_offset,
                                                    msg_name=msg_name,
                                                    credit_t=credit_t))

                a_retiming_stages = dict()
                for dest_idx in range(num_dests):
                    inst_id = supermsg_master_dest_map[dest_idx][1]
                    mdist = mdist_global_wire(YAML_DATA, msg_name, inst_coors[inst_id], router_coors[inst_assigned_router[inst_id]], w=topo_options['bus_width'])
                    # mdist = manhattan_distance(inst_coors[inst_id], router_coors[inst_assigned_router[inst_id]])
                    retiming_stages = math.ceil(max(mdist/topo_options['wire_prop_speed'] - 1, 0)) + int(topo_options.get('extra_latency',0))
                    extra_capacity = topo_options.get('extra_capacity', 0)
                    a_retiming_stages.setdefault((msg_name, retiming_stages, extra_capacity),list())
                    a_retiming_stages[(msg_name, retiming_stages, extra_capacity)].append(dest_idx)
                    global_wire_manifest.append({'name': 'routers_{supermsg_idx}_lport_link_ingress'.format(supermsg_idx=supermsg_idx), 'src_idx': inst_id, 'dest_coors': router_coors[inst_assigned_router[inst_id]], 'bus_width': topo_options['bus_width'], 'wire_prop_speed': topo_options['wire_prop_speed']})
                    if topo_options['router_module'] == 'WHVCSourceRouter':
                        global_wire_manifest.append({'name': 'routers_{supermsg_idx}_lport_credit_ingress'.format(supermsg_idx=supermsg_idx), 'src_idx': inst_id, 'dest_coors': router_coors[inst_assigned_router[inst_id]], 'bus_width': 1, 'wire_prop_speed': topo_options['wire_prop_speed']})
                add_collated_retimings("routers_{supermsg_idx}_lport_link_ingress".format(supermsg_idx=supermsg_idx), a_retiming_stages, "interconnect_config_icg")
                if topo_options['router_module'] == 'WHVCSourceRouter':
                    add_collated_retimings("routers_{supermsg_idx}_lport_credit_ingress".format(supermsg_idx=supermsg_idx), a_retiming_stages, "interconnect_config_icg", override_ic_msg_str=credit_t)



            if this_dest_idx_offset > 0:
                ic_collate_select(ic_id, grout_instances)
                ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > routers_{supermsg_idx}_lport_link_egress_{msg_name}[{num_srcs}];" \
                                        .format(supermsg_idx=supermsg_idx,
                                                num_routers=num_routers,
                                                num_srcs=this_dest_idx_offset,
                                                msg_name=msg_name))
                if topo_options['router_module'] == 'WHVCSourceRouter':
                    ic_collate_add("Connections::Combinational<{credit_t}> routers_{supermsg_idx}_lport_credit_egress_{msg_name}[{num_srcs}];" \
                        .format(supermsg_idx=supermsg_idx,
                                num_routers=num_routers,
                                num_srcs=this_dest_idx_offset,
                                msg_name=msg_name,
                                credit_t=credit_t))
                if grout:
                    ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > routers_{supermsg_idx}_lport_link_egress_{msg_name}_retimed[{num_srcs}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_srcs=this_dest_idx_offset,
                                                    msg_name=msg_name))
                    if topo_options['router_module'] == 'WHVCSourceRouter':
                        ic_collate_add("Connections::Combinational<{credit_t}> routers_{supermsg_idx}_lport_credit_egress_{msg_name}_retimed[{num_srcs}];" \
                        .format(supermsg_idx=supermsg_idx,
                                num_routers=num_routers,
                                num_srcs=this_dest_idx_offset,
                                msg_name=msg_name,
                                credit_t=credit_t))
                else:
                    ic_collate_add("Connections::Out<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > routers_{supermsg_idx}_lport_link_egress_{msg_name}_retimed[{num_srcs}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_srcs=this_dest_idx_offset,
                                                    msg_name=msg_name))
                    add_collated_combinationals("routers_{supermsg_idx}_lport_link_egress".format(supermsg_idx=supermsg_idx), {msg_name: this_src_idx_offset}, "retimed", "interconnect_config_icg")
                    if topo_options['router_module'] == 'WHVCSourceRouter':
                        ic_collate_add("Connections::Out<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > routers_{supermsg_idx}_lport_credit_egress_{msg_name}_retimed[{num_srcs}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_srcs=this_dest_idx_offset,
                                                    msg_name=msg_name))
                        add_collated_combinationals("routers_{supermsg_idx}_lport_credit_egress".format(supermsg_idx=supermsg_idx), {msg_name: this_src_idx_offset}, "retimed", "interconnect_config_icg")

                a_retiming_stages = dict()
                for src_idx in range(num_srcs):
                    inst_id = supermsg_master_src_map[src_idx][1]
                    mdist = manhattan_distance(inst_coors[inst_id], router_coors[inst_assigned_router[inst_id]])
                    retiming_stages = math.ceil(max(mdist/topo_options['wire_prop_speed'] - 1, 0))
                    extra_capacity = 0 # extra capacity at end only
                    a_retiming_stages.setdefault((msg_name, retiming_stages, extra_capacity),list())
                    a_retiming_stages[(msg_name, retiming_stages, extra_capacity)].append(src_idx)
                    global_wire_manifest.append({'name': 'routers_{supermsg_idx}_lport_link_egress'.format(supermsg_idx=supermsg_idx), 'src_coors': router_coors[inst_assigned_router[inst_id]], 'dest_idx': inst_id, 'bus_width': topo_options['bus_width'], 'wire_prop_speed': topo_options['wire_prop_speed']})
                    if topo_options['router_module'] == 'WHVCSourceRouter':
                        global_wire_manifest.append({'name': 'routers_{supermsg_idx}_lport_credit_egress'.format(supermsg_idx=supermsg_idx), 'src_coors': router_coors[inst_assigned_router[inst_id]], 'dest_idx': inst_id, 'bus_width': 1, 'wire_prop_speed': topo_options['wire_prop_speed']})
                add_collated_retimings("routers_{supermsg_idx}_lport_link_egress".format(supermsg_idx=supermsg_idx), a_retiming_stages, "interconnect_config_icg")
                if topo_options['router_module'] == 'WHVCSourceRouter':
                    add_collated_retimings("routers_{supermsg_idx}_lport_credit_egress".format(supermsg_idx=supermsg_idx), a_retiming_stages, "interconnect_config_icg", override_ic_msg_str=credit_t)
                    
            ic_collate_select(ic_id, grout_instances)
            if this_src_dummy_offset > 0:
                # Dummy ports (for partitions that don't have a source or dest)
                ic_collate_select(ic_id, grout_instances)
                ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > routers_{supermsg_idx}_lport_dummy_ingress[{num_dests}];" \
                                        .format(supermsg_idx=supermsg_idx,
                                                num_routers=num_routers,
                                                num_dests=this_src_dummy_offset,
                                                num=num_dests,
                                                msg_name=msg_name))
                ic_collate_add("Connections::DummySource<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > *routers_{supermsg_idx}_lport_dummy_source[{num_dests}];" \
                                        .format(supermsg_idx=supermsg_idx,
                                                num_routers=num_routers,
                                                num_dests=this_src_dummy_offset,
                                                num=num_dests,
                                                msg_name=msg_name))
                ic_collate_select(ic_id, grout_constructors_pre, "for(int i=0; i < {num}; ++i)".format(num=this_src_dummy_offset) )
                ic_collate_add("routers_{supermsg_idx}_lport_dummy_source[i] = new Connections::DummySource<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> >(\"lport_dummy_source\");".format(supermsg_idx=supermsg_idx,
                                                num_routers=num_routers,
                                                msg_name=msg_name))
                ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=this_src_dummy_offset) )
                ic_collate_add("routers_{supermsg_idx}_lport_dummy_source[i]->clk(clk);".format(supermsg_idx=supermsg_idx))
                ic_collate_add("routers_{supermsg_idx}_lport_dummy_source[i]->rst(rst);".format(supermsg_idx=supermsg_idx))
                ic_collate_add("routers_{supermsg_idx}_lport_dummy_source[i]->out(routers_{supermsg_idx}_lport_dummy_ingress[i]);".format(supermsg_idx=supermsg_idx))
                
                if topo_options['router_module'] == 'WHVCSourceRouter':
                    ic_collate_select(ic_id, grout_instances)
                    ic_collate_add("Connections::Combinational<{credit_t}> routers_{supermsg_idx}_lport_credit_dummy_ingress[{num_dests}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_dests=this_src_dummy_offset,
                                                    num=num_dests,
                                                    msg_name=msg_name,
                                                    credit_t=credit_t))
                    ic_collate_add("Connections::DummySource<{credit_t}> *routers_{supermsg_idx}_lport_credit_dummy_source[{num_dests}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_dests=this_src_dummy_offset,
                                                    num=num_dests,
                                                    msg_name=msg_name,
                                                    credit_t=credit_t))
                    ic_collate_select(ic_id, grout_constructors_pre, "for(int i=0; i < {num}; ++i)".format(num=this_src_dummy_offset) )
                    ic_collate_add("routers_{supermsg_idx}_lport_credit_dummy_source[i] = new Connections::DummySource<{credit_t}>(\"credit_dummy_source\");".format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    msg_name=msg_name))
                    ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=this_src_dummy_offset) )
                    ic_collate_add("routers_{supermsg_idx}_lport_credit_dummy_source[i]->clk(clk);".format(supermsg_idx=supermsg_idx))
                    ic_collate_add("routers_{supermsg_idx}_lport_credit_dummy_source[i]->rst(rst);".format(supermsg_idx=supermsg_idx))
                    ic_collate_add("routers_{supermsg_idx}_lport_credit_dummy_source[i]->out(routers_{supermsg_idx}_lport_credit_dummy_ingress[i]);".format(supermsg_idx=supermsg_idx))
                ic_collate_select(ic_id, grout_instances)
                                   
            if this_dest_dummy_offset > 0:
                ic_collate_select(ic_id, grout_instances)
                ic_collate_add("Connections::Combinational<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > routers_{supermsg_idx}_lport_dummy_egress[{num_srcs}];" \
                                        .format(supermsg_idx=supermsg_idx,
                                                num_routers=num_routers,
                                                num_srcs=this_dest_dummy_offset,
                                                msg_name=msg_name))
                ic_collate_add("Connections::DummySink<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> > *routers_{supermsg_idx}_lport_dummy_sink[{num_srcs}];" \
                                        .format(supermsg_idx=supermsg_idx,
                                                num_routers=num_routers,
                                                num_srcs=this_dest_dummy_offset,
                                                msg_name=msg_name))
                ic_collate_select(ic_id, grout_constructors_pre, "for(int i=0; i < {num}; ++i)".format(num=this_dest_dummy_offset) )
                ic_collate_add("routers_{supermsg_idx}_lport_dummy_sink[i] = new Connections::DummySink<interconnect::Msg<interconnect_config_icg::msgs::{msg_name}> >(\"lport_dummy_sink\");".format(supermsg_idx=supermsg_idx,
                                                num_routers=num_routers,
                                                msg_name=msg_name))
                ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=this_dest_dummy_offset) )
                ic_collate_add("routers_{supermsg_idx}_lport_dummy_sink[i]->clk(clk);".format(supermsg_idx=supermsg_idx))
                ic_collate_add("routers_{supermsg_idx}_lport_dummy_sink[i]->rst(rst);".format(supermsg_idx=supermsg_idx))
                ic_collate_add("routers_{supermsg_idx}_lport_dummy_sink[i]->in(routers_{supermsg_idx}_lport_dummy_egress[i]);".format(supermsg_idx=supermsg_idx))

                if topo_options['router_module'] == 'WHVCSourceRouter':
                    ic_collate_add("Connections::Combinational<{credit_t}> routers_{supermsg_idx}_lport_credit_dummy_egress[{num_srcs}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_srcs=this_dest_dummy_offset,
                                                    msg_name=msg_name,
                                                    credit_t=credit_t))
                    ic_collate_add("Connections::DummySink<{credit_t}> *routers_{supermsg_idx}_lport_credit_dummy_sink[{num_srcs}];" \
                                            .format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    num_srcs=this_dest_dummy_offset,
                                                    msg_name=msg_name,
                                                    credit_t=credit_t))
                    ic_collate_select(ic_id, grout_constructors_pre, "for(int i=0; i < {num}; ++i)".format(num=this_dest_dummy_offset) )
                    ic_collate_add("routers_{supermsg_idx}_lport_credit_dummy_sink[i] = new Connections::DummySink<{credit_t}>(\"credit_dummy_sink\");".format(supermsg_idx=supermsg_idx,
                                                    num_routers=num_routers,
                                                    msg_name=msg_name))
                    ic_collate_select(ic_id, grout_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=this_dest_dummy_offset) )
                    ic_collate_add("routers_{supermsg_idx}_lport_credit_dummy_sink[i]->clk(clk);".format(supermsg_idx=supermsg_idx))
                    ic_collate_add("routers_{supermsg_idx}_lport_credit_dummy_sink[i]->rst(rst);".format(supermsg_idx=supermsg_idx))
                    ic_collate_add("routers_{supermsg_idx}_lport_credit_dummy_sink[i]->in(routers_{supermsg_idx}_lport_credit_dummy_egress[i]);".format(supermsg_idx=supermsg_idx))

            if num_dests > 0:
                if grout:
                    this_src_idx_offset = defaultdict(lambda: defaultdict(lambda: 0))
                else:
                    this_src_idx_offset = 0

                # This needs to accomodate port_ids in the future
                for (src_part_name, src_port_ids) in sorted([ (x, [0]) for x in supermsg_src_parts.keys() ]):
                    interface_src_binds_per_iface.setdefault(src_part_name, list())
                    if grout:
                        for src_port_id in src_port_ids:
                            for sub_msg_name in supermsg_local_src_msgs_per_part_type[src_part_name]:
                                interface_src_binds_per_iface[src_part_name].append( (
                                    "",
                                    "ic_interface.send_{msg_name_lower}_{port_id}(link_ic_send_{part_name}_{msg_name}[(this_src_idx + {this_src_idx_offset})]);"
                                        .format(msg_name=sub_msg_name,
                                                msg_name_lower=sub_msg_name.lower(),
                                                port_id=src_port_id,
                                                num_dests=len(dest_map),
                                                num_srcs=len(src_map),
                                                part_name=src_part_name,
                                                this_src_idx_offset=this_src_idx_offset[src_part_name][sub_msg_name])
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_src_idx_offset[src_part_name][sub_msg_name] += len(src_map_per_iface[src_part_name])
                    else:
                        for src_port_id in src_port_ids:
                            interface_src_binds_per_iface[src_part_name].append( (
                                "",
                                "ic_interface.joinerflit_{supermsg_idx}.out_port(routers_{supermsg_idx}_lport_link_ingress_{msg_name}[this_src_idx + {this_src_idx_offset}]);"
                                    .format(supermsg_idx=supermsg_idx, msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=src_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_src_idx_offset=this_src_idx_offset, ic_id=ic_id)
                            ) )
                            if topo_options['router_module'] == 'WHVCSourceRouter':
                                interface_src_binds_per_iface[src_part_name].append( (
                                    "",
                                    "ic_interface.joinerflit_{supermsg_idx}.in_credit(routers_{supermsg_idx}_lport_credit_egress_{msg_name}_retimd[this_src_idx + {this_src_idx_offset}]);"
                                        .format(supermsg_idx=supermsg_idx, msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=src_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_src_idx_offset=this_src_idx_offset, ic_id=ic_id)
                                ) )
                            this_src_idx_offset += len(src_map_per_iface[src_part_name])

            if num_srcs > 0:
                if grout:
                    this_dest_idx_offset = defaultdict(lambda: defaultdict(lambda: 0))
                else:
                    this_dest_idx_offset = 0

                # This needs to accomodate port_ids in the future
                for (dest_part_name, dest_port_ids) in sorted([ (x, [0]) for x in supermsg_dest_parts.keys() ]):
                    interface_dest_binds_per_iface.setdefault(dest_part_name, list())
                    if grout:
                        for dest_port_id in dest_port_ids:
                            for sub_msg_name in supermsg_local_dest_msgs_per_part_type[dest_part_name]:
                                interface_dest_binds_per_iface[dest_part_name].append( (
                                    "",
                                    "ic_interface.recv_{msg_name_lower}_{port_id}(link_ic_recv_{part_name}_{msg_name}[(this_dest_idx + {this_dest_idx_offset})]);"
                                        .format(msg_name=sub_msg_name,
                                                msg_name_lower=sub_msg_name.lower(),
                                                port_id=dest_port_id,
                                                num_dests=len(dest_map),
                                                num_srcs=len(src_map),
                                                part_name=dest_part_name,
                                                this_dest_idx_offset=this_dest_idx_offset[dest_part_name][sub_msg_name])
                                ) )
                                # Increment offset since each partition type will always be (0..num_insts_of_part - 1), so we need to reindex it across all port_id and part_types
                                this_dest_idx_offset[dest_part_name][sub_msg_name] += len(dest_map_per_iface[dest_part_name])
                    else:
                        for dest_port_id in dest_port_ids:
                            interface_dest_binds_per_iface[dest_part_name].append( (
                                "",
                                "ic_interface.splitterflit_{supermsg_idx}.in_port(routers_{supermsg_idx}_lport_link_egress_{msg_name}_retimed[this_dest_idx + {this_dest_idx_offset}]);"
                                    .format(supermsg_idx=supermsg_idx, msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=dest_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_dest_idx_offset=this_dest_idx_offset, ic_id=ic_id)
                            ) )
                            if topo_options['router_module'] == 'WHVCSourceRouter':
                                interface_dest_binds_per_iface[dest_part_name].append( (
                                    "",
                                    "ic_interface.splitterflit_{supermsg_idx}.out_credit(routers_{supermsg_idx}_lport_credit_ingress_{msg_name}[this_dest_idx + {this_dest_idx_offset}]);"
                                        .format(supermsg_idx=supermsg_idx, msg_name=msg_name, msg_name_lower=msg_name.lower(), port_id=dest_port_id, num_dests=len(dest_map), num_srcs=len(src_map), this_dest_idx_offset=this_dest_idx_offset, ic_id=ic_id)
                                ) )
                            this_dest_idx_offset += len(dest_map_per_iface[dest_part_name])
            
            if grout:
                for msg_name in topo_groups:
                    for src_part_name in this_src_idx_offset.keys():
                        if msg_name not in this_src_idx_offset[src_part_name].keys():
                            continue
                        ic_collate_select(ic_id, interconnect_instances)
                        ic_collate_add("Connections::Combinational<IC_MESSAGE({msg_name})> link_ic_send_{part_name}_{msg_name}[{num}];"
                                                                .format(msg_name=msg_name,
                                                                        part_name=src_part_name,
                                                                        num=this_src_idx_offset[src_part_name][msg_name]))
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("Connections::In<IC_MESSAGE({msg_name})> link_ic_send_{part_name}_{msg_name}[{num}];"
                                                                .format(msg_name=msg_name,
                                                                        part_name=src_part_name,
                                                                        num=this_src_idx_offset[src_part_name][msg_name]))
                            
                        assert len(supermsg_src_maps[msg_name]) == this_src_idx_offset[src_part_name][msg_name]
                        for i in range(len(supermsg_src_maps[msg_name])):
                            report_pins_of_inst("link_ic_send_{part_name}_{msg_name}[{i}]".format(msg_name=msg_name,
                                                                                                    part_name=src_part_name,
                                                                                                    num=this_src_idx_offset[src_part_name][msg_name],
                                                                                                    i=i), supermsg_src_maps[msg_name][i][1], inst_name_to_part_id)

                        ic_collate_select(ic_id, interconnect_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=this_src_idx_offset[src_part_name][msg_name]) )
                        ic_collate_add("grout_{ic_id}.{base_name}_{part_name}_{msg_name}{postfix}[i]({base_name}_{part_name}_{msg_name}{postfix}[i]);"
                                                                .format(base_name="link_ic_send",
                                                                        msg_name=msg_name,
                                                                        part_name=src_part_name,
                                                                        postfix="",
                                                                        ic_id=ic_id))
                    for dest_part_name in this_dest_idx_offset.keys():
                        if msg_name not in this_dest_idx_offset[dest_part_name].keys():
                            continue
                        ic_collate_select(ic_id, interconnect_instances)
                        ic_collate_add("Connections::Combinational<IC_MESSAGE({msg_name})> link_ic_recv_{part_name}_{msg_name}[{num}];"
                                                                .format(msg_name=msg_name,
                                                                        part_name=dest_part_name,
                                                                        num=this_dest_idx_offset[dest_part_name][msg_name]))
                        ic_collate_select(ic_id, grout_instances)
                        ic_collate_add("Connections::Out<IC_MESSAGE({msg_name})> link_ic_recv_{part_name}_{msg_name}[{num}];"
                                                                .format(msg_name=msg_name,
                                                                part_name=dest_part_name,
                                                                num=this_dest_idx_offset[dest_part_name][msg_name]))

                        assert len(supermsg_dest_maps[msg_name]) == this_dest_idx_offset[dest_part_name][msg_name]
                        for i in range(len(supermsg_dest_maps[msg_name])):
                            report_pins_of_inst("link_ic_recv_{part_name}_{msg_name}[{i}]".format(msg_name=msg_name,
                                                                                                    part_name=dest_part_name,
                                                                                                    num=this_dest_idx_offset[dest_part_name][msg_name],
                                                                                                    i=i), supermsg_dest_maps[msg_name][i][1], inst_name_to_part_id)

                        ic_collate_select(ic_id, interconnect_constructors_post, "for(int i=0; i < {num}; ++i)".format(num=this_dest_idx_offset[dest_part_name][msg_name]) )
                        ic_collate_add("grout_{ic_id}.{base_name}_{part_name}_{msg_name}{postfix}[i]({base_name}_{part_name}_{msg_name}{postfix}[i]);"
                                                                .format(base_name="link_ic_recv",
                                                                        msg_name=msg_name,
                                                                        part_name=dest_part_name,
                                                                        postfix="",
                                                                        ic_id=ic_id))

            supermsg_idx += 1

    # General topology-agnostic section
    for src_part_name in src_map_per_iface:
        a = list()
        a_part_id_ = list()
        for idx in range(len(src_map_per_iface[src_part_name])):
            a_part_id_.append(src_map_per_iface[src_part_name][idx])
        for (range_start, range_end) in to_ranges(a_part_id_):
            a.append("if(part_id_ >= {range_start} && part_id_ <= {range_end}) {{ this_src_idx=part_id_ - {idx_offset}; }}".format(range_start=range_start, range_end=range_end, idx_offset=range_start-a_part_id_.index(range_start)))
        s = "unsigned int this_src_idx=0;\n          " + ' else\n          '.join(a) + '\n'
        ic_add(ic_id, interface_binds, (
            src_part_name,
            src_map_per_iface[src_part_name],
            s
        ) )
                                                                               
    for dest_part_name in dest_map_per_iface:
        a = list()
        a_part_id_ = list()
        for idx in range(len(dest_map_per_iface[dest_part_name])):
            a_part_id_.append(dest_map_per_iface[dest_part_name][idx])
        for (range_start, range_end) in to_ranges(a_part_id_):
            a.append("if(part_id_ >= {range_start} && part_id_ <= {range_end}) {{ this_dest_idx=part_id_ - {idx_offset}; }}".format(range_start=range_start, range_end=range_end, idx_offset=range_start-a_part_id_.index(range_start)))
        s = "unsigned int this_dest_idx=0;\n          " + ' else\n           '.join(a) + '\n'
        ic_add(ic_id, interface_binds, (
            dest_part_name,
            dest_map_per_iface[dest_part_name],
            s
        ) )

    for src_part_name in src_map_per_iface:
        if src_part_name in interface_src_binds_per_iface:
            for block_condition in set( [x[0] for x in interface_src_binds_per_iface[src_part_name]] ):
                if block_condition == "":
                    for block_body in [x[1] for x in interface_src_binds_per_iface[src_part_name] if x[0] == block_condition]:
                        ic_add(ic_id, interface_binds, (
                            src_part_name,
                            src_map_per_iface[src_part_name],
                            block_body
                        ) )
                else:
                    ic_add(ic_id, interface_binds, (
                        src_part_name,
                        src_map_per_iface[src_part_name],
                        "{block_condition} {{\n{block_entries}\n            }}\n".format(block_condition=block_condition, block_entries='\n'.join(["                {}".format(x[1]) for x in interface_src_binds_per_iface[src_part_name] if x[0] == block_condition]))
                    ) )

    for dest_part_name in dest_map_per_iface:
        if dest_part_name in interface_dest_binds_per_iface:
            for block_condition in set( [x[0] for x in interface_dest_binds_per_iface[dest_part_name]] ):
                if block_condition == "":
                    for block_body in [x[1] for x in interface_dest_binds_per_iface[dest_part_name] if x[0] == block_condition]:
                        ic_add(ic_id, interface_binds, (
                            dest_part_name,
                            dest_map_per_iface[dest_part_name],
                            block_body
                        ) )
                else:
                    ic_add(ic_id, interface_binds, (
                        dest_part_name,
                        dest_map_per_iface[dest_part_name],
                        "{block_condition} {{\n{block_entries}\n            }}\n".format(block_condition=block_condition, block_entries='\n'.join(["                {}".format(x[1]) for x in interface_dest_binds_per_iface[dest_part_name] if x[0] == block_condition]))
                    ) )


    # Add all the collated interconnect constructor statements
    ic_collate_recurse(ic_id, interconnect_constructors)
    ic_collate_recurse(ic_id, interconnect_constructors_pre, interconnect_constructors)
    ic_collate_recurse(ic_id, interconnect_constructors_post, interconnect_constructors)
    ic_collate_recurse(ic_id, interconnect_constructor_inits)
    ic_collate_recurse(ic_id, interconnect_instances)

    ic_collate_recurse(ic_id, grout_constructors)
    ic_collate_recurse(ic_id, grout_constructors_pre, grout_constructors)
    ic_collate_recurse(ic_id, grout_constructors_post, grout_constructors)
    ic_collate_recurse(ic_id, grout_constructor_inits)
    ic_collate_recurse(ic_id, grout_instances)

    print("""\
// Begin interconnect_gen.hpp
#if !defined(__INTERCONNECT_GEN_TOP_HPP__)
#define __INTERCONNECT_GEN_TOP_HPP__

#include <interconnect/Interconnect.hpp>
#include <interconnect/interconnect_gen_utils.hpp>
#include "../interconnect_config.hpp"

namespace interconnect {
    """, file=f)

    print(4*' ' + "/////////// Interconnect Messages ///////////////", file=f)
    print(msg_types_s, file=f)

    print(4*' ' + "/////////// Super Message Declarations ///////////////", file=f)
    print(file=f)
    print(super_msg_decls_s, file=f)

    print(4*' ' + "/////////// ICG Config ///////////////", file=f)

    s_icg_msg_types = '\n'.join(['        IC_ADD_MESSAGE_TYPE({x0},interconnect::{x1})'.format(x0=x[0], x1=x[1]) for x in icg_config_msg_types_a])

    s_icg_config = """\
}}; // namespace interconect

    namespace interconnect_config_icg {{
    IC_BEGIN_DEST_MAPS
    IC_END_DEST_MAPS

    IC_BEGIN_MESSAGE_TYPES
{s_icg_msg_types}
    IC_END_MESSAGE_TYPES

    }};

namespace interconnect {{
        """.format(s_icg_msg_types=s_icg_msg_types)

    print(s_icg_config, file=f)


    print(4*' ' + "/////////// Super Message Types ///////////////", file=f)
    print(file=f)
    print(super_msg_types_s, file=f)
    
    print(4*' ' + "/////////// Additional Classes ///////////////", file=f)

    print(super_msg_classes_s, file=f)

    print(4*' ' + "/////////// Interconnect Interfaces ///////////////", file=f)
    for part_type in part_types:
        print(link_gen.generate_ic_interface(part_type,
                                             handler_instances[part_type],
                                             handler_constructors[part_type],
                                             handler_binds[part_type],
                                             handler_inits[part_type]), file=f)

    print(4*' ' + "/////////// Interconnect ///////////////", file=f)
    for ic_type in ic_types:
        print(generate_interconnect(ic_type,
                                    interconnect_instances[ic_type],
                                    interface_binds[ic_type],
                                    interconnect_constructors[ic_type],
                                    interconnect_constructor_inits[ic_type],
                                    grout_instances[ic_type],
                                    grout_constructors[ic_type],
                                    grout_constructor_inits[ic_type]), file=f)
        print(file=f)

    print("""
#endif 

#if !defined(IC_GEN_TOP_ONLY) && !defined(__INTERCONNECT_GEN_BOT_BIND_HPP__)
#define __INTERCONNECT_GEN_BOT_BIND_HPP__

}};

namespace interconnect {{
    """.format(), file=f)

    print(4*' ' + "/////////// IC -> Interface Binds ///////////////", file=f)
    for ic_type in ic_types:
        print(gen_ic_iface_bind(ic_type), file=f)

    print("""\
}}; // namespace interconnect

#endif

// End interconnect_gen.hpp
    """.format(), file=f)

    (max_w, max_w_x_coor) = global_wire_max_tracks_x()

    # v-- add in repeater signals
    dict_wps=dict()
    find_wps_lines=list()
    for item in global_wire_manifest:
        wps = item['wire_prop_speed']
        if wps not in dict_wps.keys():
            dict_wps[wps] = int(max(list(dict_wps.values()) + [0])) + 1
        # Skip same source/dest location.
        if 'src_coors' not in item:
            if 'src_idx' not in item or item['src_idx'] not in inst_coors:
                sys.stderr.write("Couldn't find coordinates for {}\n".format(str(item)))
                sys.stderr.write("inst_coors is {}\n".format(str(inst_coors.keys())))
                sys.stderr.write("Exiting...\n")
                sys.exit(-1)
            item['src_coors'] = inst_coors[item['src_idx']]
        if 'dest_coors' not in item:
            if 'dest_idx' not in item or item['dest_idx'] not in inst_coors:
                sys.stderr.write("Couldn't find coordinates for {}\n".format(str(item)))
                sys.stderr.write("inst_coors is {}\n".format(str(inst_coors.keys())))
                sys.stderr.write("Exiting...\n")
                sys.exit(-1)
            item['dest_coors'] = inst_coors[item['dest_idx']]
        if item['src_coors'] == item['dest_coors']:
            continue

    # Reporting
    print("Info: Max Horizontal Wire Tracks: " + str(max_w) + " at X=" + str(max_w_x_coor))

    # Also write to file for use by downstream tools
    gen_rpt_dict = {'max_horizontal_wire_tracks': max_w}
    if '__pins_to_insts' in globals():
        global __pins_to_insts
        gen_rpt_dict.update({'pins_to_insts': __pins_to_insts})
    json.dump(gen_rpt_dict, rpt)


if __name__ == "__main__":
    sys.stderr.write("Pleae use interconect_designer.py generate instead! This is an internal module.")
    sys.exit(-1)

