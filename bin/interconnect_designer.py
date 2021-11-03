#!/usr/bin/env python3
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

import os
import sys
import json
import argparse
import re
import math
from functools import reduce
import yaml
import tempfile

# Generation code lives in interconnect_generate.py
from interconnect_generate import *

# Crossbar vs. Network-on-Chip Threshold
CROSSBAR_THRESHOLD = 4
NOC_THRESHOLD = 8

##############################################################

class ConnStats:
    # Statistics
    """Statistics on connections"""
    class Stat:
        def __init__(self, group, group_addressable):
            self.n = len(group)
            num_addressable = [len(x) for x in group.values()]
            self.min = min(num_addressable)
            self.max = max(num_addressable)
            self.avg = reduce(lambda a, b: a + b,
                              num_addressable) / len(num_addressable)
            self.spread = self.avg / len(group_addressable)

    def __init__(self, group_srcs, group_dests):
        self.src = self.Stat(group_srcs, group_dests)
        self.dest = self.Stat(group_dests, group_srcs)

    def get_conn_type(self):
        conn_stats = self
        conn_type = "Unknown"
        if (conn_stats.dest.min == conn_stats.dest.max == 1) and (conn_stats.src.min == conn_stats.src.max == 1):
            conn_type = "1:1"
        elif (conn_stats.dest.min == conn_stats.dest.max == 1) and (conn_stats.dest.n > 1):
            conn_type = "1:N"
        elif (conn_stats.src.min == conn_stats.src.max == 1) and (conn_stats.src.n > 1):
            conn_type = "N:1"
        elif (conn_stats.src.min > 1) and (conn_stats.src.max > 1) and (conn_stats.dest.min > 1) and (conn_stats.dest.max > 1):
            conn_type = "N:N"
        return conn_type

    def get_rec_topology(self):
        """Get the recommended topology"""
        conn_type = self.get_conn_type()

        if conn_type == "1:1":
            rec_topo_name = "Point-to-Point"
            rec_topo_type = "link"
        elif conn_type == "1:N":
            rec_topo_name = "1:N Broadcast Bus"
            rec_topo_type = "bus"
        elif conn_type == "N:1":
            rec_topo_name = "N:1 Arbiter"
            rec_topo_type = "link"
        elif conn_type == "N:N":
            rec_topo_name = "Point-to-Point"
            rec_topo_type = "link"
            if (self.src.n >= CROSSBAR_THRESHOLD) or (self.dest.n > CROSSBAR_THRESHOLD):
                rec_topo_name = "Crossbar"
                rec_topo_type = "crossbar"
            if (self.src.n >= NOC_THRESHOLD) or (self.dest.n >= NOC_THRESHOLD):
                rec_topo_name = "Network-on-Chip"
                rec_topo_type = "noc"
        else:
            rec_topo_name = "Unknown"
            rec_topo_type = "link"

        return ("Point-to-Point", "link")
#        return (rec_topo_name, rec_topo_type)

def create_report(f):
    print("# Source Units: ", file=f)
    for src_unit_name in natsorted(srcs.keys()):
        print("  - %s contains ports (N=%i):" %
              (src_unit_name, len(srcs[src_unit_name])), file=f)
        for src_port_name in natsorted(srcs[src_unit_name].keys()):
            width = srcs[src_unit_name][src_port_name]['width']
            typeid = srcs[src_unit_name][src_port_name]['typeid']
            user_typeid = srcs[src_unit_name][src_port_name]['user_typeid']
            user_typeid_str = str(user_typeid)
            if user_typeid in user_typeids:
                user_typeid_str += " (%s)" % user_typeids[user_typeid]
            print("      - %s (WIDTH = %i, USER_TYPEID = %s)" %
                  (src_port_name, width, user_typeid_str), file=f)
        print(file=f)
    print(file=f)

    print("# Destination Units: ", file=f)
    for dest_unit_name in natsorted(dests.keys()):
        print("  - %s contains ports (N=%i):" %
              (dest_unit_name, len(dests[dest_unit_name])), file=f)
        for dest_port_name in natsorted(dests[dest_unit_name].keys()):
            width = dests[dest_unit_name][dest_port_name]['width']
            typeid = dests[dest_unit_name][dest_port_name]['typeid']
            user_typeid = dests[dest_unit_name][dest_port_name]['user_typeid']
            user_typeid_str = str(user_typeid)
            if user_typeid in user_typeids:
                user_typeid_str += " (%s)" % user_typeids[user_typeid]
            print("      - %s (WIDTH = %i, USER_TYPEID = %s)" %
                  (dest_port_name, width, user_typeid_str), file=f)
        print(file=f)
    print(file=f)

    print("# Groups: ", file=f)
    print(file=f)
    for group_idx, group in sorted(groups.items()):
        width, user_typeid, typeid, msg_name = group_idx
        user_typeid_str = str(user_typeid)
        if user_typeid in user_typeids:
            user_typeid_str += " (%s)" % user_typeids[user_typeid]

        group_srcs = group['srcs']
        group_dests = group['dests']

        print("## WIDTH = %i, USER_TYPEID = %s:" %
              (width, user_typeid_str), file=f)

        conn_stats = ConnStats(group['srcs'], group['dests'])
        conn_type = conn_stats.get_conn_type()
        (recommended_topology, recommended_type) = conn_stats.get_rec_topology()

        print(file=f)
        print("  Connection pattern: %s" % conn_type, file=f)
        print("  Recommended topology: %s" % recommended_topology, file=f)
        print("  Srcs  -->  N=(Min Max Avg Spread): %i %i %.1f %.1f" %
              (conn_stats.src.min, conn_stats.src.max, conn_stats.src.avg, conn_stats.src.spread), file=f)
        print("  Dests <--  N=(Min Max Avg Spread): %i %i %.1f %.1f" %
              (conn_stats.dest.min, conn_stats.dest.max, conn_stats.dest.avg, conn_stats.dest.spread), file=f)
        print(file=f)

        print("  ### Sources (N=%i): " % len(group_srcs), file=f)
        for src_name in natsorted(group_srcs.keys()):
            if(len(group_srcs[src_name]) == len(group_dests)):
                print("    - %s --> ALL" % src_name, file=f)
            else:
                print("    - %s --> N=%i" % (src_name,
                                             len(group_srcs[src_name])), file=f)
        print(file=f)
        print("  ### Destinations (N=%i): " % len(group_dests), file=f)
        for dest_name in natsorted(group_dests.keys()):
            if(len(group_dests[dest_name]) == len(group_srcs)):
                print("    - %s <-- ALL" % dest_name, file=f)
            else:
                print("    - %s <-- N=%i" % (dest_name,
                                             len(group_dests[dest_name])), file=f)
        print(file=f)
    print(file=f)
    print(file=f)


def default_topology_options(YAML_DATA, t, n=None, override=False):
    if n == None:
        n = t
    n = n.lower()
    t = t.lower()
    if override or (n not in YAML_DATA['topologies']):
        YAML_DATA['topologies'][n] = {'type': t, 'groups': [], 'options': {}}
        if t == 'link':
            YAML_DATA['topologies'][n]['options'] = {'wire_prop_speed': 1}
        elif t == 'crossbar':
            YAML_DATA['topologies'][n]['options'] = {
                'bus_width': 64, 'wire_prop_speed': 1, 'crossbar_x_coor': 0, 'crossbar_y_coor': 0, 'crossbar_latency': 2}
        elif t == 'noc':
            YAML_DATA['topologies'][n]['options'] = {
                'bus_width': 64, 'wire_prop_speed': 1, 'router_spacing': 1, 'router_latency': 1, 'router_module': 'HybridRouter'}
        else:
            assert False, "Unhandled topology type %s!" % t
    return YAML_DATA['topologies'][n]


def default_unit_options(YAML_DATA, unit_name, override=False):
    if override or (unit_name not in YAML_DATA['units']):
        YAML_DATA['units'][unit_name] = {
            'x_coor': 0, 'y_coor': 0}


def init_yaml_data():
    YAML_DATA = dict()
    YAML_DATA['units'] = dict()
    units = list(set(list(srcs.keys()) + list(dests.keys())))
    for unit_name in units:
        default_unit_options(YAML_DATA, unit_name)

    # Groups and their associated topolgies
    YAML_DATA['groups'] = dict()
    YAML_DATA['topologies'] = dict()
    
    # Add crossbar, noc and wire sections by default, even without associated groups.
    default_topology_options(YAML_DATA, 'crossbar')
    default_topology_options(YAML_DATA, 'noc')
    default_topology_options(YAML_DATA, 'link')
    
    for group_idx, group in sorted(groups.items()):
        width, user_typeid, typeid, msg_name = group_idx

        conn_stats = ConnStats(group['srcs'], group['dests'])
        conn_type = conn_stats.get_conn_type()
        recommended_type = conn_stats.get_rec_topology()[1]

        YAML_DATA['groups'][user_typeid] = {'_width': width,
                                            '_srcs': conn_stats.src.n,
                                            '_dests': conn_stats.dest.n,
                                            '_conn_type': conn_type}
        if user_typeid not in user_typeids:
            user_typeids[user_typeid] = msg_name
        YAML_DATA['groups'][user_typeid]['name'] = msg_name
        user_typeid_ref = msg_name

        # Add topology based on type
        topo = default_topology_options(YAML_DATA, recommended_type)
        if user_typeid_ref not in topo['groups']:
            topo['groups'].append(user_typeid_ref)

    return YAML_DATA


def create_yaml_template(f):
    YAML_DATA = init_yaml_data()
    print(yaml.dump(YAML_DATA), file=f)


def read_yaml(f):
    return yaml.load(f)

def split_unit_port_name(s):
    inst_names = list()
    for part_data in IC_DATA["parts"].values():
        for inst_name in part_data["inst_names"]:
            inst_names.append(inst_name)

    matching_inst_name = None
    matching_port_name = None
    for inst_name in inst_names:
        m = re.match(r"^{}\.(.+)$".format(re.escape(inst_name)), s)
        if m:
            if not matching_inst_name or len(inst_name) > len(matching_inst_name):
                matching_inst_name = inst_name
                matching_port_name = m.group(1)

    assert matching_inst_name, "Couldn't find a unit in connectivity JSON for connection {}!".format(s)
    return (matching_inst_name, matching_port_name)


def get_group_idx_from_channel(channel):
    return (channel["width"], channel["user_typeid"], channel["typeid"], channel["msg_name"])


def read_connectivity_json(f):
    global srcs, dests, groups, user_typeids, dom
    global IC_DATA
    dom = json.loads(f.read())

    srcs = dict()
    dests = dict()
    groups = dict()
    IC_DATA = dict()

    user_typeids = dict()

    IC_DATA["ic_id"] = dom["ic_id"]
    IC_DATA["msgs"] = dom["msgs"]
    IC_DATA["parts"] = dom["parts"]

    for channel in dom["channels"].values():
        src_name = channel["src_name"]
        dest_name = channel["dest_name"]

        if "user_typeid" not in channel:
            channel["user_typeid"] = -1

        src_unit_name, src_port_name = split_unit_port_name(src_name)
        dest_unit_name, dest_port_name = split_unit_port_name(dest_name)

        if src_unit_name not in srcs:
            srcs[src_unit_name] = dict()
        if src_port_name not in srcs[src_unit_name]:
            srcs[src_unit_name][src_port_name] = dict()
        srcs[src_unit_name][src_port_name]['width'] = channel["width"]
        srcs[src_unit_name][src_port_name]['user_typeid'] = channel["user_typeid"]
        srcs[src_unit_name][src_port_name]['typeid'] = channel["typeid"]

        if dest_unit_name not in dests:
            dests[dest_unit_name] = dict()
        if dest_port_name not in dests[dest_unit_name]:
            dests[dest_unit_name][dest_port_name] = dict()
        dests[dest_unit_name][dest_port_name]['width'] = channel["width"]
        dests[dest_unit_name][dest_port_name]['user_typeid'] = channel["user_typeid"]
        dests[dest_unit_name][dest_port_name]['typeid'] = channel["typeid"]

        group_idx = get_group_idx_from_channel(channel)
        if group_idx not in groups:
            groups[group_idx] = dict()
            groups[group_idx]['srcs'] = dict()
            groups[group_idx]['dests'] = dict()

        group_srcs = groups[group_idx]['srcs']
        group_dests = groups[group_idx]['dests']

        # Add the source to the group
        if src_name not in group_srcs:
            group_srcs[src_name] = dict()
        if dest_name not in group_srcs[src_name]:
            group_srcs[src_name][dest_name] = dict()

        # Add the destination to the group
        if dest_name not in group_dests:
            group_dests[dest_name] = dict()
        if src_name not in group_dests[dest_name]:
            group_dests[dest_name][src_name] = dict()


# Update dom based on yaml data.
def update_dom(YAML_DATA):
    global dom

    ic_q = InterconnectQuery(YAML_DATA)

    for channel_name, channel in dom["channels"].items():
        channel["latency"], channel["capacity"] = ic_q.query_latency_capacity(
            channel_name, channel)

    links = dom.setdefault('links', list())
    for key, link_group in ic_q.link_groups.items():
        a = list()
        for channel_name, channel_data in link_group.items():
            new_channel_data = {"channel_name": channel_name}
            new_channel_data.update(channel_data)
            a.append(new_channel_data)
        links.append(a)

    print("Latency ranged from %i to %i" %
          (ic_q.min_latency, ic_q.max_latency))
    print("Capacity varied from %i to %i" %
          (ic_q.min_capacity, ic_q.max_capacity))

def write_annotation_json(f):
    global dom
    print(json.dumps(dom, indent=4, sort_keys=True), file=f)


class InterconnectQuery:
    def __init__(self, YAML_DATA):
        self.missing_groups = list()
        self.YAML_DATA = YAML_DATA
        self.min_latency = sys.maxsize
        self.max_latency = -sys.maxsize
        self.min_capacity = sys.maxsize
        self.max_capacity = -sys.maxsize
        self.link_groups = dict()

    def query_latency_capacity(self, channel_name, channel):
        global dom
        global groups

        src_name = channel["src_name"]
        dest_name = channel["dest_name"]
        group_idx = get_group_idx_from_channel(channel)
        width, user_typeid, typeid, msg_name = group_idx

        group_srcs = groups[group_idx]['srcs']
        group_dests = groups[group_idx]['dests']

        src_unit_name, src_port_name = split_unit_port_name(src_name)
        dest_unit_name, dest_port_name = split_unit_port_name(dest_name)

        # Get the group name, or if it doesn't exist use the user_typeid itself.
        group_options = self.YAML_DATA['groups'][user_typeid]
        group_name = group_options.get('name', user_typeid)

        # Find topology assignment
        topo_name = None
        topo_data = None
        for topo_name_, topo_data_ in self.YAML_DATA['topologies'].items():
            if group_name in topo_data_['groups'] or user_typeid in topo_data_['groups']:
                topo_name, topo_data = topo_name_, topo_data_
                break
        if (topo_name == None or topo_data == None) and user_typeid not in self.missing_groups:
            print("Warning: Didn't find a topology for group %s! Please add to yaml. Defaulting to a link." %
                  group_name, file=sys.stderr)
            self.missing_groups.append(user_typeid)
            return (0, 0)

        # Get topology information.
        topo_type = topo_data['type'].lower()
        topo_options = topo_data['options']

        src_options = YAML_DATA['units'][src_unit_name]
        dest_options = YAML_DATA['units'][dest_unit_name]

        # Determine manhattan distance
        src_coor = (src_options['x_coor'],
                    src_options['y_coor'])
        dest_coor = (dest_options['x_coor'],
                     dest_options['y_coor'])

        m_dist = manhattan_distance(src_coor, dest_coor)

        if topo_options.get('fifo_cluter_spacing', False):
            if topo_options.get('fifos_per_cluster', False):
                fifos_per_cluster = topo_options['fifos_per_cluster']
            extra_capacity = fifos_per_cluster*m_dist / \
                topo_options['fifo_cluster_spacing']
        else:
            extra_capacity = 0
        
        extra_latency = 0

        extra_capacity += topo_options.get('extra_capacity', 0)
        extra_latency += topo_options.get('extra_latency', 0)

        extra_capacity += topo_options.get('extra_capacity_model_only', 0)
        extra_latency += topo_options.get('extra_latency_model_only', 0)

        bits_needed = width
        bits_for_dest = math.ceil(math.log(len(group_dests), 2)) if topo_options.get(
            'encode_dest_bits', True) else 0
        bits_for_src = math.ceil(math.log(len(group_srcs), 2)) if topo_options.get(
            'encode_src_bits', False) else 0
        bits_needed += bits_for_dest + bits_for_src
        bits_needed -= topo_options.get('exclude_bits', 0) + \
            group_options.get('exclude_bits', 0)
        assert bits_needed > 0, "Bits needed underflowed to %i in group %s! Original width is %i." % (
            bits_needed, group_name, width)

        if topo_options.get('bus_width', False):
            serdes_derate = max(1, math.ceil(
                bits_needed/topo_options['bus_width']))
        else:
            serdes_derate = 1

        # -1 since we already include one in Interconnect Modeler itself.
        serdes_latency = serdes_derate if serdes_derate > 1 else 0
        serdes_capacity = 2 if serdes_derate > 1 else 0

        # Switched based on topo type
        link_group_keys_to_add = set()
        link_group_bw_prob = 1.0
        link_group_weight = float(serdes_derate)

        if topo_type == 'link':
            # Calculate number of retiming stages needed
            if topo_options.get('wire_prop_speed', 0) > 0:
                wire_latency = math.ceil(
                    max(0, m_dist/topo_options['wire_prop_speed'] - 1))
            else:
                wire_latency = 0

            # Purely retimed latency
            channel_latency = wire_latency
            channel_capacity = wire_latency
            # Shouldn't be any serdes latency or capacity in this mode
            serdes_latency = 0
            serdes_capacity = 0
            serdes_derate = 1

        elif topo_type == 'noc':
            # Compute the latency of the router itself
            num_routers = m_dist/topo_options['router_spacing'] + 1
            router_latency = num_routers*topo_options['router_latency']

            # Calculate wire latency as amount of retiming needed between routers
            if topo_options.get('wire_prop_speed', 0) > 0:
                wire_latency = math.ceil(max(
                    0, topo_options['router_spacing']/topo_options['wire_prop_speed'] - 1))*(num_routers - 1)
            else:
                wire_latency = 0
            # sanity check wire_latency is not negative
            assert(wire_latency >= 0)

            channel_latency = wire_latency + router_latency
            # derate capacity based on serdes_derating
            channel_capacity = channel_latency/serdes_derate

            # Calculate all the link groups that this hits.
            # Assume X then Y routing.
            def to_router_coor(c):
                return (int(round(c[0]/topo_options['router_spacing'], 0)), int(round(c[1]/topo_options['router_spacing'], 0)))
            src_coor_router = to_router_coor(src_coor)
            dest_coor_router = to_router_coor(dest_coor)

            # Get the X segments to add
            if(src_coor_router[0] < dest_coor_router[0]):
                a = range(src_coor_router[0], dest_coor_router[0],1)
                direction = 'E'
            else:
                a = range(src_coor_router[0], dest_coor_router[0],-1)
                direction = 'W'
            for i in a:
                current_coor = (i, src_coor_router[1])
                current_linear_dist = float(manhattan_distance(current_coor, src_coor_router))
                link_group_keys_to_add.add( (topo_type, topo_name, current_coor,  direction, 1/(current_linear_dist+1.0)))

            # Get the Y segments to add
            if(src_coor_router[1] < dest_coor_router[1]):
                a = range(src_coor_router[1], dest_coor_router[1],1)
                direction = 'N'
            else:
                a = range(src_coor_router[1], dest_coor_router[1],-1)
                direction = 'S'
            for i in a:
                current_coor = (dest_coor_router[0], i)
                current_linear_dist = float(manhattan_distance(current_coor, src_coor_router))
                link_group_keys_to_add.add( (topo_type, topo_name, current_coor,  direction, 1/(current_linear_dist+1.0)))

        elif topo_type == 'crossbar':
            # Compute mdist and wire latency based on crossbar location
            crossbar_coor = (
                topo_options['crossbar_x_coor'], topo_options['crossbar_y_coor'])
            m_dist=manhattan_distance(
                src_coor, crossbar_coor) + manhattan_distance(dest_coor, crossbar_coor)
            if topo_options.get('wire_prop_speed', 0) > 0:
                wire_latency=math.ceil(
                    max(0, m_dist/topo_options['wire_prop_speed'] - 1))
            else:
                wire_latency=0

            crossbar_latency=topo_options['crossbar_latency']

            channel_latency=wire_latency + crossbar_latency
            channel_capacity=channel_latency/serdes_derate

            crossbar_bw=topo_options.get('crossbar_bw', -1)

            if crossbar_bw >= 0:
                link_group_keys_to_add.add( ((topo_type, topo_name), crossbar_bw) )

        else:
            print("Error: Unhandled topology type '%s'! Exiting..." %
                  (topo_type), file=sys.stderr)
            sys.exit(-1)

        # Compute our latency and capacity targets from the different components
        latency = channel_latency + serdes_latency + extra_latency
        capacity = channel_capacity + serdes_capacity + extra_capacity

        assert latency >= 0 and capacity >= 0, "Latency (%f) or capacity (%f) less-than zero, indicating bug in latency/capacity estimates." % (
            latency, capacity)

        # Subtract intrisict latency and capacity
        # 8/6/20 now this isn't subtracted out, so better matching with generated code.
        ic_modeler_intrisict = {
            'in_latency': 1 if len(group_srcs[src_name]) > 1 else 0,
            'in_capacity': 1 if len(group_srcs[src_name]) > 1 else 0,
            'out_latency': 1 if len(group_dests[dest_name]) > 1 else 0,
            'out_capacity': 1 if len(group_dests[dest_name]) > 1 else 0
        }
        latency -= ic_modeler_intrisict['in_latency'] + \
            ic_modeler_intrisict['out_latency']
        capacity -= ic_modeler_intrisict['in_latency'] + \
            ic_modeler_intrisict['out_latency']

        # If it's flagged as an inject, always 0 latency and capacity
        if dest_options.get('inject', False) or src_options.get('inject', False) or topo_options.get('inject', False) or group_options.get('inject', False):
            latency, capacity = (0, 0)

        # Ensure capacity doesn't drop below minimum
        latency = max(0, latency)
        capacity = max(0, capacity)

        # Ensure latency and capacity become ints rounded up
        latency = int(math.ceil(latency))
        capacity = int(math.ceil(capacity))

        # If latency or capacity is greater than 1, ensure the other is at least one.
        if capacity > 0 and latency == 0:
            latency = 1
        if latency > 0 and capacity == 0:
            capacity = 1

        # Sanity check annotation for allowed combinations by modeler
        assert not ((latency > 0 and capacity == 0) or (latency == 0 and capacity > 0) or (latency < 0) or (capacity < 0)
                    ), "Unsupported annotation of latency = %i and capacity = %i for type %s" % (latency, capacity, topo_type)

        # Record data and return
        self.min_latency = min(self.min_latency, latency)
        self.max_latency = max(self.max_latency, latency)
        self.min_capacity = min(self.min_capacity, capacity)
        self.max_capacity = max(self.max_capacity, capacity)

        # Record link group
        for link_group_key_and_local_weight in link_group_keys_to_add:
            link_group_key = link_group_key_and_local_weight[:-1]
            local_weight = link_group_key_and_local_weight[-1]
            self.link_groups.setdefault(
                link_group_key, dict()).setdefault(channel_name, dict())
            self.link_groups[link_group_key][channel_name]['bw_prob'] = float(
                link_group_bw_prob)
            self.link_groups[link_group_key][channel_name]['weight'] = float(
                link_group_weight)*float(local_weight)

        return (latency, capacity)

##############################################################

# Modes of operation:
# - "init" to create a template of groups of objects
# - "annotate" to create sim model annotation
# - "generate" to create an I/O based on current mapping


# init mode
#
# For a set of ports to be valid they must:
# - Be of the same width.
# - Be of the same class type (implies same width).
# - Be of the same (optional) user id. No id is treated as its own ID class.
#
# They don't have to:
# - Share the same dest id map, since may be shared over multiple dest maps, depending on program implementation.
#
# Json includes every combinational channel.
# Can also include class type, width, and user id in output.
#
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interconnect Designer")
    parser.add_argument(
        'mode', choices=['init', 'annotate', 'generate'], help='run mode')
    args = parser.parse_args()

    INTERCONNECT_SC_OBJECT_PATH=os.environ.get('INTERCONNECT_SC_OBJECT_PATH').strip()
    if len(INTERCONNECT_SC_OBJECT_PATH) == 0:
        sys.stderr.print("INTERCONNECT_SC_OBJECT_PATH must be set to path in hierarchy to SC object path!")
        sys.exit(-1)

    INTERCONNECT_DIR="interconnect"
    if os.environ.get('BUILD') and os.environ.get('BUILD').strip():
        INTERCONNECT_DIR += "_" + os.environ.get('BUILD').strip()
    BUILDPREFIX=""
    BUILDPREFIXJSON=INTERCONNECT_SC_OBJECT_PATH

    connectivity_json_file_name = INTERCONNECT_DIR + "/{}.connectivity.json".format(BUILDPREFIXJSON)
    annotation_json_file_name = INTERCONNECT_DIR + "/{}.input.json".format(BUILDPREFIXJSON)
    rpt_file_name = INTERCONNECT_DIR + '/{}interconnect.rpt'.format(BUILDPREFIX)
    yaml_file_name = INTERCONNECT_DIR + '/{}interconnect.yaml'.format(BUILDPREFIX)
    generate_file_name = INTERCONNECT_DIR + '/{}interconnect_gen.hpp'.format(BUILDPREFIX)
    phys_rpt_file_name = INTERCONNECT_DIR + '/{}interconnect.phy.rpt.json'.format(BUILDPREFIX)

    print("INFO: Connectivity JSON = " + connectivity_json_file_name)
    print("INFO: Annotate JSON = " + annotation_json_file_name)
    print("INFO: Interconnect Report = " + rpt_file_name)
    print("INFO: Interconnect Configuration YAML = " + yaml_file_name)
    print("INFO: Interconnect Gen HPP = " + generate_file_name)

    if args.mode == 'init':
        print("Reading connectivity json %s..." %
              connectivity_json_file_name, end='')
        read_connectivity_json(open(connectivity_json_file_name))
        print(" Done.")

        print("Writing report to %s..." % rpt_file_name, end='')
        create_report(open(rpt_file_name, 'w'))
        print(" Done.")

        yaml_file_name_template = yaml_file_name + ".template"

        print("Writing interconnect configuration yaml template to %s..." %
              yaml_file_name_template, end='')
        create_yaml_template(open(yaml_file_name_template, 'w'))
        print(" Done.")
    elif args.mode == "annotate":
        print("Reading connectivity json %s..." %
              connectivity_json_file_name, end='')
        read_connectivity_json(open(connectivity_json_file_name))
        print(" Done.")

        print("Reading interconnect configuration yaml %s..." %
              yaml_file_name, end='')
        YAML_DATA = read_yaml(open(yaml_file_name))
        print(" Done.")

        update_dom(YAML_DATA)

        #print("Writing report to %s..." % rpt_file_name, end='')
        #create_report(open(rpt_file_name, 'w'))
        #print(" Done.")

        tempfile_handle = tempfile.NamedTemporaryFile(mode='w',suffix='.json',prefix='input_annotation.',dir='.',delete=False)
        tempfile_name = tempfile_handle.name
        print("Writing annotation json to %s..." %
              tempfile_name, end='')
        write_annotation_json(tempfile_handle)
        tempfile_handle.close()
        print(" Done.")
        print("Renaming annotation json to %s..." %
              annotation_json_file_name, end='')
        os.rename(tempfile_name, annotation_json_file_name)
        print(" Done.")
    elif args.mode == "generate":
        print("Reading connectivity json %s..." %
              connectivity_json_file_name, end='')
        read_connectivity_json(open(connectivity_json_file_name))
        print(" Done.")

        print("Reading interconnect configuration yaml %s..." %
              yaml_file_name, end='')
        YAML_DATA = read_yaml(open(yaml_file_name))
        print(" Done.")

        update_dom(YAML_DATA)

        print("Generating interconnect...", end='')
        do_generate(YAML_DATA, IC_DATA, open(generate_file_name, 'w'), open(phys_rpt_file_name, 'w'))
        print(" Done.")

# End InterconnectDesigner.py
