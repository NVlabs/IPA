Interconnect Prototyping Assistant (IPA)
========

# Getting Started

Interconnect Prototyping Assistant (IPA) is a VLSI interconnect modeling and generation framework built atop [MatchLib](https://github.com/NVlabs/matchlib) and [Connections](https://github.com/hlslibs/matchlib_connections) libraries.

## Tool versions

MatchLib is regressed against the following tool/dependency verions:

* `gcc` - 4.9.3 (with C++11)
* `systemc` - 2.3.1
* `boost` - 1.68.0
* `doxygen` - 1.8.11
* `make` - 3.82
* `catapult` - 10.5a
* `connections` - included with catapult
* `vcs` - 2017.03-SP2-11
* `verdi` - 2017.12-SP2-11
* `rapidjson` - v1.1.0
* `python` - 3.4.2

## Environment requirements

Makefiles assume the appropriate definition of the following environment variables:

* `MATCHLIB_HOME`
* `BOOST_HOME`
* `SYSTEMC_HOME`
* `CATAPULT_HOME`
* `VCS_HOME`
* `NOVAS_INST_DIR`

In addition, the boost and systemc library locations are expected to be in `LD_LIBRARY_PATH`.

## Build and run example

### C++ compile and simulate
    cd cmod/ProducerConsumer/
    make
    make run 

### HLS run and Verilog simulate all
    cd hls/ProducerConsumer/Grout_0
    make hls

# Directory structure

* `bin` contains Python scripts for interconnect prototyping assistant
* `cmod/interconnect/include/*.h` contains header files for functions and classes for Interconnect Prototyping Assistant
* `cmod/interconnect/ProducerConsumer/` sub-directories containing example IPA use
* `hls/interconnect/ProducerConsumer/Grout_0/` sub-directories contain HLS scripts for IPA implementation
* `doc/` contains Makefiles for building Doxygen-based documentation

# API Reference

Supported language: SystemC.

See Doxygen generated documentation in `doc/` with more information.

## Macro Reference

### Message and Unit Macros

`IC_MESSAGE(MSG_TYPE)`: Message data type for message classes of `MSG_TYPE`, whose payload is defined in the interconnect config header. Can be used to instantiate new messages (ex. `IC_Message(my_msg) new_message;`), or passed as a template parameter to Connections ports (ex. `Connections::In<IC_MESSAGE(my_msg)>`).

`IC_TO(DEST_ID)`: Used along with the `<<` operator to append dest_id of `DEST_ID` to the message's destination list. Multiple destinations can be added for a single message. For example, `im << IC_TO(42) << IC_TO(86) << IC_TO(27);` will add destination indexes of 42, 86, and 27 to message `im`. Destination indexes are assigned in the IC_BIND_PART macro described below. `DEST_ID` may be a runtime variable.

### Partition Macros

`IC_HAS_INTERFACE(PART_REF)`: Added to the partition class declaration. Declares that the class is a partition of name `PART_REF`. PART_REF should match the C++ class name and the name given to `IC_ADD_PARTITION()` in the interconnect config header.

`IC_BIND_PORT(PORT_INST)`: Added to the partition class constructor. Registered the `Connections::In` and `::Out` port `PORT_INST` with the Interconnect API. Should only be called once per port.

### Top-Level Macros

`IC_HAS_INTERCONNECT(INTERCONNECT_REF)`: Added to the declaration of the top-level class. `INTERCONNECT_REF` should match the name given in `IC_ADD_INTERCONNECT()` in the interconnect config header.

`IC_BIND_PART(PART_INST, DEST_ID)`: Added to the top-level class's constructor. Registers partition instance `PART_INST` with the Interconnect API, and assigns it a destination index of `DEST_ID` that can be references with `IC_TO(). `DEST_ID` must be a constant at compile time. 

## Interconnect Configuration File Reference

File type: YAML

### Interconnect Topology Options

#### Common Options

`wire_prop_speed`: Sets the speed in clock cycles of wires per unit-distance. For `X` units of manhattan distance, `ceil(X/wire_prop_speed)` of clock cycles are needed.  (default: 1.0)
`extra_latency` (modeling only): Extra latency to add to an interconnect type, in clock cycles. Useful for sensitivity analysis. Will also be added to generated interconnect as increased number of retiming stages. (default: 0)
`extra_capacity` (modeling only):  Extra link capacity to add to an interconnect type, in clock cycles. Useful for sensitivity analysis. Will also be added to generated interconnect as increased number buffers within retiming stages. (default: 0)

#### Directly-Connected Links

YAML topology type set to `link`.

No additional supported options beyond the common options set above.

#### Centralized Crossbar

YAML topology type set to `crossbar`.

Supported YAML Options:

* `crossbar_latency` (modeling only): Cycle latency through the crossbar, from input to output.  (default: 2)
* `crossbar_bw` (modeling only): Amount of bandwidth supported by the crossbar as a multiples of the max message size through the crossbar. -1 indicates no limits.  (default: -1)
* `crossbar_x_coor`: x coordinate of centralized crossbar on floorplan.  (default: 0.0)
* `crossbar_y_coor`: y coordinate of centralized crossbar on floorplan.  (default: 0.0)

#### Uniform Mesh Network-on-Chip

YAML topology type set to `noc`.

Supported YAML Options:

`router_latency` (modeling only): Cycle latency through a router, from input to output. It should be set for what the router can be scheduled for in HLS.  (default: 1)
`router_spacing`: Unit distance between routers on floorplan.  (default: 1.0)
`bus_width`: Flit width of the generated network-on-chip. Messages are serialized and deserialized to this width. (default: 64)
Unit Options

#### Supported options for units:

`x_coor`: X-coordinate in arbitrary units of partition instance on floorplan,.  (default: 0.0)
`y_coor`: Y-coordinate in arbitrary units of partition instance on floorplan.  (default: 0.0)
`inject` (modeling only): When True marks that message to/from this partition should not be annotated with latency or capacity during modeling. Useful for testbench partitions that don't have physical meaning.  (default: False)

# Contributors

MatchLib originated as a project of [NVIDIA Research](https://research.nvidia.com).

Contributors to the initial open-source release (alphabetical): Ben Keller, Brucek Khailany, Nathaniel Pinckney, Rangharajan Venkatesan

IPA's channel annotation feature is based on MatchLib's implementation and dependent on RapidJSON released under the MIT License. 

# Attribution

If used for research, please cite the [ICCAD paper](https://d1qx31qr3h6wln.cloudfront.net/publications/IPA_ICCAD_2021_Manuscript.pdf):

Nathaniel Pinckney, Rangharajan Venkatesan, Ben Keller, and Brucek Khailany, "IPA: Floorplan-Aware SystemC Interconnect Performance Modeling and Generation for HLS-based SoCs," in International Conference On Computer-Aided Design (ICCAD), November 2021.

