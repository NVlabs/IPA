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

* `TUTORIAL.md` information on getting started
* `bin` contains Python scripts for interconnect prototyping assistant
* `cmod/interconnect/include/*.h` contains header files for functions and classes for Interconnect Prototyping Assistant
* `cmod/interconnect/ProducerConsumer` sub-directories containing example IPA use
* `hls/interconnect/ProducerConsumer/Grout_0` sub-directories contain HLS scripts for IPA implementation
* `doc` contains Makefiles for building Doxygen-based documentation

# Contributors

MatchLib originated as a project of [NVIDIA Research](https://research.nvidia.com).

Contributors to the initial open-source release (alphabetical): Ben Keller, Brucek Khailany, Nathaniel Pinckney, Rangharajan Venkatesan

IPA's channel annotation feature is based on MatchLib's implementation and dependent on RapidJSON released under the MIT License. 

# Attribution

If used for research, please cite the [ICCAD paper](https://d1qx31qr3h6wln.cloudfront.net/publications/IPA_ICCAD_2021_Manuscript.pdf):

Nathaniel Pinckney, Rangharajan Venkatesan, Ben Keller, and Brucek Khailany, "IPA: Floorplan-Aware SystemC Interconnect Performance Modeling and Generation for HLS-based SoCs," in International Conference On Computer-Aided Design (ICCAD), November 2021.

