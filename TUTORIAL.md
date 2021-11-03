# IPA Tutorial

Set up the environment variables listed in README.md.

This tutorial will cover an arrayed _producer_ and _consumer_ testbench. Each unit or partition has a single producer (source) and a single consumer (sink). Each producer sends randomly generated traffic to random chosen consumers as the simulation runs. The producer-consumer units are tiled in an array.

Take a look within the Producer Consumer demo directory:

    `cd cmod/ProducerConsumer/`

Take a look first at `interconnect_config.hpp`. This file declares identifiers for the top-level interconnect, partition (e.g. unit) types, and message types. In particular, one interconnect object, one partition type, and one message type (`my_msg`) are included in the example.

The SystemC file testbench.cpp is the main code entry point for SystemC simulations, while ProducerConsumerArray.hpp defines the main top-level sc_module under test.

Compile and run the design:

    `make run`

This command will run both the initial modeling mode of IPA (run_model) and the generation mode (run_gen). Exam the output, which will print number of messages sent and average measured latency between every pair of sinks and sources. You will notice the latency is all very similar on this initial run, between 7-11 cycles. The Makefile, testbench, and design are all parameterized to set different Producer-Consumer array sizes, though the PE_WIDTH and PE_HEIGHT make variables. By default, they set the array to 4x4 in size. Subsequently, the work directory created is called `interconnect_4x4`. Let's investigate what was generated within that direcctory.

    `cd interconnect_4x4`

The IPA-generated interconnect config template is given in `interconnect.yaml.template`, which is symlinked to `interconnect.yaml` by the Makefile by default. However, if a user-specified interconnect config file `interconnect.yaml.user` is provided, then that is symlinked intead.

    `cp interconnect.yaml.template interconnect.yaml.user`

Edit interconnect.yaml.user with your favorite text editor. You will see that the `my_msg` class is initially assigned to the `link` group (directly-connected, pairwise links between every producer and consumer) and that the default floorplan coordinates for each unit are set to the origin (x=0, y=0). Let's modify the `interconnect.yaml.user` so that a `noc` is generated (uniform mesh network-on-chip) and arrange the units in an array.

    ```
    groups:
    2: {_conn_type: 'N:N', _dests: 16, _srcs: 16, _width: 64, name: my_msg}
    topologies:
    crossbar:
        groups: []
        options: {bus_width: 64, crossbar_latency: 2, crossbar_x_coor: 0, crossbar_y_coor: 0,
        wire_prop_speed: 1}
        type: crossbar
    link:
        groups: []
        options: {wire_prop_speed: 1}
        type: link
    noc:
        groups: [my_msg]
        options: {bus_width: 64, router_latency: 1, router_module: HybridRouter, router_spacing: 1,
        wire_prop_speed: 1}
        type: noc
    units:
    my_testbench.dut.pemodulearray_inst.pe_part_inst_0: {x_coor: 0, y_coor: 0}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_1: {x_coor: 1, y_coor: 0}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_2: {x_coor: 2, y_coor: 0}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_3: {x_coor: 3, y_coor: 0}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_4: {x_coor: 0, y_coor: 1}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_5: {x_coor: 1, y_coor: 1}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_6: {x_coor: 2, y_coor: 1}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_7: {x_coor: 3, y_coor: 1}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_8: {x_coor: 0, y_coor: 2}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_9: {x_coor: 1, y_coor: 2}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_10: {x_coor: 2, y_coor: 2}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_11: {x_coor: 3, y_coor: 2}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_12: {x_coor: 0, y_coor: 3}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_13: {x_coor: 1, y_coor: 3}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_14: {x_coor: 2, y_coor: 3}
    my_testbench.dut.pemodulearray_inst.pe_part_inst_15: {x_coor: 3, y_coor: 3}
    ```

Note that units for x_coor, y_coor, router_spacing, crossbar_x_coor, and crossbar_y_coor are unitless but must be consistent with wire_prop_speed, which is in cycles per unit-distance. For example, if 1 unit of distance = 100 micron then wire_prop_speed (wired route propogation speed) is cycles per 100 micron of distance, and x_coor / y_coor are in intervals of 100 micron. The coordinates and wire_prop_speed given need not be integers; floats are allowed.

Save the file, and go back up one level above the `interconnect_4x4` directory.

    ` cd ..`

Now, rerun the simulations.

    `make run`

Inspect the output now, and you should see more variation and higher measured latency (10-80 cycles), due to both increased spatial arrangement of the units, hops through routers, and cross-sectional bandwidth limits since all of the producers are sending messages every cycle. Let's dial down the activity rate of the producers to 5% activity:

    `make clean; make run PE_ACTIVITY=0.05`

The measured latency should now be lower (10-35 cycles) because there is less congestion in the network.

Let's now exam the generated interconnect code, in `interconnect_4x4/interconnect_gen.hpp`. This file includes synthesizable code to encode the message (including destination information), interfaces to connect the unit message ports with the routers, and instantiations of the routers themselves. The top-level interconnect object is named `Grout_0` and thus a `Grout_0.hpp` is needed in the source code directory for Catapult HLS to refer to.

Next, let's bring the generated interconnect through high-level synthesis.

    ```
    cd ../../../hls/interconnect/ProducerConsumer/Grout_0/`
    make 
    ```

Once HLS is complete, the resulting RTL can be viewed in `4x4/Grout_0.v1/concat_rtl.v`.
