# 
# SPDX-FileCopyrightText: Copyright (c) 2016-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
# 

ROOT = $(abspath ../../../)
include $(ROOT)/cmod/cmod_Makefile

# Configure build name and debug level.
BUILD ?= $(PE_WIDTH)x$(PE_HEIGHT)
DEBUG_LEVEL ?= -1

# The below must match up with interconnect_designer.py
INTERCONNECT_SC_OBJECT_PATH := my_testbench.dut.pcmodulearray_inst.interconnect_0
INTERCONNECT_TB_OBJECT_PATH := my_testbench.dut.interconnect_0

include $(ROOT)/cmod/interconnect/Interconnect.mk

PE_WIDTH ?= 4
PE_HEIGHT ?= 4
RI_BUFSIZE ?= 1

# Needed for Catapult 10.6+ compatibility to maintain original Connections signal naming.
USER_FLAGS += -DCONNECTIONS_NAMING_ORIGINAL
USER_FLAGS += -DSC_INCLUDE_DYNAMIC_PROCESSES -DCONNECTIONS_ACCURATE_SIM
USER_FLAGS += -DRAPIDJSON_HAS_STDSTRING=1 -DENABLE_PERF_COUNTERS
USER_FLAGS += -DPE_WIDTH=$(PE_WIDTH) -DPE_HEIGHT=$(PE_HEIGHT)

INTERCONNECT_DISABLE_CONGESTION ?= 0

# For CSolver
CFLAGS += -I${VCS_HOME}/include
CFLAGS += -g

PE_ACTIVITY ?= 1.00
USER_FLAGS += -DPE_ACTIVITY=$(PE_ACTIVITY)

PE_LOCAL_ONLY ?= 0
USER_FLAGS += -DPE_LOCAL_ONLY=$(PE_LOCAL_ONLY)

CLEANUP_BA_FILES ?= 0
USER_FLAGS += -O1 -Wno-uninitialized -Wno-unused-local-typedefs -Wno-unused-value

EXE_GEN_NAME ?= $(EXE_NAME)_gen
EXE_MODEL_NAME ?= $(EXE_NAME)_model

SLEEP ?= 

ifeq ($(SLEEP),1)
SLEEP_CMD ?= sleep 5
else
SLEEP_CMD ?= 
endif


###### Run targets for interconnect mode

ifneq ($(INTERCONNECT_GEN),)

ifeq ($(INTERCONNECT_GEN),1)
.PHONY: all run
all: ${EXE_GEN_NAME}
run: run_gen
else
.PHONY: all run
all: ${EXE_MODEL_NAME}
run: run_model
endif

else
.PHONY: all run
all: ${EXE_MODEL_NAME} ${EXE_GEN_NAME}
run: run_model run_gen
endif

${EXE_MODEL_NAME}: $(wildcard *.h) $(filter-out $(INTERCONNECT_GEN_HPP_FILE),$(wildcard *.hpp)) $(wildcard *.cpp) $(wildcard $(ROOT)/cmod/magnet/include/*.h)
	mkdir -p "$(INTERCONNECT_DIR)"
	env BUILD=$(BUILD) $(CC) -o ${EXE_MODEL_NAME} -DHLS_CATAPULT -DDEBUG_LEVEL=$(DEBUG_LEVEL) $(CFLAGS) $(USER_FLAGS) testbench.cpp $(BOOSTLIBS) $(LIBS)

${EXE_GEN_NAME}: $(wildcard *.h) $(filter-out $(INTERCONNECT_GEN_HPP_FILE),$(wildcard *.hpp)) $(wildcard *.cpp) $(wildcard $(ROOT)/cmod/magnet/include/*.h) $(INTERCONNECT_GEN_HPP_FILE)
	mkdir -p "$(INTERCONNECT_DIR)"
	env BUILD=$(BUILD) $(CC) -o ${EXE_GEN_NAME} -DHLS_CATAPULT -DDEBUG_LEVEL=$(DEBUG_LEVEL) $(CFLAGS) $(USER_FLAGS) -DINTERCONNECT_GEN="$(INTERCONNECT_GEN_HPP_FILE)" testbench.cpp $(BOOSTLIBS) $(LIBS)

# Run elaboration to generate connectivity.json
$(CONNECTIVITY_JSON): ${EXE_MODEL_NAME}
	mkdir -p "$(INTERCONNECT_DIR)"
	env BUILD=$(BUILD) NVHLS_ELAB_ONLY=1 ./${EXE_MODEL_NAME}
	rm -f "$(CONNECTIVITY_TB_JSON)"

$(YAML_CONFIG_TEMPLATE) $(INTERCONNECT_RPT): $(CONNECTIVITY_JSON) $(ROOT)/bin/interconnect_designer.py
	mkdir -p "$(INTERCONNECT_DIR)"
	BUILD=$(BUILD) $(ROOT)/bin/interconnect_designer.py init

$(YAML_CONFIG): $(YAML_CONFIG_USER) $(YAML_CONFIG_TEMPLATE)
	mkdir -p "$(INTERCONNECT_DIR)"
	ln -sf $(notdir $<) $@
	$(SLEEP_CMD)

$(INTERCONNECT_GEN_HPP_FILE) $(INTERCONNECT_PHY_RPT): $(YAML_CONFIG) $(ROOT)/bin/interconnect_designer.py $(ROOT)/bin/interconnect_generate.py
	mkdir -p "$(INTERCONNECT_DIR)"
	env BUILD=$(BUILD) $(ROOT)/bin/interconnect_designer.py generate
	$(SLEEP_CMD)

$(INPUT_JSON): $(YAML_CONFIG) $(ROOT)/bin/interconnect_designer.py
	mkdir -p "$(INTERCONNECT_DIR)"
	env BUILD=$(BUILD) $(ROOT)/bin/interconnect_designer.py annotate
	$(SLEEP_CMD)


.PHONY: run_model
run_model $(OUTPUT_JSON): ${EXE_MODEL_NAME} $(INPUT_JSON) 
	mkdir -p "$(INTERCONNECT_DIR)"
	env BUILD=$(BUILD) INTERCONNECT_DISABLE_CONGESTION=$(INTERCONNECT_DISABLE_CONGESTION) ./${EXE_MODEL_NAME}
	rm -f "$(OUTPUT_TB_JSON)"
	rm -f "$(OUTPUT_JSON)"
ifeq ($(CLEANUP_BA_FILES),1)
	rm -f "$(INPUT_JSON)"
endif

.PHONY: run_gen
run_gen: ${EXE_GEN_NAME}
	mkdir -p "$(INTERCONNECT_DIR)"
	env BUILD=$(BUILD) INTERCONNECT_DISABLE_CONGESTION=$(INTERCONNECT_DISABLE_CONGESTION) ./${EXE_GEN_NAME}

###### Shared targets
.PHONY: clean
clean: cov_clean
	rm -f "${EXE_NAME}" "${EXE_MODEL_NAME}" "${EXE_GEN_NAME}" dump.vcd nvhls_runset.mk
	rm -f "${CONNECTIVITY_JSON}" "${INPUT_JSON}" "${OUTPUT_JSON}"
	rm -f "${CONNECTIVITY_TB_JSON}" "${OUTPUT_TB_JSON}"
	rm -f "${YAML_CONFIG}" "${YAML_CONFIG_TEMPLATE}" "$(INTERCONNECT_RPT)" "$(INTERCONNECT_GEN_HPP_FILE)" "$(INTERCONNECT_PHY_RPT)"
	rm -f "$(INTERCONNECT_DIR)/interconnect_gen_mazeRoute.pm" "$(INTERCONNECT_DIR)/interconnect_gen_mazeRoute_post.sh"
	! test -d "$(INTERCONNECT_DIR)" || rmdir "$(INTERCONNECT_DIR)"

.PHONY: sim_clean
sim_clean:
	rm -rf *.o sim_* parallel_runs*
