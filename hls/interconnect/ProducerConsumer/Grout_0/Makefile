# 
# SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#

ROOT := ../../../..

PE_WIDTH ?= 4
PE_HEIGHT ?= 4

BUILD ?= $(PE_WIDTH)x$(PE_HEIGHT)

# The below must match up with interconnect_designer.py and interconnect_update.py. They can't be changed arbitrarly.
INTERCONNECT_SC_OBJECT_PATH := my_testbench.dut.pemodulearray_inst.interconnect_0
INTERCONNECT_TB_OBJECT_PATH := my_testbench.dut.interconnect_0

include ${ROOT}/cmod/interconnect/Interconnect.mk

BUP_BLOCKS := 
RUN_SCVERIFY := 0

HLS(LSF_QUEUE) ?= o_cpu_16G_8H

include ${ROOT}/hls/hls_Makefile

COMPILER_FLAGS += VECTOR_SIZE=$(VECTOR_SIZE) VECTOR_LANES=$(VECTOR_LANES) ACTIVATION_WORDWIDTH=$(ACTIVATION_WIDTH) WEIGHT_WORDWIDTH=$(WEIGHT_WIDTH) DATAFLOW=$(DATAFLOW) WC_DEPTH=$(WC_DEPTH) OC_DEPTH=$(OC_DEPTH) PE_WIDTH=$(PE_WIDTH) PE_HEIGHT=$(PE_HEIGHT)
COMPILER_FLAGS += DISABLE_PEPARTITIONTEST
COMPILER_FLAGS += INTERCONNECT_MODELER_NATIVE
COMPILER_FLAGS += INTERCONNECT_GEN=$(INTERCONNECT_GEN_HPP_FILE)

COMPILER_FLAGS += DEBUG_LEVEL=2

SEARCH_PATH += $(RAPIDJSON_HOME)/include
SEARCH_PATH += ${TOT}/cmod/interconnect/include

SRC_PATH = ${ROOT}/cmod/interconnect/ProducerConsumer
