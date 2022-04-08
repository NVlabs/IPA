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
# 
# Additional options needed for interconnect generation

ifndef INTERCONNECT_SC_OBJECT_PATH
$(error INTERCONNECT_SC_OBJECT_PATH is not set)
endif
export INTERCONNECT_SC_OBJECT_PATH

ifndef INTERCONNECT_TB_OBJECT_PATH
$(error INTERCONNECT_TB_OBJECT_PATH is not set)
endif
export INTERCONNECT_TB_OBJECT_PATH

# Set default binary names.
ifneq ($(strip $(BUILD)),)
EXE_NAME ?= sim_$(BUILD)
BUILDPREFIX=$(BUILD).
BUILDSUFFIX=_$(BUILD)
else
EXE_NAME ?= sim_test
BUILDPREFIX=
BUILDSUFFIX=
endif

INTERCONNECT_DIR=interconnect$(BUILDSUFFIX)
INTERCONNECT_GEN_HPP_FILE=$(INTERCONNECT_DIR)/interconnect_gen.hpp

CONNECTIVITY_JSON=$(INTERCONNECT_DIR)/$(INTERCONNECT_SC_OBJECT_PATH).connectivity.json
CONNECTIVITY_TB_JSON=$(INTERCONNECT_DIR)/$(INTERCONNECT_TB_OBJECT_PATH).connectivity.json
INPUT_JSON=$(INTERCONNECT_DIR)/$(INTERCONNECT_SC_OBJECT_PATH).input.json
OUTPUT_JSON=$(INTERCONNECT_DIR)/$(INTERCONNECT_SC_OBJECT_PATH).output.json
OUTPUT_TB_JSON=$(INTERCONNECT_DIR)/$(INTERCONNECT_TB_OBJECT_PATH).output.json
YAML_CONFIG=$(INTERCONNECT_DIR)/interconnect.yaml
YAML_CONFIG_TEMPLATE=$(YAML_CONFIG).template
YAML_CONFIG_USER=$(wildcard $(YAML_CONFIG).user)
INTERCONNECT_RPT=$(INTERCONNECT_DIR)/interconnect.rpt
INTERCONNECT_PHY_RPT=$(INTERCONNECT_DIR)/interconnect.phy.rpt.json
