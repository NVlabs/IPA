/*
 * SPDX-FileCopyrightText: Copyright (c) 2020-2021 NVIDIA CORPORATION &
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

// Header file to declare interconnects, units, and messages for Interconnect
// Prototyping Assistant. (IPA)

#ifndef __INTERCONNECT_CONFIG_H__
#define __INTERCONNECT_CONFIG_H__

#include <interconnect/Interconnect.hpp>

namespace interconnect_config {
IC_BEGIN_INTERCONNECTS
IC_ADD_INTERCONNECT(pcmodulearray)
IC_END_INTERCONNECTS

// Maps are used to set custom numbered routing, for flit adapter traffic or
// other more custom uses.
IC_BEGIN_DEST_MAPS
IC_END_DEST_MAPS

IC_BEGIN_PARTITION_TYPES
IC_BEGIN_PARTITION(ProducerConsumerPart)
IC_END_PARTITION
IC_END_PARTITION_TYPES

IC_BEGIN_MESSAGE_TYPES
IC_ADD_MESSAGE_TYPE(my_msg, NVUINTW(64))
IC_ADD_MESSAGE_TYPE(my_ctrl, bool)
IC_END_MESSAGE_TYPES
};

#endif
