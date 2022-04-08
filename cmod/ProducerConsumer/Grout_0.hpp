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

// This is a stub needed for the generated interconnect, so that downstream flow
// and simulation knows where to look for it.

#ifndef GROUT_0_H
#define GROUT_0_H

#include <systemc.h>
#include <nvhls_connections.h>
#include <nvhls_packet.h>

#include <interconnect/Interconnect.hpp>
#include "interconnect_config.hpp"

#if defined(INTERCONNECT_GEN)
#include ic_header(INTERCONNECT_GEN)
#endif  // if defined(INTERCONNECT_GEN)

#endif
