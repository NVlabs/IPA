/*
 * SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION &
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

#ifndef __INTERCONNECT_CHANNELS_HPP__
#define __INTERCONNECT_CHANNELS_HPP__

#include <iostream>
#include <string>
#include <functional>
#include <algorithm>

#include <nvhls_connections.h>

namespace interconnect {

#ifdef CONNECTIONS_SIM_ONLY

/////////////

class ConManagerLinks;
ConManagerLinks& get_conManagerLinks();

template <class Dummy>
struct ConManagerLinks_statics {
  static ConManagerLinks conManagerLinks;
};

inline ConManagerLinks& get_conManagerLinks() {
  return ConManagerLinks_statics<void>::conManagerLinks;
}

class CombinationalLink_abs;
template <typename Message>
class CombinationalLink;

class CombinationalLink_abs {
 public:
  double bw_prob;

  virtual void Reset_BWProb() {
    NVHLS_ASSERT_MSG(0, "Unreachable virtual function in abstract class!");
  }

  virtual bool EvaluateCongest(bool degraded_mode = false) {
    NVHLS_ASSERT_MSG(0, "Unreachable virtual function in abstract class!");
    return 0;
  }

  virtual bool IsValid() {
    NVHLS_ASSERT_MSG(0, "Unreachable virtual function in abstract class!");
    return 0;
  }

  virtual bool ShouldSkipUpdateCongest() {
    NVHLS_ASSERT_MSG(0, "Unreachable virtual function in abstract class!");
    return 0;
  }

  virtual const char* const get_name() {
    NVHLS_ASSERT_MSG(0, "Unreachable virtual function in abstract class!");
  }
};

// A list of Combinationals that share the same link,
// for back-annotation modeling.
class LinkGroup {
 public:
  std::map<CombinationalLink_abs*, double> weights;
  std::map<CombinationalLink_abs*, double> link_probs;

  int num_total_count;
  float num_non_zero_count = -1;

  double count_active_weights(double min_weight = 0) {
    double a = 0;
    for (std::map<CombinationalLink_abs*, double>::iterator it =
             weights.begin();
         it != weights.end(); ++it) {
      if (it->second < min_weight)
        continue;
      if (!it->first->ShouldSkipUpdateCongest())  // ! ShouldSkipUpdateCongest()
                                                  // (this includes isBypass)
                                                  // would be better to use. We
                                                  // can't stall when isBypass()
                                                  // but we can still count it
                                                  // towards active.
        a += it->second;
    }
    return a;
  }

  double max_active_weights() {
    double a = 0;
    for (std::map<CombinationalLink_abs*, double>::iterator it =
             weights.begin();
         it != weights.end(); ++it) {
      if (!it->first->ShouldSkipUpdateCongest())  // ! ShouldSkipUpdateCongest()
                                                  // (this includes isBypass)
                                                  // would be better to use. We
                                                  // can't stall when isBypass()
                                                  // but we can still count it
                                                  // towards active.
        a = std::max(a, it->second);
    }
    return a;
  }

  double count_active_links() {
    double a = 0;
    for (std::map<CombinationalLink_abs*, double>::iterator it =
             weights.begin();
         it != weights.end(); ++it) {
      if (!it->first->ShouldSkipUpdateCongest())  // ! ShouldSkipUpdateCongest()
                                                  // (this includes isBypass)
                                                  // would be better to use. We
                                                  // can't stall when isBypass()
                                                  // but we can still count it
                                                  // towards active.
        a += 1;
    }
    return a;
  }

  inline double bound(double v, double min_v, double max_v) {
    return std::max(min_v, std::min(max_v, v));
  }

  void UpdateCongest(bool degraded_mode = false) {
    double max_weight = max_active_weights();
    double active_weights = count_active_weights(max_weight);
    double active_links = count_active_links();

    if (active_links == 0)
      return;

    NVHLS_ASSERT(active_weights > 0);

    for (std::map<CombinationalLink_abs*, double>::iterator it =
             link_probs.begin();
         it != link_probs.end(); ++it) {
      // If we're not valid, then skip calculations.
      if (it->first->ShouldSkipUpdateCongest())
        continue;

      // Calculate local bw prob
      double ratio;
      if (!degraded_mode) {
        ratio = weights[it->first] / active_weights;
      } else {
        ratio = 1.0 / active_links;
      }

      // limit range of bw_prob to 0.0 <= <= 1.0
      double link_prob = it->second * ratio;
      if (weights[it->first] < max_weight && !degraded_mode) {
        link_prob = 0;
      }
      NVHLS_ASSERT(0 <= link_prob && link_prob <= 1.0);

      it->first->bw_prob = std::min(link_prob, it->first->bw_prob);
    }
  }

  void CalculateNonZero() {
    num_non_zero_count = 0;
    num_total_count = 0;

    for (std::map<CombinationalLink_abs*, double>::iterator it =
             link_probs.begin();
         it != link_probs.end(); ++it) {
      if (it->first->ShouldSkipUpdateCongest())
        continue;
      num_total_count++;
      num_non_zero_count += it->first->bw_prob;
    }
  }
};

class ConManagerLinks : public Connections::ConManager {
 public:
  std::vector<LinkGroup*> link_groups;
  std::vector<CombinationalLink_abs*>
      links;  // Links of the connnections themselves
  std::map<CombinationalLink_abs*, Connections::Blocking_abs*>
      links_tracked;  // Links of the connnections themselves

  ConManagerLinks() : Connections::ConManager() {
    sc_spawn(sc_bind(&ConManagerLinks::run, this, true),
             "connection_manager_links_run");
  }

  template <typename T>
  void addlink(T* c) {
    links.push_back(static_cast<CombinationalLink_abs*>(c));
    links_tracked[c] = static_cast<Connections::Blocking_abs*>(c);
  }

  void run(bool value) {
    wait(SC_ZERO_TIME);  // Allow the simulation to catch up with clock
                         // registration, etc...

#ifdef CAP_CONNECTIONS_MULTI_CLK
    // Find all that we (ConManagerLinks) are tracking and remove from
    // tracked_per_clk.
    for (auto it_to_find = links_tracked.begin();
         it_to_find != links_tracked.end(); ++it_to_find) {
      for (auto it_outer =
               Connections::get_conManager().tracked_per_clk.begin();
           it_outer != Connections::get_conManager().tracked_per_clk.end();
           ++it_outer) {
        for (auto it_inner = (*it_outer)->begin();
             it_inner != (*it_outer)->end(); ++it_inner)
          if (*it_inner == it_to_find->second) {
            (*it_outer)->erase(it_inner);
          }
      }
    }
#endif

    bool last_degraded_mode = false;

#ifdef CAP_CONNECTIONS_MULTI_CLK
    Connections::get_sim_clk().post_delay(
        0);  // align to occur just after the cycle
#else
    Connections::get_sim_clk()
        .post_delay();  // align to occur just after the cycle
#endif

    while (1) {
      bool degraded_mode = false;

      // Post(): Update, set rdy and val appropriately
      for (std::vector<CombinationalLink_abs*>::iterator it = links.begin();
           it != links.end();)
        if (links_tracked[(*it)]->Post())
          ++it;
        else
          links.erase(it);

      // Wait zero cycle, so that all signals update before counting valids.
      wait(SC_ZERO_TIME);

      // UpdateCongest(): For each link group, calculate individual link
      // probability.
      for (std::vector<LinkGroup*>::iterator it = link_groups.begin();
           it != link_groups.end(); ++it) {
        (*it)->UpdateCongest(degraded_mode);
      }

      int num_total_count = 0;
      float num_non_zero_count = 0;
      for (std::vector<LinkGroup*>::iterator it = link_groups.begin();
           it != link_groups.end(); ++it) {
        (*it)->CalculateNonZero();
        num_total_count += (*it)->num_total_count;
        num_non_zero_count += (*it)->num_non_zero_count;
        NVHLS_ASSERT((*it)->num_non_zero_count == 0 ||
                     (*it)->num_total_count > 0);
      }
      if (num_total_count > 0 && num_non_zero_count == 0) {
        degraded_mode = true;
        if (last_degraded_mode != degraded_mode) {
          cout << "@" << sc_time_stamp()
               << ": Warning: Deadlock detection triggered, rerunning "
                  "congestion calculation in degraded mode so that it can "
                  "continue..."
               << endl;
        }
        // Deadlock case, re-do UpdateCongestion with balanced
        for (std::vector<CombinationalLink_abs*>::iterator it = links.begin();
             it != links.end(); ++it) {
          (*it)->Reset_BWProb();
        }
        for (std::vector<LinkGroup*>::iterator it = link_groups.begin();
             it != link_groups.end(); ++it) {
          (*it)->UpdateCongest(degraded_mode);
        }
        num_total_count = 0;
        num_non_zero_count = 0;
        for (std::vector<LinkGroup*>::iterator it = link_groups.begin();
             it != link_groups.end(); ++it) {
          (*it)->CalculateNonZero();
          num_total_count += (*it)->num_total_count;
          num_non_zero_count += (*it)->num_non_zero_count;
          NVHLS_ASSERT((*it)->num_non_zero_count == 0 ||
                       (*it)->num_total_count > 0);
        }
      }
      last_degraded_mode = degraded_mode;
      NVHLS_ASSERT(num_total_count == 0 || num_non_zero_count > 0);
      // EvaluateCongest: examine and deassert rdy to limit bandwidth
      for (std::vector<CombinationalLink_abs*>::iterator it = links.begin();
           it != links.end();)
        if ((*it)->EvaluateCongest(false))
          ++it;
        else
          links.erase(it);

// Post -> Pre delay like normal
#ifdef CAP_CONNECTIONS_MULTI_CLK
      Connections::get_sim_clk().post2pre_delay(0);
#else
      Connections::get_sim_clk().post2pre_delay();
#endif

      for (std::vector<CombinationalLink_abs*>::iterator it = links.begin();
           it != links.end();)
        if (links_tracked[(*it)]->Pre())
          ++it;
        else
          links.erase(it);

      // Pre -> Post delay like normal
      Connections::get_sim_clk().pre2post_delay();
    }
  }
};

template <class Dummy>
ConManagerLinks ConManagerLinks_statics<Dummy>::conManagerLinks;

template <typename Message>
class CombinationalLink : public Connections::Combinational<Message>,
                          public CombinationalLink_abs {
 protected:
  static const int precision = 1000;

 public:
  CombinationalLink()
      : Connections::Combinational<Message>(), CombinationalLink_abs() {
    Reset_BWProb();
    Connections::get_conManager().remove(this);
    Connections::get_conManager().remove_annotate(this);
    get_conManagerLinks().addlink(this);
    get_conManagerLinks().add_annotate(this);
  }

  explicit CombinationalLink(const char* name)
      : Connections::Combinational<Message>(name), CombinationalLink_abs() {
    Reset_BWProb();
    Connections::get_conManager().remove(this);
    Connections::get_conManager().remove_annotate(this);
    get_conManagerLinks().addlink(this);
    get_conManagerLinks().add_annotate(this);
  }

  void Reset_SIM() {
    Reset_BWProb();
    Connections::Combinational<Message>::Reset_SIM();
  }

  bool EvaluateCongest(bool degraded_mode) {
    if (!ShouldSkipUpdateCongest() && !degraded_mode) {
      NVHLS_ASSERT(bw_prob >= 0 && bw_prob <= 1.0);

      if (this->val_set_by_api && (bw_prob < 1.0) &&
          ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) >
           bw_prob)) {
        // Stall due to congestion modeling!
        this->_VLDNAMEOUT_.write(false);
        this->val_set_by_api = false;
      }
    }
    // Reset bw_prob after using it.
    Reset_BWProb();

    return true;
  }

  bool IsValid() {

#ifdef CONNECTIONS_ACCURATE_SIM
    return this->_VLDNAMEOUT_.read();
#else
    return false;  // FIXME CONNECTONS_FAST_SIM is currently unsupported by IPA
#endif
  }

  bool IsReady() {
#ifdef CONNECTIONS_ACCURATE_SIM
    return this->_RDYNAMEOUT_.read();
#else
    return false;  // FIXME CONNECTONS_FAST_SIM is currently unsupported by IPA
#endif
  }

  virtual bool ShouldSkipUpdateCongest() {
#ifdef CONNECTIONS_ACCURATE_SIM
    return (!(IsValid() && IsReady())) || this->is_bypass();
#else
    return false;  // FIXME CONNECTONS_FAST_SIM is currently unsupported by IPA
#endif
  }

  void Reset_BWProb() { bw_prob = 1.0; }

  virtual const char* const get_name() { return this->name(); }
};

#else

#endif
};

#endif  // __INTERCONNECT_CHANNELS_HPP__
