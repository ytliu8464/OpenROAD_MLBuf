// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2018-2025, The OpenROAD Authors
#include "ord/OpenRoad.hh" // YL 
#include "sta/VerilogWriter.hh" // YL
#include "timingBase.h"
#include <filesystem> // YL
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "nesterovBase.h"
#include "placerBase.h"
#include "rsz/Resizer.hh"
#include "sta/Fuzzy.hh"
#include "utl/Logger.h"

namespace gpl {

using utl::GPL;

// TimingBase
TimingBase::TimingBase() = default;

TimingBase::TimingBase(std::shared_ptr<NesterovBaseCommon> nbc,
                       rsz::Resizer* rs,
                       utl::Logger* log)
    : TimingBase()
{
  rs_ = rs;
  nbc_ = std::move(nbc);
  log_ = log;
}

void TimingBase::initTimingOverflowChk()
{
  timingOverflowChk_.clear();
  timingOverflowChk_.resize(timingNetWeightOverflow_.size(), false);
}

bool TimingBase::isTimingNetWeightOverflow(float overflow)
{
  int intOverflow = std::round(overflow * 100);
  // exception case handling
  if (timingNetWeightOverflow_.empty()
      || intOverflow > timingNetWeightOverflow_[0]) {
    return false;
  }

  bool needTdRun = false;
  for (int i = 0; i < timingNetWeightOverflow_.size(); i++) {
    if (timingNetWeightOverflow_[i] > intOverflow) {
      if (!timingOverflowChk_[i]) {
        timingOverflowChk_[i] = true;
        needTdRun = true;
      }
      continue;
    }
    return needTdRun;
  }
  return needTdRun;
}

void TimingBase::addTimingNetWeightOverflow(int overflow)
{
  std::vector<int>::iterator it = std::find(timingNetWeightOverflow_.begin(),
                                            timingNetWeightOverflow_.end(),
                                            overflow);

  // only push overflow when the overflow is not in vector.
  if (it == timingNetWeightOverflow_.end()) {
    timingNetWeightOverflow_.push_back(overflow);
  }

  // do sort in reverse order
  std::sort(timingNetWeightOverflow_.begin(),
            timingNetWeightOverflow_.end(),
            std::greater<int>());
}

void TimingBase::setTimingNetWeightOverflows(std::vector<int>& overflows)
{
  // sort by decreasing order
  std::sort(overflows.begin(), overflows.end(), std::greater<int>());
  for (auto& overflow : overflows) {
    addTimingNetWeightOverflow(overflow);
  }
  initTimingOverflowChk();
}

void TimingBase::deleteTimingNetWeightOverflow(int overflow)
{
  std::vector<int>::iterator it = std::find(timingNetWeightOverflow_.begin(),
                                            timingNetWeightOverflow_.end(),
                                            overflow);
  // only erase overflow when the overflow is in vector.
  if (it != timingNetWeightOverflow_.end()) {
    timingNetWeightOverflow_.erase(it);
  }
}

void TimingBase::clearTimingNetWeightOverflow()
{
  timingNetWeightOverflow_.clear();
}

size_t TimingBase::getTimingNetWeightOverflowSize() const
{
  return timingNetWeightOverflow_.size();
}

void TimingBase::setTimingNetWeightMax(float max)
{
  net_weight_max_ = max;
}

bool TimingBase::executeTimingDriven(bool run_journal_restore)
{
  rs_->findResizeSlacks(run_journal_restore);

  // YL write DEF + verilog
  const char* env_dir = std::getenv("SAVE_DEF_DIR");
  std::string save_directory;

  if (env_dir != nullptr) {
    save_directory = env_dir;
    // Make sure the directory name is ended with '/'
    if (save_directory.back() != '/') {
      save_directory += "/";
    }
  
    if (!std::filesystem::exists(save_directory)) {
        std::filesystem::create_directories(save_directory);
    }

    if (auto* openroad = ord::OpenRoad::openRoad(); openroad != nullptr) {
    static int dump_id = 0;

    // Dump DEF
    std::string def_name = fmt::format(
        "gp_repair_design_iter_{:04d}.def",
        ++dump_id);
    std::string defFilePath = std::string(save_directory) + def_name;
    openroad->writeDef(defFilePath.c_str(), "");
    log_->report("DEF dumped to {}", defFilePath);

    // Dump Verilog
    std::string verilog_name = fmt::format(
        "gp_repair_design_iter_{:04d}.v",
        dump_id);
    std::string verilogFilePath = std::string(save_directory) + verilog_name;
    sta::Network* network = openroad->getSta()->network();
    sta::writeVerilog(
        verilogFilePath.c_str(),
        false,  // sort
        true,  // include_pwr_gnd
        nullptr,  // remove_cells
        network);
  
    } else {
      log_->report("OpenRoad global pointer is null; cannot dump DEF/Verilog.");
    }
  }



  if (!run_journal_restore) {
    nbc_->fixPointers();
  }

  // get worst resize nets
  sta::NetSeq& worst_slack_nets = rs_->resizeWorstSlackNets();

  if (worst_slack_nets.empty()) {
    log_->warn(
        GPL,
        105,
        "Timing-driven: no net slacks found. Timing-driven mode disabled.");
    return false;
  }

  // min/max slack for worst nets
  auto slack_min = rs_->resizeNetSlack(worst_slack_nets[0]).value();
  auto slack_max
      = rs_->resizeNetSlack(worst_slack_nets[worst_slack_nets.size() - 1])
            .value();

  log_->info(GPL, 106, "Timing-driven: worst slack {:.3g}", slack_min);

  if (sta::fuzzyInf(slack_min)) {
    log_->warn(GPL,
               102,
               "Timing-driven: no slacks found. Timing-driven mode disabled.");
    return false;
  }

  int weighted_net_count = 0;
  for (auto& gNet : nbc_->getGNets()) {
    // default weight
    gNet->setTimingWeight(1.0);
    if (gNet->gPins().size() > 1) {
      auto net_slack_opt = rs_->resizeNetSlack(gNet->net()->dbNet());
      if (!net_slack_opt) {
        continue;
      }
      auto net_slack = net_slack_opt.value();
      if (net_slack < slack_max) {
        if (slack_max == slack_min) {
          gNet->setTimingWeight(1.0);
        } else {
          // weight(min_slack) = net_weight_max_
          // weight(max_slack) = 1
          const float weight = 1
                               + (net_weight_max_ - 1) * (slack_max - net_slack)
                                     / (slack_max - slack_min);
          gNet->setTimingWeight(weight);
        }
        weighted_net_count++;
      }
      debugPrint(log_,
                 GPL,
                 "timing",
                 1,
                 "net:{} slack:{} weight:{}",
                 gNet->net()->dbNet()->getConstName(),
                 net_slack,
                 gNet->totalWeight());
    }
  }

  debugPrint(log_,
             GPL,
             "timing",
             1,
             "Timing-driven: weighted {} nets.",
             weighted_net_count);
  return true;
}

}  // namespace gpl
