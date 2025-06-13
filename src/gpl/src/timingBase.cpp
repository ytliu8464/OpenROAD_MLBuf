// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2018-2025, The OpenROAD Authors
#include "ord/OpenRoad.hh" // YL 
#include "sta/VerilogWriter.hh" // YL
#include "timingBase.h"
#include <filesystem> // YL
#include <fstream> // YL
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

  // YL: save worst slack nets and their slack to file
  const char* env_slack_save_dir = std::getenv("SAVE_SLACK_DIR");
  if (env_slack_save_dir != nullptr) {
    save_directory = env_slack_save_dir;
    // Make sure the directory name is ended with '/'
    if (save_directory.back() != '/') {
      save_directory += "/";
    }
    static int dump_id = 0;
 
    if (!std::filesystem::exists(save_directory)) {
        std::filesystem::create_directories(save_directory);
    }

    // get worst resize nets
    sta::NetSeq& worst_slack_nets_or = rs_->resizeWorstSlackNets();

    std::string net_slack_file_name = fmt::format(
        "net_slack_iter_{:04d}.tsv",
        ++dump_id);
    std::string slackFilePath = std::string(save_directory) + net_slack_file_name;
    std::ofstream ofs(slackFilePath.c_str(), std::ios::app);
    if (!ofs.is_open() || worst_slack_nets_or.empty()) {
      if (!ofs.is_open()){
        log_->report("Failed to open net_slack_file: {}", slackFilePath);
      }
      else if (worst_slack_nets_or.empty()) {
        log_->report("No worst slack nets found. Skipping net slack save.");
      }
    }
    else{
      ofs << "# net_name\tdriver_slack_or\n";
      for (const sta::Net* net : worst_slack_nets_or) {
        // obtain net name
        auto* openroad = ord::OpenRoad::openRoad();
        auto* db_sta = openroad->getSta();
        auto* db_network = db_sta->getDbNetwork();
        odb::dbNet* db_net = db_network->staToDb(net);
        std::string net_name = db_net->getName();
        auto slack_value = rs_->resizeNetSlack(net).value();


        // save to file
        ofs << net_name << "\t" << slack_value << "\n";
      }
      ofs.close();
      log_->report("Net slacks saved to {}", slackFilePath);
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
