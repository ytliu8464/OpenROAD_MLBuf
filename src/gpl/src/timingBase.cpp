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
#include <sstream> // YL
#include <map> // YL
#include <unordered_map> // YL
#include <limits> // YL
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
  static int iter_id = 0;
  ++iter_id;
  std::string def_name      = fmt::format("gp_repair_design_iter_{:04d}.def", iter_id);
  std::string verilog_name  = fmt::format("gp_repair_design_iter_{:04d}.v",   iter_id);

  const char* env_dir = std::getenv("SAVE_DEF_DIR");
  std::string save_directory;
  std::string verilogFilePath;
  std::string defFilePath;

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
    // Dump DEF
    defFilePath = std::string(save_directory) + def_name;
    openroad->writeDef(defFilePath.c_str(), "");
    log_->report("DEF dumped to {}", defFilePath);
    // Dump Verilog
    verilogFilePath = std::string(save_directory) + verilog_name;
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
 
    if (!std::filesystem::exists(save_directory)) {
        std::filesystem::create_directories(save_directory);
    }

    std::string net_slack_file_name = fmt::format(
        "net_slack_iter_{:04d}.tsv",
        iter_id);
    std::string slackFilePath = std::string(save_directory) + net_slack_file_name;
    std::ofstream ofs(slackFilePath.c_str());
    // get worst resize nets
    sta::NetSeq& worst_slack_nets_or = rs_->resizeWorstSlackNets();
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
      // for (auto& gNet : nbc_->getGNets()) {
      //   odb::dbNet* db_net = gNet->net()->dbNet();
      //   std::string net_name = db_net->getName();
      //   auto net_slack_opt = rs_->resizeNetSlack(db_net);
      //   if (!net_slack_opt) {
      //     continue;
      //   }
      //   auto slack_value = net_slack_opt.value();
      //   // save to file
      //   ofs << net_name << "\t" << slack_value << "\n";
      // }
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
  
  // YL: load Innovus slacks
  const char* env_invs_slack_save_dir = std::getenv("INVS_SLACK_SAVE_DIR");
  if (env_invs_slack_save_dir != nullptr) {
    save_directory = env_invs_slack_save_dir;
    // Make sure the directory name is ended with '/'
    if (save_directory.back() != '/') {
      save_directory += "/";
    }

    if (!std::filesystem::exists(save_directory)) {
      std::filesystem::create_directories(save_directory);
    }

    std::string invs_slack_file_name = fmt::format(
        "invs_net_slack_iter_{:04d}.tsv",
        iter_id);
    
    //std::string invs_slack_file_name = "invs_net_slack_iter_0003.tsv"; // YL: use fixed name for now
    std::string invsSlackFilePath = std::string(save_directory) + invs_slack_file_name;
    const char* tsv_env = invsSlackFilePath.c_str();  

    // run invs
    ::setenv("NETLIST_NAME",     verilogFilePath.c_str(),   /*overwrite=*/1);
    ::setenv("DEF_NAME",         defFilePath.c_str(),       1);
    ::setenv("SLACK_FILE_NAME",  invsSlackFilePath.c_str(), 1);  
    const char* invs_script =
      "/home/fetzfs_projects/MLBuf/new_project/timer_calibration/"
      "invs_runs/jpeg_encoder/run_invs_slack.sh";

    int ret = std::system(invs_script);      
    if (ret != 0) {
      log_->report("run_invs_slack.sh failed (exit code {})", ret);
      return false; // YL: exit if invs slack calculation failed
    }


    std::unordered_map<std::string, float> invs_slack_tbl;
    if (!run_journal_restore && tsv_env != nullptr) {
      // replace the slacks with those calculated by invs during actual buffering
      bool use_invs_slack = false;
      invs_slack_tbl = loadInvsSlacks(tsv_env);
      use_invs_slack = !invs_slack_tbl.empty();
      if (use_invs_slack){
        rs_->findResizeSlacks1_update(invs_slack_tbl);
      }
    }
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

// -----------------------------------------------
// New functions for timer calibration
// -----------------------------------------------

/// Add '\' before '[' and ']' 
static std::string escapeBrackets(const std::string& s)
{
  std::string out;
  out.reserve(s.size() * 2);         
  for (char c : s) {
    if (c == '[' || c == ']')
      out.push_back('\\');
    out.push_back(c);
  }
  return out;
}


std::unordered_map<std::string, float>
TimingBase::loadInvsSlacks(const std::string& tsv_path)
{
  std::unordered_map<std::string, float> map;
  std::ifstream fin(tsv_path);
  if (!fin) {
    log_->report("Cannot open Innovus slack file '{}'", tsv_path);
    return map;
  }

  std::string line;
  while (std::getline(fin, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream iss(line);
    std::string net;
    float slack;
    if (iss >> net >> slack) {
      const std::string escaped_net = escapeBrackets(net);
      slack /= 1e9; // convert from ns to s
      map[escaped_net] = slack;        // last value wins on duplicates
    }
      
  }

  log_->report("Loaded {} slacks from '{}'", map.size(), tsv_path);
  return map;
}

}  // namespace gpl
