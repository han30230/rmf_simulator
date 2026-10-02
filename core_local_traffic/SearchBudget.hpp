#ifndef RMF_LOCAL_TRAFFIC_SEARCH_BUDGET_HPP
#define RMF_LOCAL_TRAFFIC_SEARCH_BUDGET_HPP
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
namespace rmf_fleet_adapter { namespace local_traffic {
// Opt-in laboratory budgets: apply to this fleet-adapter PROCESS, not a corridor.
struct SearchBudget
{
  bool enabled = false;
  double compliant_leeway = 3.0, extra_cost = 0.0, seconds = 5.0;
  std::size_t nodes = 10000;
  static SearchBudget load()
  {
    SearchBudget b;
    const char* enabled = std::getenv("RMF_LOCAL_TRAFFIC_BUDGETS");
    if (!enabled || std::string(enabled) != "1") return b;
    b.enabled = true;
    auto value = [](const char* key, double fallback, double lower, double upper)
    {
      const char* text = std::getenv(key);
      if (!text) return fallback;
      std::size_t end;
      const double v = std::stod(text, &end);
      if (end != std::string(text).size() || !std::isfinite(v) || v < lower || v > upper)
        throw std::invalid_argument(std::string("invalid ") + key);
      return v;
    };
    b.compliant_leeway = value("RMF_LOCAL_TRAFFIC_COST_LEEWAY", 10, 1, 100);
    b.extra_cost = value("RMF_LOCAL_TRAFFIC_EXTRA_COST", 120, 0, 3600);
    b.seconds = value("RMF_LOCAL_TRAFFIC_SOLVE_SECONDS", 20, 1, 120);
    const double nodes = value("RMF_LOCAL_TRAFFIC_NODE_LIMIT", 100000, 1000, 1000000);
    if (std::floor(nodes) != nodes) throw std::invalid_argument("node limit must be integral");
    b.nodes = static_cast<std::size_t>(nodes);
    return b;
  }
  std::size_t planning_node_limit(std::optional<std::size_t> requested) const
  { return enabled ? requested.value_or(nodes) : 10000; }
  double compliant_cost(double baseline) const
  { return std::max(compliant_leeway * baseline, baseline + extra_cost); }
};
// Atomic cancellation and an optional STEADY deadline; no nullable comparison.
template<class Time>
bool interrupted(const std::atomic_bool& flag, const std::optional<Time>& deadline, Time now)
{ return flag.load() || (deadline && now >= *deadline); }
}}
#endif
