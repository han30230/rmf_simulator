// Laboratory evaluator: real RMF negotiation; no command application.
#include <rmf_traffic/agv/CentralizedNegotiation.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

namespace rt = rmf_traffic;
namespace agv = rt::agv;
using Clock = std::chrono::steady_clock;
static rt::Duration seconds(double t)
{ return std::chrono::duration_cast<rt::Duration>(std::chrono::duration<double>(t)); }
static double elapsed(rt::Time t, rt::Time t0)
{ return std::chrono::duration<double>(t-t0).count(); }
static void require(bool ok, const std::string& message)
{ if (!ok) throw std::runtime_error(message); }
static void emit(const YAML::Node& out)
{ YAML::Emitter e; e << out; std::cout << e.c_str() << '\n'; }
struct Input
{
  std::string name, goal;
  std::size_t start, destination;
  double yaw;
  bool fixed;
};

int main(int argc, char** argv)
{
  try
  {
    require(argc == 3, "usage: local_traffic_evaluator NAV_GRAPH SCENARIO");
    const auto nav = YAML::LoadFile(argv[1]);
    const auto cfg = YAML::LoadFile(argv[2]);
    require(nav["levels"].size() == 1, "lab evaluator supports one level only");
    const auto level_it = nav["levels"].begin();
    const auto map = level_it->first.as<std::string>();
    const auto level = level_it->second;
    agv::Graph graph;
    std::map<std::string, std::size_t> names;
    for (const auto& v : level["vertices"])
    {
      auto& w = graph.add_waypoint(map, {v[0].as<double>(), v[1].as<double>()});
      const auto name = v[2]["name"].as<std::string>();
      require(names.emplace(name, w.index()).second, "duplicate waypoint name");
      if (v[2]["is_holding_point"]) w.set_holding_point(v[2]["is_holding_point"].as<bool>());
      if (v[2]["is_passthrough_point"]) w.set_passthrough_point(v[2]["is_passthrough_point"].as<bool>());
    }
    for (const auto& lane : level["lanes"])
    {
      const auto a = lane[0].as<std::size_t>(), b = lane[1].as<std::size_t>();
      require(a < graph.num_waypoints() && b < graph.num_waypoints(), "invalid lane endpoint");
      // Fail closed for semantics that the minimal lab parser does not implement.
      if (lane.size() > 2 && lane[2].IsMap())
        for (const auto& property : lane[2])
          require(property.first.as<std::string>() == "graph_idx",
            "lane properties require a full RMF graph parser");
      graph.add_lane(a, b);
    }
    const double radius = cfg["radius"] ? cfg["radius"].as<double>() : 0.7;
    const double budget = cfg["solve_seconds"] ? cfg["solve_seconds"].as<double>() : 10.0;
    const double tail = cfg["tail_seconds"] ? cfg["tail_seconds"].as<double>() : 60.0;
    require(std::isfinite(radius) && radius > 0, "radius must be positive");
    require(std::isfinite(budget) && budget > 0 && budget <= 120, "solve_seconds must be in (0,120]");
    require(std::isfinite(tail) && tail >= 1 && tail <= 3600, "tail_seconds must be in [1,3600]");
    std::set<std::string> robot_names;
    std::vector<Input> input;
    for (const auto& a : cfg["agents"])
    {
      Input i{a["name"].as<std::string>(), a["goal"].as<std::string>(),
        names.at(a["start"].as<std::string>()), names.at(a["goal"].as<std::string>()),
        a["yaw"] ? a["yaw"].as<double>() : 0.0,
        a["fixed"] && a["fixed"].as<bool>()};
      require(robot_names.insert(i.name).second, "duplicate robot name");
      require(std::isfinite(i.yaw), "invalid yaw");
      require(!i.fixed || i.start == i.destination, "fixed robot must have goal=start");
      input.push_back(i);
    }
    require(!input.empty(), "no agents");
    YAML::Node out;
    out["schema_version"] = 1;
    out["backend"] = "rmf_traffic::agv::CentralizedNegotiation";
    for (std::size_t a = 0; a < input.size(); ++a)
      for (std::size_t b = a+1; b < input.size(); ++b)
        if ((graph.get_waypoint(input[a].start).get_location()
           - graph.get_waypoint(input[b].start).get_location()).norm() <= 2*radius)
        {
          out["status"] = "INVALID_INITIAL_OCCUPANCY";
          emit(out); return 0;
        }
    const rt::Profile profile{rt::geometry::make_final_convex<rt::geometry::Circle>(radius)};
    agv::VehicleTraits traits{{1.0, 0.75}, {0.6, 2.0}, profile,
      agv::VehicleTraits::Differential{Eigen::Vector2d::UnitX(), false}};
    const auto t0 = Clock::now(), deadline = t0 + seconds(budget);
    const double node_limit = cfg["node_limit"] ? cfg["node_limit"].as<double>() : 100000;
    const double cost_leeway = cfg["cost_leeway"] ? cfg["cost_leeway"].as<double>() : 10.0;
    const double extra_cost = cfg["extra_cost"] ? cfg["extra_cost"].as<double>() : 120.0;
    require(std::isfinite(node_limit) && node_limit >= 1000 && node_limit <= 1000000
      && std::floor(node_limit) == node_limit, "node_limit must be an integer in [1000,1000000]");
    require(std::isfinite(cost_leeway) && cost_leeway >= 1 && cost_leeway <= 100,
      "cost_leeway must be in [1,100]");
    require(std::isfinite(extra_cost) && extra_cost >= 0 && extra_cost <= 3600,
      "extra_cost must be in [0,3600]");
    agv::Planner::Options options{nullptr};
    options.saturation_limit(static_cast<std::size_t>(node_limit));
    options.interrupter([deadline] { return Clock::now() >= deadline; });
    auto planner = std::make_shared<agv::Planner>(agv::Planner::Configuration{graph, traits}, options);
    auto db = std::make_shared<rt::schedule::Database>();
    std::vector<rt::schedule::Participant> participants;
    participants.reserve(input.size());
    std::vector<agv::CentralizedNegotiation::Agent> agents;
    std::map<rt::schedule::ParticipantId, std::size_t> indexes;
    const auto occupancy_end = t0 + seconds(3600);
    for (std::size_t n = 0; n < input.size(); ++n)
    {
      const auto& i = input[n];
      participants.push_back(rt::schedule::make_participant(
        {i.name, "local_traffic_lab", rt::schedule::ParticipantDescription::Rx::Responsive, profile}, db));
      indexes.emplace(participants.back().id(), n);
      if (i.fixed)
      {
        const auto xy = graph.get_waypoint(i.start).get_location();
        rt::Trajectory stationary;
        stationary.insert(t0, {xy.x(), xy.y(), i.yaw}, Eigen::Vector3d::Zero());
        stationary.insert(occupancy_end, {xy.x(), xy.y(), i.yaw}, Eigen::Vector3d::Zero());
        participants.back().set(participants.back().assign_plan_id(), {rt::Route{map, stationary}});
      }
      else
      {
        agv::SimpleNegotiator::Options neg;
        neg.maximum_cost_leeway(cost_leeway);
        neg.minimum_cost_threshold(extra_cost);
        neg.maximum_alternatives(100);
        agents.emplace_back(participants.back().id(), agv::Plan::Start{t0, i.start, i.yaw},
          agv::Plan::Goal{i.destination}, planner, neg);
      }
    }
    require(!agents.empty(), "at least one movable agent is required");
    agv::CentralizedNegotiation negotiation{db};
    negotiation.log(true);
    auto result = negotiation.solve(agents);
    out["solve_seconds"] = elapsed(Clock::now(), t0);
    out["logs"] = result.log();
    out["blockers"] = std::vector<rt::schedule::ParticipantId>(result.blockers().begin(), result.blockers().end());
    if (!result.proposal())
    {
      out["status"] = Clock::now() >= deadline ? "TIMEOUT" : "NO_PROPOSAL";
      emit(out); return 0;
    }
    if (Clock::now() >= deadline)
    {
      out["status"] = "TIMEOUT"; emit(out); return 0;
    }
    require(result.proposal()->size() == agents.size(), "incomplete proposal");
    rt::Time latest = t0;
    std::vector<rt::Trajectory> paths(input.size());
    for (const auto& entry : *result.proposal())
    {
      const auto n = indexes.at(entry.first);
      for (const auto& route : entry.second.get_itinerary())
        for (const auto& w : route.trajectory())
          paths[n].insert(w.time(), w.position(), w.velocity());
      if (!paths[n].empty()) latest = std::max(latest, paths[n].back().time());
    }
    require(latest + seconds(tail) < occupancy_end, "plan exceeds fixed occupancy horizon");
    for (std::size_t n = 0; n < input.size(); ++n)
    {
      auto& path = paths[n];
      const auto xy = graph.get_waypoint(input[n].start).get_location();
      if (path.empty()) path.insert(t0, {xy.x(), xy.y(), input[n].yaw}, Eigen::Vector3d::Zero());
      require(path.front().time() == t0, "plan omitted initial occupancy");
      const auto goal = graph.get_waypoint(input[n].destination).get_location();
      require((path.back().position().head<2>() - goal).norm() < 1e-5, "plan changed original goal");
      YAML::Node p;
      p["name"] = input[n].name; p["goal"] = input[n].goal;
      p["name"].SetTag("tag:yaml.org,2002:str"); p["goal"].SetTag("tag:yaml.org,2002:str");
      p["finish_seconds"] = elapsed(path.back().time(), t0);
      // Continue to occupy the final pose through the entire group's common horizon.
      path.insert(latest+seconds(tail), path.back().position(), Eigen::Vector3d::Zero());
      for (const auto& w : path)
      {
        YAML::Node point;
        point["t"] = elapsed(w.time(), t0);
        point["x"] = w.position().x(); point["y"] = w.position().y();
        point["yaw"] = w.position().z();
        point["vx"] = w.velocity().x(); point["vy"] = w.velocity().y();
        point["omega"] = w.velocity().z();
        p["trajectory"].push_back(point);
      }
      out["plans"].push_back(p);
    }
    out["horizon_seconds"] = elapsed(latest, t0) + tail;
    out["status"] = "VALID";
    for (std::size_t a=0; a<input.size(); ++a)
      for (std::size_t b=a+1; b<input.size(); ++b)
        if (auto c = rt::DetectConflict::between(profile, paths[a], nullptr, profile, paths[b], nullptr))
        {
          out["status"] = "CONFLICT";
          YAML::Node pair;
          pair["a"] = input[a].name; pair["b"] = input[b].name;
          pair["t"] = elapsed(c->time, t0);
          out["conflicts"].push_back(pair);
        }
    emit(out);
    return 0;
  }
  catch (const std::exception& e)
  { std::cerr << "input/evaluator error: " << e.what() << '\n'; return 2; }
}
