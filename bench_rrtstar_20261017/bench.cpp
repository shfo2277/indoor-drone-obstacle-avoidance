// 오프라인 RRT 벤치마크: 녹화한 /planning_map 위에서 기존 RRT(baseline)와 개선 RRT*를 같은 시드로 비교
// usage: bench <grid.bin> <grid.json> <runs> <budget_ms> <out.csv>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>

#include "planning/rrt_star.hpp"

struct Grid {
  int w, h;
  float res, ox, oy;
  std::vector<int8_t> data;
};

static double json_num(const std::string& s, const std::string& key) {
  auto p = s.find("\"" + key + "\"");
  p = s.find(':', p);
  return std::stod(s.substr(p + 1));
}

// planning.cpp의 is_occupied와 동일
static bool is_occupied(const Grid& g, float x, float y) {
  int mx = static_cast<int>((x - g.ox) / g.res);
  int my = static_cast<int>((y - g.oy) / g.res);
  if (mx < 0 || my < 0 || mx >= g.w || my >= g.h) return true;
  int8_t v = g.data[my * g.w + mx];
  return (v == -1 || v > 50);
}

// ── 기존 planning.cpp generate_path()의 RRT 루프 그대로 (보간 전 노드 경로 반환) ──
static std::vector<std::pair<float, float>> baseline_plan(const Grid& grid, std::pair<float, float> start,
                                                          std::pair<float, float> goal, std::mt19937& gen) {
  struct RRTNode { float x, y; int parent_idx; RRTNode(float a, float b, int c) : x(a), y(b), parent_idx(c) {} };
  std::vector<std::pair<float, float>> path;
  std::vector<RRTNode> nodes;
  std::vector<float> cost;
  nodes.emplace_back(start.first, start.second, -1);
  cost.push_back(0.0f);
  std::uniform_real_distribution<> dis_x(-1.0, 5.5);
  std::uniform_real_distribution<> dis_y(-4.0, 5.5);
  float step_size = 0.1f, goal_thresh = 0.2f, goal_sample_rate = 0.12f, rewire_radius = 0.4f;
  for (int i = 0; i < 10000; ++i) {
    float rx, ry;
    if (std::generate_canonical<float, 10>(gen) < goal_sample_rate) { rx = goal.first; ry = goal.second; }
    else { rx = dis_x(gen); ry = dis_y(gen); }
    float nearest_dist = 1e6; int nearest_idx = -1;
    for (size_t j = 0; j < nodes.size(); ++j) {
      float d = std::hypot(rx - nodes[j].x, ry - nodes[j].y);
      if (d < nearest_dist) { nearest_dist = d; nearest_idx = j; }
    }
    if (nearest_idx == -1) continue;
    float theta = std::atan2(ry - nodes[nearest_idx].y, rx - nodes[nearest_idx].x);
    float nx = nodes[nearest_idx].x + step_size * std::cos(theta);
    float ny = nodes[nearest_idx].y + step_size * std::sin(theta);
    if (is_occupied(grid, nx, ny)) continue;
    float new_cost = cost[nearest_idx] + std::hypot(nx - nodes[nearest_idx].x, ny - nodes[nearest_idx].y);
    int new_idx = nodes.size();
    nodes.emplace_back(nx, ny, nearest_idx);
    cost.push_back(new_cost);
    for (size_t k = 0; k < nodes.size(); ++k) {
      if ((int)k == new_idx) continue;
      float d = std::hypot(nodes[k].x - nx, nodes[k].y - ny);
      if (d < rewire_radius && cost[new_idx] + d < cost[k]) {
        bool col = false;
        for (int j = 1; j <= 25; ++j) {
          if (is_occupied(grid, nx + j * (nodes[k].x - nx) / 25, ny + j * (nodes[k].y - ny) / 25)) { col = true; break; }
        }
        if (!col) { nodes[k].parent_idx = new_idx; cost[k] = cost[new_idx] + d; }
      }
    }
    if (std::hypot(nx - goal.first, ny - goal.second) < goal_thresh) {
      for (int c = new_idx; c != -1; c = nodes[c].parent_idx) path.emplace_back(nodes[c].x, nodes[c].y);
      std::reverse(path.begin(), path.end());
      break;
    }
  }
  return path;
}

static double length(const std::vector<std::pair<float, float>>& p) {
  double L = 0;
  for (size_t i = 1; i < p.size(); ++i) L += std::hypot(p[i].first - p[i - 1].first, p[i].second - p[i - 1].second);
  return L;
}

// 경로 꺾임 정도: 연속 구간 방향 변화량의 합 (rad)
static double total_turn(const std::vector<std::pair<float, float>>& p) {
  double T = 0;
  for (size_t i = 2; i < p.size(); ++i) {
    double a1 = std::atan2(p[i - 1].second - p[i - 2].second, p[i - 1].first - p[i - 2].first);
    double a2 = std::atan2(p[i].second - p[i - 1].second, p[i].first - p[i - 1].first);
    double d = std::fabs(std::remainder(a2 - a1, 2 * M_PI));
    T += d;
  }
  return T;
}

// 경로(1cm 간격 샘플)에서 점유 셀(팽창 포함)까지의 최소 거리
static double min_clearance(const Grid& g, const std::vector<std::pair<float, float>>& occ,
                            const std::vector<std::pair<float, float>>& p) {
  double best = 1e9;
  for (size_t i = 1; i < p.size(); ++i) {
    double seg = std::hypot(p[i].first - p[i - 1].first, p[i].second - p[i - 1].second);
    int n = std::max(1, (int)(seg / 0.01));
    for (int k = 0; k <= n; ++k) {
      double x = p[i - 1].first + (p[i].first - p[i - 1].first) * k / n;
      double y = p[i - 1].second + (p[i].second - p[i - 1].second) * k / n;
      for (const auto& o : occ) best = std::min(best, std::hypot(x - o.first, y - o.second) - g.res / 2);
    }
  }
  return std::max(0.0, best);
}

int main(int argc, char** argv) {
  if (argc < 6) { std::cerr << "usage: bench grid.bin grid.json runs budget_ms out.csv\n"; return 1; }
  Grid g;
  std::stringstream ss; ss << std::ifstream(argv[2]).rdbuf();
  std::string js = ss.str();
  g.w = json_num(js, "width"); g.h = json_num(js, "height"); g.res = json_num(js, "resolution");
  g.ox = json_num(js, "origin_x"); g.oy = json_num(js, "origin_y");
  g.data.resize(g.w * g.h);
  std::ifstream(argv[1], std::ios::binary).read(reinterpret_cast<char*>(g.data.data()), g.data.size());
  int runs = std::stoi(argv[3]);
  double budget = std::stod(argv[4]);

  // 경로 주변(샘플링 영역 + 여유)의 점유 셀 중심 목록 (clearance 계산용)
  std::vector<std::pair<float, float>> occ;
  for (int y = 0; y < g.h; ++y)
    for (int x = 0; x < g.w; ++x) {
      float cx = g.ox + (x + 0.5f) * g.res, cy = g.oy + (y + 0.5f) * g.res;
      if (cx < -2 || cx > 6.5 || cy < -5 || cy > 6.5) continue;
      if (g.data[y * g.w + x] > 50 || g.data[y * g.w + x] == -1) occ.emplace_back(cx, cy);
    }

  std::pair<float, float> start{0.0f, 0.0f}, goal{5.0f, 0.0f};
  planning::RRTStarParams params;
  params.time_budget_ms = budget;

  std::ofstream csv(argv[5]);
  csv << "algo,seed,success,length_m,time_ms,first_ms,first_len_m,nodes,iterations,turn_rad,clearance_m\n";
  for (int s = 0; s < runs; ++s) {
    {
      std::mt19937 gen(s);
      auto t0 = std::chrono::steady_clock::now();
      auto p = baseline_plan(g, start, goal, gen);
      double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      bool ok = !p.empty();
      csv << "baseline," << s << "," << ok << "," << (ok ? length(p) : 0) << "," << ms << "," << ms << ","
          << (ok ? length(p) : 0) << ",," << "," << (ok ? total_turn(p) : 0) << ","
          << (ok ? min_clearance(g, occ, p) : 0) << "\n";
    }
    {
      std::mt19937 gen(s);
      planning::RRTStarStats st;
      auto p = planning::rrt_star_plan([&](float x, float y) { return is_occupied(g, x, y); }, start, goal,
                                       params, gen, &st);
      bool ok = !p.empty();
      csv << "rrtstar," << s << "," << ok << "," << (ok ? length(p) : 0) << "," << st.total_ms << ","
          << st.first_solution_ms << "," << st.first_solution_cost << "," << st.tree_size << "," << st.iterations
          << "," << (ok ? total_turn(p) : 0) << "," << (ok ? min_clearance(g, occ, p) : 0) << "\n";
      if (ok) {
        // planning.cpp와 같은 후처리: 목표점 추가(구간이 비어 있으면) → shortcut
        auto t0 = std::chrono::steady_clock::now();
        auto q = p;
        bool free = true;
        for (int j = 1; j <= 20 && free; ++j) {
          float t = j / 20.0f;
          free = !is_occupied(g, q.back().first + t * (goal.first - q.back().first), q.back().second + t * (goal.second - q.back().second));
        }
        if (free) q.push_back(goal);
        q = planning::shortcut_path([&](float x, float y) { return is_occupied(g, x, y); }, q);
        double sc_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        csv << "rrtstar_shortcut," << s << ",1," << length(q) << "," << st.total_ms + sc_ms << "," << sc_ms << ","
            << q.size() << "," << st.tree_size << "," << st.iterations << "," << total_turn(q) << ","
            << min_clearance(g, occ, q) << "\n";
      }
    }
  }
  std::cerr << "done " << runs << " runs, occupied cells in area: " << occ.size() << "\n";
}
