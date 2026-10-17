// RRT* 경로계획 코어 (ROS 의존성 없음: planning_node와 오프라인 벤치마크가 같은 코드를 사용)
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

namespace planning
{

struct RRTStarParams
{
  // 샘플링 범위 (map 좌표, m)
  float x_min = -1.0f, x_max = 5.5f;
  float y_min = -4.0f, y_max = 5.5f;

  float step_size = 0.1f;         // 확장 거리
  float goal_thresh = 0.2f;       // 목표 도달 기준
  float goal_sample_rate = 0.12f; // 목표점 샘플링 비율 12%
  float rewire_radius = 0.4f;     // choose parent / rewire 반경
  int max_iterations = 10000;
  double time_budget_ms = 200.0;  // 첫 해를 찾은 뒤 경로 개선에 쓰는 최대 시간
  float edge_check_res = 0.025f;  // 선분 충돌 검사 간격 (격자 해상도의 절반)
};

struct RRTStarStats
{
  int iterations = 0;
  size_t tree_size = 0;
  double first_solution_ms = -1.0;
  float first_solution_cost = -1.0f;
  float final_cost = -1.0f;
  double total_ms = 0.0;
};

// is_occupied(x, y) -> bool 을 받아 시작점에서 목표점까지의 노드 경로를 반환 (실패 시 빈 벡터)
template <typename IsOccupied, typename Rng>
std::vector<std::pair<float, float>> rrt_star_plan(
    IsOccupied is_occupied,
    const std::pair<float, float>& start,
    const std::pair<float, float>& goal,
    const RRTStarParams& p,
    Rng& gen,
    RRTStarStats* stats = nullptr,
    std::vector<std::pair<float, float>>* tree_out = nullptr)
{
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  auto elapsed_ms = [&]() {
    return std::chrono::duration<double, std::milli>(clock::now() - t0).count();
  };

  struct Node
  {
    float x, y;
    int parent;
  };
  std::vector<Node> nodes{{start.first, start.second, -1}};
  std::vector<float> cost{0.0f};
  std::vector<std::vector<int>> children(1);
  std::vector<int> goal_nodes;

  std::uniform_real_distribution<float> dis_x(p.x_min, p.x_max);
  std::uniform_real_distribution<float> dis_y(p.y_min, p.y_max);
  std::uniform_real_distribution<float> dis_01(0.0f, 1.0f);

  auto edge_free = [&](float x0, float y0, float x1, float y1) {
    int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(x1 - x0, y1 - y0) / p.edge_check_res)));
    for (int j = 1; j <= steps; ++j) {
      float t = static_cast<float>(j) / steps;
      if (is_occupied(x0 + t * (x1 - x0), y0 + t * (y1 - y0))) return false;
    }
    return true;
  };

  // rewire로 부모가 바뀐 노드의 하위 트리 전체 cost를 delta만큼 감소
  std::vector<int> stack;
  auto propagate_cost = [&](int root, float delta) {
    stack.assign(1, root);
    while (!stack.empty()) {
      int n = stack.back();
      stack.pop_back();
      cost[n] -= delta;
      for (int c : children[n]) stack.push_back(c);
    }
  };

  std::vector<int> near_ids;
  double first_solution_ms = -1.0;
  int it = 0;
  for (; it < p.max_iterations; ++it) {
    // 첫 해를 찾은 뒤에는 시간 예산 동안만 경로 개선
    if (first_solution_ms >= 0.0 && elapsed_ms() - first_solution_ms > p.time_budget_ms) break;

    float rx, ry;
    if (dis_01(gen) < p.goal_sample_rate) {
      rx = goal.first;
      ry = goal.second;
    } else {
      rx = dis_x(gen);
      ry = dis_y(gen);
    }

    // 가장 가까운 노드 찾기
    int nearest = 0;
    float nearest_dist = 1e9f;
    for (size_t j = 0; j < nodes.size(); ++j) {
      float d = std::hypot(rx - nodes[j].x, ry - nodes[j].y);
      if (d < nearest_dist) {
        nearest_dist = d;
        nearest = static_cast<int>(j);
      }
    }
    if (nearest_dist < 1e-6f) continue;

    float nx = nodes[nearest].x + p.step_size * (rx - nodes[nearest].x) / nearest_dist;
    float ny = nodes[nearest].y + p.step_size * (ry - nodes[nearest].y) / nearest_dist;
    if (is_occupied(nx, ny)) continue;

    // ── Choose Parent: 반경 안에서 출발점→새 노드 비용이 가장 작은 부모 선택 ──
    near_ids.clear();
    int best_parent = -1;
    float best_cost = 1e9f;
    for (size_t k = 0; k < nodes.size(); ++k) {
      float d = std::hypot(nodes[k].x - nx, nodes[k].y - ny);
      if (d > p.rewire_radius) continue;
      near_ids.push_back(static_cast<int>(k));
      float c = cost[k] + d;
      if (c < best_cost && edge_free(nodes[k].x, nodes[k].y, nx, ny)) {
        best_cost = c;
        best_parent = static_cast<int>(k);
      }
    }
    if (best_parent < 0) continue;

    int new_idx = static_cast<int>(nodes.size());
    nodes.push_back({nx, ny, best_parent});
    cost.push_back(best_cost);
    children.emplace_back();
    children[best_parent].push_back(new_idx);

    // ── Rewire: 이웃이 새 노드를 거치는 게 더 싸면 부모 교체 (하위 트리 cost도 갱신) ──
    for (int k : near_ids) {
      if (k == best_parent) continue;
      float d = std::hypot(nodes[k].x - nx, nodes[k].y - ny);
      float c = best_cost + d;
      if (c < cost[k] && edge_free(nx, ny, nodes[k].x, nodes[k].y)) {
        auto& siblings = children[nodes[k].parent];
        siblings.erase(std::find(siblings.begin(), siblings.end(), k));
        nodes[k].parent = new_idx;
        children[new_idx].push_back(k);
        propagate_cost(k, cost[k] - c);
      }
    }

    if (std::hypot(nx - goal.first, ny - goal.second) < p.goal_thresh) {
      if (goal_nodes.empty()) {
        first_solution_ms = elapsed_ms();
        if (stats) {
          stats->first_solution_ms = first_solution_ms;
          stats->first_solution_cost = best_cost;
        }
      }
      goal_nodes.push_back(new_idx);
    }
  }

  std::vector<std::pair<float, float>> path;
  if (stats) {
    stats->iterations = it;
    stats->tree_size = nodes.size();
  }
  if (tree_out) {
    tree_out->clear();
    for (const auto& n : nodes) tree_out->emplace_back(n.x, n.y);
  }
  if (!goal_nodes.empty()) {
    // 목표 근처 노드 중 (트리 cost + 목표까지 거리)가 가장 작은 노드로 경로 구성
    int best_goal = goal_nodes.front();
    float best_total = 1e9f;
    for (int g : goal_nodes) {
      float total = cost[g] + std::hypot(nodes[g].x - goal.first, nodes[g].y - goal.second);
      if (total < best_total) {
        best_total = total;
        best_goal = g;
      }
    }
    for (int n = best_goal; n != -1; n = nodes[n].parent) path.emplace_back(nodes[n].x, nodes[n].y);
    std::reverse(path.begin(), path.end());
    if (stats) stats->final_cost = cost[best_goal];
  }
  if (stats) stats->total_ms = elapsed_ms();
  return path;
}

// Path shortcut: 경로의 점 i에서 장애물 없이 직선으로 닿는 가장 먼 점 j로 바로 연결 (사이 점 제거)
// 결과는 꺾이는 점만 남은 경로. 선분 충돌 검사는 check_res 간격으로 샘플링
template <typename IsOccupied>
std::vector<std::pair<float, float>> shortcut_path(
    IsOccupied is_occupied,
    const std::vector<std::pair<float, float>>& path,
    float check_res = 0.025f)
{
  if (path.size() < 3) return path;
  auto segment_free = [&](const std::pair<float, float>& a, const std::pair<float, float>& b) {
    int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(b.first - a.first, b.second - a.second) / check_res)));
    for (int j = 1; j < steps; ++j) {
      float t = static_cast<float>(j) / steps;
      if (is_occupied(a.first + t * (b.first - a.first), a.second + t * (b.second - a.second))) return false;
    }
    return true;
  };

  std::vector<std::pair<float, float>> out{path.front()};
  size_t i = 0;
  while (i < path.size() - 1) {
    size_t j = path.size() - 1;
    while (j > i + 1 && !segment_free(path[i], path[j])) --j;
    out.push_back(path[j]);
    i = j;
  }
  return out;
}

}  // namespace planning
