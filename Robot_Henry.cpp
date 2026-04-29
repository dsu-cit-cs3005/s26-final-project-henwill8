#include "RobotBase.h"

#include <algorithm>
#include <climits>
#include <cstdint>
#include <functional>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

/**
 * Decision tree:
 * 1) If a target is locked, keep scanning its lane and shoot while seen.
 * 2) Otherwise move to the closest wall (pathfind on known map).
 * 3) On the wall, patrol toward an edge then reverse; pathfind each straight-line move.
 * 4) Patrol scan cycle: forward wall lane, away from wall, backward wall lane.
 * 5) Maintain an internal grid from radar (free vs blocked); BFS scores candidate moves.
 */
class Robot_Henry : public RobotBase {
private:
    enum Wall { west_wall, east_wall, north_wall, south_wall };

    static constexpr uint8_t kMapUnknown = 0;
    static constexpr uint8_t kMapFree = 1;
    static constexpr uint8_t kMapBlocked = 2;

    Wall patrol_wall_ = west_wall;
    bool wall_selected_ = false;
    bool patrol_forward_ = true;

    int target_row_ = -1;
    int target_col_ = -1;
    bool target_locked_ = false;
    bool saw_robot_this_scan_ = false;
    int target_miss_streak_ = 0;
    int locked_scan_dir_ = 0;

    int last_radar_dir_ = 0;
    std::vector<std::vector<uint8_t>> map_{};
    std::map<std::pair<int, int>, int> dynamic_blocked_ttl_{};

    bool has_prev_loc_ = false;
    int prev_row_ = -1;
    int prev_col_ = -1;
    bool attempted_move_last_turn_ = false;
    int last_move_dir_ = 0;

    int patrol_cycle_ = 0;
    int cached_patrol_move_dir_ = 0;
    int cached_patrol_move_dist_ = 0;

    static constexpr int kTargetMissTolerance_ = 2;
    static constexpr int kDynamicBlockTtl_ = 3;

    static int opposite_dir(int d)
    {
        return (d >= 1 && d <= 8) ? (((d + 3) % 8) + 1) : 0;
    }

    static std::pair<int, int> lateral_vec(int dr, int dc)
    {
        return {dc, -dr};
    }

    void ensure_map()
    {
        if (m_board_row_max <= 0 || m_board_col_max <= 0) {
            return;
        }
        const size_t rows = static_cast<size_t>(m_board_row_max);
        const size_t cols = static_cast<size_t>(m_board_col_max);
        if (map_.size() == rows && !map_.empty() && map_[0].size() == cols) {
            return;
        }
        map_.assign(rows, std::vector<uint8_t>(cols, kMapUnknown));
    }

    bool in_bounds(int r, int c) const
    {
        return r >= 0 && c >= 0 && r < m_board_row_max && c < m_board_col_max;
    }

    bool is_map_blocked(int r, int c) const
    {
        if (!in_bounds(r, c)) {
            return true;
        }
        return map_[static_cast<size_t>(r)][static_cast<size_t>(c)] == kMapBlocked;
    }

    bool walkable(int r, int c) const
    {
        if (!in_bounds(r, c) || is_map_blocked(r, c)) {
            return false;
        }
        return dynamic_blocked_ttl_.count({r, c}) == 0U;
    }

    bool first_step_ok(int cr, int cc, int dir) const
    {
        return walkable(cr + directions[dir].first, cc + directions[dir].second);
    }

    int max_safe_distance_in_dir(int cr, int cc, int dir, int desired) const
    {
        if (dir < 1 || dir > 8 || desired <= 0) {
            return 0;
        }
        int safe = 0;
        const int dr = directions[dir].first;
        const int dc = directions[dir].second;
        for (int step = 1; step <= desired; ++step) {
            const int r = cr + dr * step;
            const int c = cc + dc * step;
            if (!walkable(r, c)) {
                break;
            }
            safe = step;
        }
        return safe;
    }

    void collect_radar_cells(int sr, int sc, int radar_direction, std::vector<std::pair<int, int>>& out) const
    {
        out.clear();
        if (!in_bounds(sr, sc)) {
            return;
        }
        auto push_cell = [&](int r, int c) {
            if (!in_bounds(r, c) || (r == sr && c == sc)) {
                return;
            }
            out.push_back({r, c});
        };

        if (radar_direction == 0) {
            for (int d = 1; d <= 8; ++d) {
                push_cell(sr + directions[d].first, sc + directions[d].second);
            }
            return;
        }
        if (radar_direction < 1 || radar_direction > 8) {
            return;
        }

        const int dr = directions[radar_direction].first;
        const int dc = directions[radar_direction].second;
        const auto lat = lateral_vec(dr, dc);
        const int lr = lat.first;
        const int lc = lat.second;

        for (int t = 1;; ++t) {
            const int cr = sr + t * dr;
            const int cc = sc + t * dc;
            if (!in_bounds(cr, cc)) {
                break;
            }
            push_cell(cr + lr, cc + lc);
            push_cell(cr, cc);
            push_cell(cr - lr, cc - lc);
        }
    }

    void merge_map_from_radar(int sr, int sc, int radar_direction, const std::vector<RadarObj>& radar_results)
    {
        ensure_map();
        if (map_.empty()) {
            return;
        }

        std::vector<std::pair<int, int>> cells;
        collect_radar_cells(sr, sc, radar_direction, cells);

        std::map<std::pair<int, int>, char> hit;
        for (const RadarObj& o : radar_results) {
            hit[{o.m_row, o.m_col}] = o.m_type;
        }

        std::set<std::pair<int, int>> seen;
        for (const auto& rc : cells) {
            if (!seen.insert(rc).second) {
                continue;
            }
            const auto it = hit.find(rc);
            const int r = rc.first;
            const int c = rc.second;
            if (it == hit.end()) {
                map_[static_cast<size_t>(r)][static_cast<size_t>(c)] = kMapFree;
            } else {
                const char t = it->second;
                if (t == 'M' || t == 'P' || t == 'X' || t == 'F') {
                    map_[static_cast<size_t>(r)][static_cast<size_t>(c)] = kMapBlocked;
                } else if (t == 'R') {
                    map_[static_cast<size_t>(r)][static_cast<size_t>(c)] = kMapFree;
                    dynamic_blocked_ttl_[{r, c}] = kDynamicBlockTtl_;
                }
            }
        }
    }

    int wall_distance(Wall w, int r, int c) const
    {
        const int rmax = m_board_row_max - 1;
        const int cmax = m_board_col_max - 1;
        switch (w) {
            case west_wall: return c;
            case east_wall: return cmax - c;
            case north_wall: return r;
            case south_wall: return rmax - r;
        }
        return c;
    }

    void select_closest_wall(int r, int c)
    {
        patrol_wall_ = west_wall;
        int best = wall_distance(west_wall, r, c);
        const Wall walls[3] = {east_wall, north_wall, south_wall};
        for (Wall w : walls) {
            const int d = wall_distance(w, r, c);
            if (d < best) {
                best = d;
                patrol_wall_ = w;
            }
        }
    }

    bool on_patrol_wall(int r, int c) const
    {
        const int rmax = m_board_row_max - 1;
        const int cmax = m_board_col_max - 1;
        switch (patrol_wall_) {
            case west_wall: return c == 0;
            case east_wall: return c == cmax;
            case north_wall: return r == 0;
            case south_wall: return r == rmax;
        }
        return false;
    }

    int away_from_wall_dir() const
    {
        switch (patrol_wall_) {
            case west_wall: return 3;
            case east_wall: return 7;
            case north_wall: return 5;
            case south_wall: return 1;
        }
        return 3;
    }

    int toward_wall_primary_dir() const
    {
        switch (patrol_wall_) {
            case west_wall: return 7;
            case east_wall: return 3;
            case north_wall: return 1;
            case south_wall: return 5;
        }
        return 7;
    }

    int patrol_forward_dir() const
    {
        if (patrol_wall_ == west_wall || patrol_wall_ == east_wall) {
            return patrol_forward_ ? 5 : 1;
        }
        return patrol_forward_ ? 3 : 7;
    }

    int patrol_backward_dir() const
    {
        return opposite_dir(patrol_forward_dir());
    }

    void maybe_flip_at_wall_edge(int cr, int cc)
    {
        if (!on_patrol_wall(cr, cc)) {
            return;
        }
        const int rmax = m_board_row_max - 1;
        const int cmax = m_board_col_max - 1;
        int to_end = 0;
        if (patrol_wall_ == west_wall || patrol_wall_ == east_wall) {
            to_end = patrol_forward_ ? (rmax - cr) : cr;
        } else {
            to_end = patrol_forward_ ? (cmax - cc) : cc;
        }
        if (to_end == 0) {
            patrol_forward_ = !patrol_forward_;
            return;
        }
        if (to_end == 1) {
            const int d = patrol_forward_dir();
            const int nr = cr + directions[d].first;
            const int nc = cc + directions[d].second;
            if (!in_bounds(nr, nc) || !walkable(nr, nc)) {
                patrol_forward_ = !patrol_forward_;
            }
        }
    }

    void wall_anchor_on_patrol_wall(int cr, int cc, int& ar, int& ac) const
    {
        const int rmax = m_board_row_max - 1;
        const int cmax = m_board_col_max - 1;
        switch (patrol_wall_) {
            case west_wall:
                ar = cr;
                ac = 0;
                return;
            case east_wall:
                ar = cr;
                ac = cmax;
                return;
            case north_wall:
                ar = 0;
                ac = cc;
                return;
            case south_wall:
                ar = rmax;
                ac = cc;
                return;
        }
        ar = cr;
        ac = cc;
    }

    bool patrol_ahead_zone_nonempty_from(int ar, int ac) const
    {
        const int rmax = m_board_row_max - 1;
        const int cmax = m_board_col_max - 1;
        if (patrol_wall_ == west_wall) {
            for (int r = 0; r <= rmax; ++r) {
                if (in_patrol_ahead_zone(r, 0, ar, ac)) {
                    return true;
                }
            }
        } else if (patrol_wall_ == east_wall) {
            for (int r = 0; r <= rmax; ++r) {
                if (in_patrol_ahead_zone(r, cmax, ar, ac)) {
                    return true;
                }
            }
        } else if (patrol_wall_ == north_wall) {
            for (int c = 0; c <= cmax; ++c) {
                if (in_patrol_ahead_zone(0, c, ar, ac)) {
                    return true;
                }
            }
        } else {
            for (int c = 0; c <= cmax; ++c) {
                if (in_patrol_ahead_zone(rmax, c, ar, ac)) {
                    return true;
                }
            }
        }
        return false;
    }

    void sync_patrol_forward_from_wall_move(int ocr, int occ, int ncr, int ncc)
    {
        if (!on_patrol_wall(ocr, occ) || !on_patrol_wall(ncr, ncc)) {
            return;
        }
        if (ocr == ncr && occ == ncc) {
            return;
        }
        if (patrol_wall_ == west_wall || patrol_wall_ == east_wall) {
            if (ncr > ocr) {
                patrol_forward_ = true;
            } else if (ncr < ocr) {
                patrol_forward_ = false;
            }
            return;
        }
        if (ncc > occ) {
            patrol_forward_ = true;
        } else if (ncc < occ) {
            patrol_forward_ = false;
        }
    }

    bool in_patrol_ahead_zone(int r, int c, int from_r, int from_c) const
    {
        if (!on_patrol_wall(r, c)) {
            return false;
        }
        const int rmax = m_board_row_max - 1;
        const int cmax = m_board_col_max - 1;
        if (patrol_wall_ == west_wall) {
            if (c != 0) {
                return false;
            }
            return patrol_forward_ ? (r > from_r) : (r < from_r);
        }
        if (patrol_wall_ == east_wall) {
            if (c != cmax) {
                return false;
            }
            return patrol_forward_ ? (r > from_r) : (r < from_r);
        }
        if (patrol_wall_ == north_wall) {
            if (r != 0) {
                return false;
            }
            return patrol_forward_ ? (c > from_c) : (c < from_c);
        }
        if (r != rmax) {
            return false;
        }
        return patrol_forward_ ? (c > from_c) : (c < from_c);
    }

    int bfs_min_dist_to_predicate(int sr, int sc, const std::function<bool(int, int)>& goal) const
    {
        if (!in_bounds(sr, sc) || !walkable(sr, sc)) {
            return INT_MAX;
        }
        if (goal(sr, sc)) {
            return 0;
        }
        const int rows = m_board_row_max;
        const int cols = m_board_col_max;
        std::vector<int> dist(static_cast<size_t>(rows * cols), INT_MAX);
        const int start = sr * cols + sc;
        dist[static_cast<size_t>(start)] = 0;
        std::queue<int> q;
        q.push(start);

        while (!q.empty()) {
            const int cur = q.front();
            q.pop();
            const int cr = cur / cols;
            const int cc = cur % cols;
            const int d0 = dist[static_cast<size_t>(cur)];
            for (int nd = 1; nd <= 8; ++nd) {
                const int nr = cr + directions[nd].first;
                const int nc = cc + directions[nd].second;
                if (!walkable(nr, nc)) {
                    continue;
                }
                const int ni = nr * cols + nc;
                if (dist[static_cast<size_t>(ni)] != INT_MAX) {
                    continue;
                }
                dist[static_cast<size_t>(ni)] = d0 + 1;
                if (goal(nr, nc)) {
                    return d0 + 1;
                }
                q.push(ni);
            }
        }
        return INT_MAX;
    }

    int patrol_forward_step_bonus(int cr, int cc, int er, int ec, int move_dir, int move_len) const
    {
        const int pf = patrol_forward_dir();
        if (move_dir != pf) {
            return 0;
        }
        if (patrol_wall_ == west_wall || patrol_wall_ == east_wall) {
            return std::abs(er - cr);
        }
        return std::abs(ec - cc);
    }

    bool choose_straight_move_by_bfs(int cr, int cc, const std::function<bool(int, int)>& goal_at_end, int& dir, int& dist)
    {
        const int speed = get_move_speed();
        if (speed <= 0) {
            return false;
        }
        int best_dir = 0;
        int best_len = 0;
        int best_score = INT_MAX;
        int best_bonus = -1;

        for (int d = 1; d <= 8; ++d) {
            const int max_len = max_safe_distance_in_dir(cr, cc, d, speed);
            for (int k = 1; k <= max_len; ++k) {
                const int er = cr + directions[d].first * k;
                const int ec = cc + directions[d].second * k;
                const int score = bfs_min_dist_to_predicate(er, ec, [&](int r, int c) { return goal_at_end(r, c); });
                if (score == INT_MAX) {
                    continue;
                }
                const int bonus = patrol_forward_step_bonus(cr, cc, er, ec, d, k);
                if (score < best_score || (score == best_score && bonus > best_bonus)) {
                    best_score = score;
                    best_bonus = bonus;
                    best_dir = d;
                    best_len = k;
                }
            }
        }

        if (best_dir == 0) {
            return false;
        }
        dir = best_dir;
        dist = best_len;
        return true;
    }

    bool choose_move_to_wall(int cr, int cc, int& dir, int& dist)
    {
        int ar = 0;
        int ac = 0;
        wall_anchor_on_patrol_wall(cr, cc, ar, ac);
        if (patrol_ahead_zone_nonempty_from(ar, ac)) {
            if (choose_straight_move_by_bfs(cr, cc,
                    [&](int r, int c) { return in_patrol_ahead_zone(r, c, ar, ac); }, dir, dist)) {
                return true;
            }
        }
        return choose_straight_move_by_bfs(cr, cc, [&](int r, int c) { return on_patrol_wall(r, c); }, dir, dist);
    }

    bool choose_patrol_step(int cr, int cc, int& dir, int& dist)
    {
        return choose_straight_move_by_bfs(cr, cc,
            [&](int r, int c) { return in_patrol_ahead_zone(r, c, cr, cc); }, dir, dist);
    }

    int best_wall_allowed_scan_dir_toward_target(int sr, int sc, int tr, int tc) const
    {
        const int cand[3] = {patrol_forward_dir(), patrol_backward_dir(), away_from_wall_dir()};
        int best_dir = cand[0];
        int best_dot = INT_MIN;
        const int vr = tr - sr;
        const int vc = tc - sc;
        for (int d : cand) {
            const int dot = vr * directions[d].first + vc * directions[d].second;
            if (dot > best_dot) {
                best_dot = dot;
                best_dir = d;
            }
        }
        return best_dir;
    }

    void age_dynamic_blockers()
    {
        for (auto it = dynamic_blocked_ttl_.begin(); it != dynamic_blocked_ttl_.end();) {
            if (--(it->second) <= 0) {
                it = dynamic_blocked_ttl_.erase(it);
            } else {
                ++it;
            }
        }
    }

    void infer_blocked_move_if_any(int cr, int cc)
    {
        if (has_prev_loc_ && attempted_move_last_turn_ && last_move_dir_ >= 1 && last_move_dir_ <= 8 && cr == prev_row_ && cc == prev_col_) {
            const int br = cr + directions[last_move_dir_].first;
            const int bc = cc + directions[last_move_dir_].second;
            if (in_bounds(br, bc) && !is_map_blocked(br, bc)) {
                dynamic_blocked_ttl_[{br, bc}] = kDynamicBlockTtl_;
            }
        }
        prev_row_ = cr;
        prev_col_ = cc;
        has_prev_loc_ = true;
        attempted_move_last_turn_ = false;
        last_move_dir_ = 0;
        age_dynamic_blockers();
    }

public:
    Robot_Henry() : RobotBase(2, 5, railgun)
    {
        m_name = "Henry";
        m_character = 'H';
    }

    void get_radar_direction(int& radar_direction) override
    {
        int cr = 0;
        int cc = 0;
        get_current_location(cr, cc);
        ensure_map();
        infer_blocked_move_if_any(cr, cc);

        if (!wall_selected_) {
            select_closest_wall(cr, cc);
            wall_selected_ = true;
        }

        if (target_locked_) {
            patrol_cycle_ = 0;
            if (locked_scan_dir_ < 1 || locked_scan_dir_ > 8) {
                locked_scan_dir_ = best_wall_allowed_scan_dir_toward_target(cr, cc, target_row_, target_col_);
            }
            last_radar_dir_ = locked_scan_dir_;
            radar_direction = locked_scan_dir_;
            return;
        }

        int planned_dir = 0;
        int planned_dist = 0;
        if (!on_patrol_wall(cr, cc)) {
            choose_move_to_wall(cr, cc, planned_dir, planned_dist);
            cached_patrol_move_dir_ = planned_dir;
            cached_patrol_move_dist_ = planned_dist;
            patrol_cycle_ = 0;
            last_radar_dir_ = (planned_dir >= 1 && planned_dir <= 8) ? planned_dir : 0;
            radar_direction = last_radar_dir_;
            return;
        }

        maybe_flip_at_wall_edge(cr, cc);
        choose_patrol_step(cr, cc, planned_dir, planned_dist);
        cached_patrol_move_dir_ = planned_dir;
        cached_patrol_move_dist_ = planned_dist;

        if (patrol_cycle_ == 0) {
            last_radar_dir_ = patrol_forward_dir();
        } else if (patrol_cycle_ == 1) {
            last_radar_dir_ = away_from_wall_dir();
        } else {
            last_radar_dir_ = patrol_backward_dir();
        }
        radar_direction = last_radar_dir_;
    }

    void process_radar_results(const std::vector<RadarObj>& radar_results) override
    {
        int cr = 0;
        int cc = 0;
        get_current_location(cr, cc);
        merge_map_from_radar(cr, cc, last_radar_dir_, radar_results);

        int nearest_r = -1;
        int nearest_c = -1;
        int best = INT_MAX;
        bool lock_seen = false;
        for (const RadarObj& o : radar_results) {
            if (o.m_type != 'R') {
                continue;
            }
            if (target_locked_ && o.m_row == target_row_ && o.m_col == target_col_) {
                lock_seen = true;
            }
            const int d = std::abs(o.m_row - cr) + std::abs(o.m_col - cc);
            if (d < best) {
                best = d;
                nearest_r = o.m_row;
                nearest_c = o.m_col;
            }
        }

        if (target_locked_ && lock_seen) {
            target_miss_streak_ = 0;
            saw_robot_this_scan_ = true;
            return;
        }

        if (nearest_r >= 0) {
            target_locked_ = true;
            target_row_ = nearest_r;
            target_col_ = nearest_c;
            locked_scan_dir_ = best_wall_allowed_scan_dir_toward_target(cr, cc, target_row_, target_col_);
            target_miss_streak_ = 0;
            saw_robot_this_scan_ = true;
            patrol_cycle_ = 0;
            return;
        }

        if (target_locked_ && target_miss_streak_ < kTargetMissTolerance_) {
            ++target_miss_streak_;
            saw_robot_this_scan_ = false;
            return;
        }

        target_locked_ = false;
        target_row_ = -1;
        target_col_ = -1;
        locked_scan_dir_ = 0;
        target_miss_streak_ = 0;
        saw_robot_this_scan_ = false;
    }

    bool get_shot_location(int& shot_row, int& shot_col) override
    {
        if (!target_locked_ || !saw_robot_this_scan_) {
            return false;
        }
        shot_row = target_row_;
        shot_col = target_col_;
        return true;
    }

    void get_move_direction(int& direction, int& distance) override
    {
        direction = 0;
        distance = 0;

        if (get_move_speed() <= 0) {
            attempted_move_last_turn_ = false;
            return;
        }

        int cr = 0;
        int cc = 0;
        get_current_location(cr, cc);
        ensure_map();

        if (!wall_selected_) {
            select_closest_wall(cr, cc);
            wall_selected_ = true;
        }

        if (target_locked_ || saw_robot_this_scan_) {
            attempted_move_last_turn_ = false;
            patrol_cycle_ = 0;
            return;
        }

        if (!on_patrol_wall(cr, cc)) {
            if (choose_move_to_wall(cr, cc, direction, distance)) {
                attempted_move_last_turn_ = true;
                last_move_dir_ = direction;
            } else {
                attempted_move_last_turn_ = false;
            }
            return;
        }

        if (patrol_cycle_ == 0) {
            const int ocr = cr;
            const int occ = cc;
            direction = cached_patrol_move_dir_;
            distance = cached_patrol_move_dist_;
            if (direction < 1 || direction > 8 || distance <= 0) {
                if (!choose_patrol_step(cr, cc, direction, distance)) {
                    direction = 0;
                    distance = 0;
                }
            }

            if (direction >= 1 && direction <= 8 && distance > 0) {
                const int ncr = cr + directions[direction].first * distance;
                const int ncc = cc + directions[direction].second * distance;
                if (on_patrol_wall(ocr, occ) && on_patrol_wall(ncr, ncc)) {
                    sync_patrol_forward_from_wall_move(ocr, occ, ncr, ncc);
                } else {
                    if (patrol_wall_ == west_wall || patrol_wall_ == east_wall) {
                        if (direction == 5) {
                            patrol_forward_ = true;
                        } else if (direction == 1) {
                            patrol_forward_ = false;
                        }
                    } else {
                        if (direction == 3) {
                            patrol_forward_ = true;
                        } else if (direction == 7) {
                            patrol_forward_ = false;
                        }
                    }
                }
            }

            attempted_move_last_turn_ = (direction >= 1 && direction <= 8 && distance > 0);
            last_move_dir_ = direction;
            patrol_cycle_ = 1;
            return;
        }

        attempted_move_last_turn_ = false;
        patrol_cycle_ = (patrol_cycle_ + 1) % 3;
    }
};

extern "C" RobotBase* create_robot()
{
    return new Robot_Henry();
}

extern "C" const char* robot_summary()
{
    return "Patrols along one wall, snipes with railgun.";
}
