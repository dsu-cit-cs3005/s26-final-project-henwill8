#include "Arena.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <thread>

#include <dlfcn.h>

namespace fs = std::filesystem;

namespace {
constexpr std::size_t kMaxRobotSummaryChars = 50;

using RobotSummaryFn = const char* (*)();
} // namespace

Arena::~Arena()
{
    for (LoadedRobot& lr : robots_) {
        delete lr.robot;
        lr.robot = nullptr;
        if (lr.dl_handle != nullptr) {
            dlclose(lr.dl_handle);
            lr.dl_handle = nullptr;
        }
    }
    robots_.clear();
}

namespace {

constexpr int kRailDamageMin = 10;
constexpr int kRailDamageMax = 20;
constexpr int kHammerDamageMin = 50;
constexpr int kHammerDamageMax = 60;
constexpr int kGrenadeDamageMin = 10;
constexpr int kGrenadeDamageMax = 40;
constexpr int kFlameDamageMin = 30;
constexpr int kFlameDamageMax = 50;

const char* weapon_label(WeaponType w)
{
    switch (w) {
        case railgun:
            return "railgun";
        case flamethrower:
            return "flamethrower";
        case grenade:
            return "grenade";
        case hammer:
            return "hammer";
        default:
            return "weapon";
    }
}

char printable_robot_char(char ch)
{
    const auto uch = static_cast<unsigned char>(ch);
    return std::isprint(uch) ? ch : '?';
}

std::string trim(std::string s)
{
    auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

std::pair<int, int> lateral_vec(int dr, int dc)
{
    return {dc, -dr};
}

bool parse_bool(std::string v)
{
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v == "true" || v == "1" || v == "yes";
}

/** Bresenham line in row/col space; returns inclusive cells from (r0,c0) to (r1,c1). */
std::vector<std::pair<int, int>> bresenham_line(int r0, int c0, int r1, int c1)
{
    std::vector<std::pair<int, int>> out;
    int x0 = c0;
    int y0 = r0;
    int x1 = c1;
    int y1 = r1;

    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    int x = x0;
    int y = y0;

    for (;;) {
        out.push_back({y, x});
        if (x == x1 && y == y1) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }

    return out;
}

int nearest_ray_direction(int sr, int sc, int tr, int tc)
{
    const int vr = tr - sr;
    const int vc = tc - sc;
    int best = 1;
    int best_dot = INT_MIN;
    for (int d = 1; d <= 8; ++d) {
        const int dr = directions[d].first;
        const int dc = directions[d].second;
        const int dot = vr * dr + vc * dc;
        if (dot > best_dot) {
            best_dot = dot;
            best = d;
        }
    }
    return best;
}

} // namespace

bool Arena::load_config(const std::string& config_path)
{
    std::ifstream in(config_path);
    if (!in) {
        std::cerr << "Cannot open config: " << config_path << '\n';
        return false;
    }

    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        if (!parse_config_line(line)) {
            std::cerr << "Bad config line: " << line << '\n';
            return false;
        }
    }

    if (height_ <= 0 || width_ <= 0) {
        std::cerr << "Arena_Size must be set with positive Height Width.\n";
        return false;
    }

    obstacles_.assign(static_cast<size_t>(height_), std::vector<char>(static_cast<size_t>(width_), '.'));
    return true;
}

bool Arena::parse_config_line(const std::string& line)
{
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
        return false;
    }

    std::string key = trim(line.substr(0, colon));
    std::string rest = trim(line.substr(colon + 1));

    std::istringstream iss(rest);
    if (key == "Arena_Size") {
        int h = 0;
        int w = 0;
        if (!(iss >> h >> w)) {
            return false;
        }
        height_ = h;
        width_ = w;
        return true;
    }
    if (key == "Max_Rounds") {
        if (!(iss >> max_rounds_)) {
            return false;
        }
        return true;
    }
    if (key == "Sleep_interval") {
        if (!(iss >> sleep_seconds_)) {
            return false;
        }
        return true;
    }
    if (key == "Game_State_Live") {
        game_live_ = parse_bool(rest);
        return true;
    }
    if (key == "Flamethrowers") {
        if (!(iss >> flamethrowers_)) {
            return false;
        }
        return true;
    }
    if (key == "Pits") {
        if (!(iss >> pits_)) {
            return false;
        }
        return true;
    }
    if (key == "Mounds") {
        if (!(iss >> mounds_)) {
            return false;
        }
        return true;
    }

    return false;
}

bool Arena::in_bounds(int row, int col) const
{
    return row >= 0 && col >= 0 && row < height_ && col < width_;
}

char Arena::obstacle_at(int row, int col) const
{
    return obstacles_[static_cast<size_t>(row)][static_cast<size_t>(col)];
}

bool Arena::is_alive(RobotBase* r) const
{
    return r != nullptr && r->get_health() > 0;
}

void Arena::maybe_sleep() const
{
    if (sleep_seconds_ > 0.0 && game_live_) {
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_seconds_));
    }
}

std::vector<std::string> Arena::discover_robot_sources() const
{
    std::vector<std::string> out;

    auto scan_dir = [&](const fs::path& dir) {
        if (!fs::exists(dir) || !fs::is_directory(dir)) {
            return;
        }
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            const auto name = entry.path().filename().string();
            if (name.size() > 9 && name.substr(0, 6) == "Robot_" && name.substr(name.size() - 4) == ".cpp") {
                out.push_back(entry.path().string());
            }
        }
    };

    scan_dir("robots");
    scan_dir(".");

    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

bool Arena::compile_and_load_robots()
{
    const auto sources = discover_robot_sources();
    if (sources.empty()) {
        std::cerr << "No Robot_*.cpp files found under ./robots or .\n";
        return false;
    }

    for (const std::string& cpp_path : sources) {
        fs::path p(cpp_path);
        const std::string stem = p.stem().string();
        const fs::path so_path = p.parent_path() / ("lib" + stem + ".so");

        std::ostringstream cmd;
        cmd << "g++ -shared -fPIC -o ";
        cmd << std::quoted(so_path.string());
        cmd << ' ' << std::quoted(cpp_path);
        cmd << " RobotBase.o -I. -std=c++20";

        std::cout << "Compiling " << cpp_path << " -> " << so_path.string() << '\n';
        const int rc = std::system(cmd.str().c_str());
        if (rc != 0) {
            std::cerr << "Compile failed for " << cpp_path << '\n';
            continue;
        }

        const std::string abs_so = fs::absolute(so_path).string();
        void* handle = dlopen(abs_so.c_str(), RTLD_LAZY);
        if (!handle) {
            std::cerr << "dlopen failed " << so_path << ": " << dlerror() << '\n';
            continue;
        }

        auto create = reinterpret_cast<RobotFactory>(dlsym(handle, "create_robot"));
        if (!create) {
            std::cerr << "Missing create_robot in " << so_path << '\n';
            dlclose(handle);
            continue;
        }

        auto summary_fn = reinterpret_cast<RobotSummaryFn>(dlsym(handle, "robot_summary"));
        if (!summary_fn) {
            std::cerr << "Missing robot_summary in " << so_path << ": " << dlerror() << '\n';
            dlclose(handle);
            continue;
        }
        const char* summary_text = summary_fn();
        if (!summary_text) {
            std::cerr << "robot_summary returned null for " << so_path << '\n';
            dlclose(handle);
            continue;
        }
        const std::size_t summary_len = std::strlen(summary_text);
        if (summary_len == 0 || summary_len > kMaxRobotSummaryChars) {
            std::cerr << "Invalid robot_summary length (" << summary_len << ") for " << so_path
                      << ". Required: 1-" << kMaxRobotSummaryChars << " chars.\n";
            dlclose(handle);
            continue;
        }
        std::cout << "  robot_summary: " << summary_text << '\n';

        RobotBase* robot = create();
        if (!robot) {
            std::cerr << "create_robot returned null for " << so_path << '\n';
            dlclose(handle);
            continue;
        }

        LoadedRobot lr;
        lr.robot = robot;
        lr.dl_handle = handle;
        lr.source_file = cpp_path;
        robots_.push_back(lr);
    }

    if (robots_.empty()) {
        std::cerr << "No robots loaded.\n";
        return false;
    }

    return true;
}

void Arena::place_obstacles_random()
{
    std::uniform_int_distribution<int> rr(0, height_ - 1);
    std::uniform_int_distribution<int> cc(0, width_ - 1);

    auto place_kind = [&](char kind, int count) {
        int placed = 0;
        int guard = 0;
        while (placed < count && guard < count * 10000) {
            ++guard;
            const int r = rr(rng_);
            const int c = cc(rng_);
            if (obstacles_[static_cast<size_t>(r)][static_cast<size_t>(c)] != '.') {
                continue;
            }
            obstacles_[static_cast<size_t>(r)][static_cast<size_t>(c)] = kind;
            ++placed;
        }
    };

    place_kind('F', flamethrowers_);
    place_kind('P', pits_);
    place_kind('M', mounds_);
}

void Arena::place_robots_random()
{
    std::uniform_int_distribution<int> rr(0, height_ - 1);
    std::uniform_int_distribution<int> cc(0, width_ - 1);

    for (auto& lr : robots_) {
        RobotBase* bot = lr.robot;
        bot->set_boundaries(height_, width_);

        int guard = 0;
        while (guard < 100000) {
            ++guard;
            const int r = rr(rng_);
            const int c = cc(rng_);
            if (obstacles_[static_cast<size_t>(r)][static_cast<size_t>(c)] != '.') {
                continue;
            }
            if (robot_index_at(r, c) >= 0) {
                continue;
            }
            bot->move_to(r, c);
            break;
        }
    }
}

int Arena::robot_index_at(int row, int col)
{
    for (size_t i = 0; i < robots_.size(); ++i) {
        RobotBase* r = robots_[i].robot;
        int rr = 0;
        int cc = 0;
        r->get_current_location(rr, cc);
        if (rr == row && cc == col) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::vector<RadarObj> Arena::scan_radar(RobotBase& observer, int radar_direction)
{
    std::vector<RadarObj> results;

    int sr = 0;
    int sc = 0;
    observer.get_current_location(sr, sc);

    auto append_cell = [&](int r, int c) {
        if (!in_bounds(r, c)) {
            return;
        }
        if (r == sr && c == sc) {
            return;
        }

        const int idx = robot_index_at(r, c);
        if (idx >= 0) {
            RobotBase* other = robots_[static_cast<size_t>(idx)].robot;
            const char type = is_alive(other) ? 'R' : 'X';
            results.emplace_back(type, r, c);
            return;
        }

        const char o = obstacle_at(r, c);
        if (o != '.') {
            results.emplace_back(o, r, c);
        }
    };

    if (radar_direction == 0) {
        for (int d = 1; d <= 8; ++d) {
            const int dr = directions[d].first;
            const int dc = directions[d].second;
            append_cell(sr + dr, sc + dc);
        }
        return results;
    }

    if (radar_direction < 1 || radar_direction > 8) {
        return results;
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

        append_cell(cr + lr, cc + lc);
        append_cell(cr, cc);
        append_cell(cr - lr, cc - lc);
    }

    return results;
}

int Arena::apply_damage(RobotBase& target, int raw_min, int raw_max)
{
    std::uniform_int_distribution<int> dist(raw_min, raw_max);
    int raw = dist(rng_);
    const int armor = target.get_armor();
    const double factor = 1.0 - 0.10 * static_cast<double>(armor);
    int mitigated = static_cast<int>(std::floor(static_cast<double>(raw) * factor));
    if (mitigated < 0) {
        mitigated = 0;
    }
    const int before = target.get_health();
    target.take_damage(mitigated);
    if (mitigated > 0) {
        target.reduce_armor(1);
    }
    return before - target.get_health();
}

void Arena::resolve_move(RobotBase& robot, int robot_index, int direction, int distance)
{
    if (!is_alive(&robot)) {
        return;
    }

    int max_speed = robot.get_move_speed();
    if (max_speed <= 0) {
        return;
    }

    if (direction < 1 || direction > 8) {
        return;
    }

    int steps = std::min(distance, max_speed);
    if (steps <= 0) {
        return;
    }

    const int dr = directions[direction].first;
    const int dc = directions[direction].second;

    int r = 0;
    int c = 0;
    robot.get_current_location(r, c);

    for (int s = 0; s < steps; ++s) {
        const int nr = r + dr;
        const int nc = c + dc;

        if (!in_bounds(nr, nc)) {
            break;
        }

        const int occ = robot_index_at(nr, nc);
        if (occ >= 0 && occ != robot_index) {
            break;
        }

        const char tile = obstacle_at(nr, nc);
        if (tile == 'M') {
            break;
        }

        if (tile == 'F') {
            const int flame_loss = apply_damage(robot, kFlameDamageMin, kFlameDamageMax);
            if (flame_loss > 0) {
                std::cout << "    (Flame tile: -" << flame_loss << " HP)\n";
            }
            robot.move_to(nr, nc);
            r = nr;
            c = nc;
            if (!is_alive(&robot)) {
                obstacles_[static_cast<size_t>(nr)][static_cast<size_t>(nc)] = '.';
                break;
            }
            continue;
        }

        if (tile == 'P') {
            robot.move_to(nr, nc);
            robot.disable_movement();
            break;
        }

        robot.move_to(nr, nc);
        r = nr;
        c = nc;
    }
}

bool Arena::resolve_shoot(RobotBase& shooter, int shooter_index, int shot_row, int shot_col)
{
    if (!is_alive(&shooter)) {
        return false;
    }

    const WeaponType w = shooter.get_weapon();

    int sr = 0;
    int sc = 0;
    shooter.get_current_location(sr, sc);

    std::set<std::pair<int, int>> hit_cells;
    auto add_hit = [&](int r, int c) {
        if (!in_bounds(r, c)) {
            return;
        }
        hit_cells.insert({r, c});
    };

    if (w == hammer) {
        const int dir = nearest_ray_direction(sr, sc, shot_row, shot_col);
        const int dr = directions[dir].first;
        const int dc = directions[dir].second;
        add_hit(sr + dr, sc + dc);
    } else if (w == grenade) {
        if (shooter.get_grenades() <= 0) {
            return false;
        }
        if (!in_bounds(shot_row, shot_col)) {
            return false;
        }
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                add_hit(shot_row + dr, shot_col + dc);
            }
        }
        shooter.decrement_grenades();
    } else if (w == flamethrower) {
        const int dir = nearest_ray_direction(sr, sc, shot_row, shot_col);
        const int fdr = directions[dir].first;
        const int fdc = directions[dir].second;
        const auto lat = lateral_vec(fdr, fdc);
        const int lr = lat.first;
        const int lc = lat.second;

        for (int t = 1; t <= 4; ++t) {
            const int cr = sr + t * fdr;
            const int cc = sc + t * fdc;
            if (!in_bounds(cr, cc)) {
                break;
            }
            add_hit(cr + lr, cc + lc);
            add_hit(cr, cc);
            add_hit(cr - lr, cc - lc);
        }
    } else if (w == railgun) {
        if (sr == shot_row && sc == shot_col) {
            return false;
        }

        auto line = bresenham_line(sr, sc, shot_row, shot_col);
        if (line.size() < 2) {
            return false;
        }

        for (size_t i = 1; i < line.size(); ++i) {
            add_hit(line[i].first, line[i].second);
        }

        const auto prev = line[line.size() - 2];
        const auto last = line.back();
        int step_r = last.first - prev.first;
        int step_c = last.second - prev.second;

        auto cur = last;
        for (;;) {
            const int nr = cur.first + step_r;
            const int nc = cur.second + step_c;
            if (!in_bounds(nr, nc)) {
                break;
            }
            add_hit(nr, nc);
            cur = {nr, nc};
        }
    }

    bool any = false;
    for (const auto& cell : hit_cells) {
        const int hr = cell.first;
        const int hci = cell.second;

        const int ti = robot_index_at(hr, hci);
        if (ti < 0 || ti == shooter_index) {
            continue;
        }

        RobotBase* tgt = robots_[static_cast<size_t>(ti)].robot;
        if (!is_alive(tgt)) {
            continue;
        }

        int lost = 0;
        if (w == railgun) {
            lost = apply_damage(*tgt, kRailDamageMin, kRailDamageMax);
        } else if (w == hammer) {
            lost = apply_damage(*tgt, kHammerDamageMin, kHammerDamageMax);
        } else if (w == grenade) {
            lost = apply_damage(*tgt, kGrenadeDamageMin, kGrenadeDamageMax);
        } else if (w == flamethrower) {
            lost = apply_damage(*tgt, kFlameDamageMin, kFlameDamageMax);
        }

        if (lost > 0) {
            any = true;
            std::cout << "    Hit " << tgt->m_name << " at (" << hr << ',' << hci << ") for " << lost
                      << " damage -> HP " << tgt->get_health();
            if (!is_alive(tgt)) {
                std::cout << " (destroyed)";
            }
            std::cout << '\n';
        }
    }

    return any;
}

void Arena::execute_turn(RobotBase& robot, int robot_index)
{
    if (!is_alive(&robot)) {
        return;
    }

    int r0 = 0;
    int c0 = 0;
    robot.get_current_location(r0, c0);

    std::cout << "\n  " << robot.m_name << ' ' << printable_robot_char(robot.m_character) << " at (" << r0 << ','
              << c0 << ")  HP " << robot.get_health() << "  weapon " << weapon_label(robot.get_weapon()) << '\n';

    int radar_dir = 0;
    robot.get_radar_direction(radar_dir);

    const std::vector<RadarObj> radar = scan_radar(robot, radar_dir);
    std::cout << "    Radar dir " << radar_dir;
    if (radar.empty()) {
        std::cout << " -> no contacts\n";
    } else {
        std::cout << " ->";
        for (const RadarObj& o : radar) {
            std::cout << ' ' << o.m_type << "@(" << o.m_row << ',' << o.m_col << ')';
        }
        std::cout << '\n';
    }

    robot.process_radar_results(radar);

    int shot_r = 0;
    int shot_c = 0;
    const bool wants_shoot = robot.get_shot_location(shot_r, shot_c);

    if (wants_shoot) {
        std::cout << "    Shoot at (" << shot_r << ',' << shot_c << ") with " << weapon_label(robot.get_weapon())
                  << '\n';
        const bool hit = resolve_shoot(robot, robot_index, shot_r, shot_c);
        if (!hit) {
            std::cout << "    (No damage to enemy robots — miss, empty blast, or out of ammo.)\n";
        }
        return;
    }

    int move_dir = 0;
    int move_dist = 0;
    robot.get_move_direction(move_dir, move_dist);
    if (move_dir < 1 || move_dir > 8 || move_dist <= 0 || robot.get_move_speed() <= 0) {
        std::cout << "    Hold position (no move)\n";
        return;
    }

    std::cout << "    Move dir " << move_dir << " distance " << move_dist << " (cap " << robot.get_move_speed()
              << ")\n";
    resolve_move(robot, robot_index, move_dir, move_dist);
    int r1 = 0;
    int c1 = 0;
    robot.get_current_location(r1, c1);
    std::cout << "    Now at (" << r1 << ',' << c1 << ")  HP " << robot.get_health() << '\n';
}

void Arena::print_arena_state()
{
    std::cout << '\n';
    std::cout << "  ";
    for (int c = 0; c < width_; ++c) {
        std::cout << std::setw(3) << c;
    }
    std::cout << '\n';

    for (int r = 0; r < height_; ++r) {
        std::cout << std::setw(3) << r << ' ';
        for (int c = 0; c < width_; ++c) {
            const int idx = robot_index_at(r, c);
            if (idx >= 0) {
                RobotBase* rb = robots_[static_cast<size_t>(idx)].robot;
                const bool alive = is_alive(rb);
                char mark = alive ? 'R' : 'X';
                std::cout << mark << printable_robot_char(rb->m_character) << ' ';
            } else {
                std::cout << obstacle_at(r, c) << "  ";
            }
        }
        std::cout << '\n';
    }

    std::cout << '\n';
    for (size_t i = 0; i < robots_.size(); ++i) {
        RobotBase* rb = robots_[i].robot;
        std::cout << rb->m_name << ' ' << printable_robot_char(rb->m_character) << ' ';
        int rr = 0;
        int cc = 0;
        rb->get_current_location(rr, cc);
        std::cout << '(' << rr << ',' << cc << ") ";
        if (!is_alive(rb)) {
            std::cout << "- dead\n";
        } else {
            std::cout << rb->print_stats() << '\n';
        }
    }
}

int Arena::count_living()
{
    int n = 0;
    for (const auto& lr : robots_) {
        if (is_alive(lr.robot)) {
            ++n;
        }
    }
    return n;
}

int Arena::winner_index()
{
    int idx = -1;
    int n = 0;
    for (size_t i = 0; i < robots_.size(); ++i) {
        if (is_alive(robots_[i].robot)) {
            ++n;
            idx = static_cast<int>(i);
        }
    }
    return n == 1 ? idx : -1;
}

void Arena::run()
{
    place_obstacles_random();
    place_robots_random();

    // RobotWarz_spec.md §30–43: each robot slot prints round + arena, then acts; round increments after each slot.
    int round = 1;
    int turns_taken = 0;

    while (turns_taken < max_rounds_) {
        for (size_t i = 0; i < robots_.size(); ++i) {
            std::cout << "\n         =========== starting round " << round << " ===========\n";
            print_arena_state();

            const int w = winner_index();
            if (w >= 0) {
                std::cout << "\nWinner: " << robots_[static_cast<size_t>(w)].robot->m_name << '\n';
                return;
            }

            if (count_living() == 0) {
                std::cout << "\nNo survivors.\n";
                return;
            }

            RobotBase* rb = robots_[i].robot;
            if (!is_alive(rb)) {
                ++round;
                continue;
            }

            execute_turn(*rb, static_cast<int>(i));
            ++turns_taken;

            const int w2 = winner_index();
            if (w2 >= 0) {
                std::cout << "\nWinner: " << robots_[static_cast<size_t>(w2)].robot->m_name << '\n';
                return;
            }

            maybe_sleep();
            if (turns_taken >= max_rounds_) {
                std::cout << "\nMax rounds reached — no winner declared.\n";
                return;
            }

            ++round;
        }
    }

    if (winner_index() < 0) {
        std::cout << "\nMax rounds reached — no winner declared.\n";
    }
}
