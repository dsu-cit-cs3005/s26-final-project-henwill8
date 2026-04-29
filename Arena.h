#pragma once

#include "RobotBase.h"
#include "RadarObj.h"

#include <string>
#include <vector>
#include <utility>
#include <random>

struct LoadedRobot {
    RobotBase* robot = nullptr;
    void* dl_handle = nullptr;
    std::string source_file;
};

class Arena {
public:
    ~Arena();

    bool load_config(const std::string& config_path);

    bool compile_and_load_robots();

    void run();

private:
    bool parse_config_line(const std::string& line);

    void place_obstacles_random();
    void place_robots_random();

    bool in_bounds(int row, int col) const;

    char obstacle_at(int row, int col) const;

    int robot_index_at(int row, int col);

    bool is_alive(RobotBase* r) const;

    std::vector<RadarObj> scan_radar(RobotBase& observer, int radar_direction);

    void execute_turn(RobotBase& robot, int robot_index);

    bool resolve_shoot(RobotBase& shooter, int shooter_index, int shot_row, int shot_col);

    void resolve_move(RobotBase& robot, int robot_index, int direction, int distance);

    int apply_damage(RobotBase& target, int raw_damage_min, int raw_damage_max);

    void print_arena_state();

    void maybe_sleep() const;

    int count_living();

    int winner_index();

    std::vector<std::string> discover_robot_sources() const;

    int height_ = 0;
    int width_ = 0;
    int max_rounds_ = 10000;
    double sleep_seconds_ = 0.0;
    bool game_live_ = true;
    int flamethrowers_ = 0;
    int pits_ = 0;
    int mounds_ = 0;

    std::vector<std::vector<char>> obstacles_;

    std::vector<LoadedRobot> robots_;

    mutable std::mt19937 rng_{std::random_device{}()};
};
