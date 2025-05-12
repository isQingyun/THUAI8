#include <vector>
#include <thread>
#include <array>
#include <map>
#include <memory>
#include "AI.h"
#include "API.h"
#include "constants.h"
#include <queue>
#include <tuple>
#include <cmath>
#include <chrono>
#include <iostream>
#include <cassert>
#include <algorithm>
#define PI 3.1415926535

// ---------------全局变量声明------------------------------
bool getMapSuccess = false;
std::pair<int, int> myHomeLocation;
std::pair<int, int> enemyHomeLocation;
std::vector<std::pair<int, int>> economyResourceLocations;
std::vector<std::pair<int, int>> additionResourceLocations;
std::vector<std::pair<int, int>> constructionLocations;
std::vector<std::vector<bool>> boolMap;
std::shared_ptr<const THUAI8::Character> selfinfo;
std::vector<std::vector<THUAI8::PlaceType>> mapinfo;

// --------------------------------------------------------

//=================全局函数声明=============================

void GetMap(IAPI& api);
std::vector<std::pair<int, int>> FindPath(const std::vector<std::vector<bool>>& mapData, std::pair<int, int> start, std::pair<int, int> end);
std::pair<int, int> GetEnemyToAttack(ICharacterAPI& api);
void Print_Path(std::vector<std::pair<int32_t, int32_t>> path);
void MoveToCenter(ICharacterAPI& api, const std::pair<double, double>& location);
void MoveFollowPath(ICharacterAPI& api, const std::vector<std::pair<int, int>>& path);
void _execute_straight(ICharacterAPI& api, int dx, int dy, double t1, double t2 = 0.0);
double _get_angle(int dx, int dy);

// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新，大致一帧更新一次
extern const bool asynchronous = true;

// 选手需要依次将player1到player5的角色类型在这里定义
extern const std::array<THUAI8::CharacterType, 6> BuddhistsCharacterTypeDict = {
    THUAI8::CharacterType::TangSeng,
    THUAI8::CharacterType::SunWukong,
    THUAI8::CharacterType::ZhuBajie,
    THUAI8::CharacterType::ShaWujing,
    THUAI8::CharacterType::BaiLongma,
    THUAI8::CharacterType::Monkid,
};

extern const std::array<THUAI8::CharacterType, 6> MonstersCharacterTypeDict = {
    THUAI8::CharacterType::JiuLing,
    THUAI8::CharacterType::HongHaier,
    THUAI8::CharacterType::NiuMowang,
    THUAI8::CharacterType::TieShan,
    THUAI8::CharacterType::ZhiZhujing,
    THUAI8::CharacterType::Pawn,
};

void AI::play(ICharacterAPI& api)
{
    selfinfo = api.GetSelfInfo();
    mapinfo = api.GetFullMap();

    if (!getMapSuccess) {
        GetMap(api);
        getMapSuccess = true;
    }

    if (this->playerID == 1)
    {
        // player1的操作
        // std::this_thread::sleep_for(std::chrono::milliseconds(70));
        auto path = FindPath(boolMap, { selfinfo->x , selfinfo->y }, enemyHomeLocation);
        std::this_thread::sleep_for(std::chrono::milliseconds(70));
        MoveFollowPath(api, path);

    }
    else if (this->playerID == 2)
    {
        // player2的操作
    }
    else if (this->playerID == 3)
    {
        // player3的操作
    }
    else if (this->playerID == 4)
    {
        // player4的操作
    }
    else if (this->playerID == 5)
    {
        // player5的操作
    }
}

void AI::play(ITeamAPI& api)  // 默认team playerID 为0
{
    // player0的操作
}

//获取地图，存储经济资源，加成资源，建筑点，大本营的位置
//存储表示可通行性的bool地图，1表示可通行，0表示不可通行
void GetMap(IAPI& api)
{
    if (!getMapSuccess)
    {
        auto mapData = api.GetFullMap();
        getMapSuccess = true;

        economyResourceLocations.clear();
        additionResourceLocations.clear();
        constructionLocations.clear();

        for (int i = 0; i < mapData.size(); ++i)
        {
            auto& row = mapData[i];
            for (int j = 0; j < row.size(); ++j)
            {
                auto cell = row[j];
                if (cell == THUAI8::PlaceType::EconomyResource)
                {
                    economyResourceLocations.emplace_back(i, j);
                    std::cout << "Economy Resource Location: (" << i << ", " << j << ")" << std::endl;
                }
                else if (cell == THUAI8::PlaceType::AdditionResource)
                {
                    additionResourceLocations.emplace_back(i, j);
                    std::cout << "Addition Resource Location: (" << i << ", " << j << ")" << std::endl;
                }
                else if (cell == THUAI8::PlaceType::Construction)
                {
                    constructionLocations.emplace_back(i, j);
                }
                else if (cell == THUAI8::PlaceType::Home)
                {
                    if (i < 25)
                        myHomeLocation = { i, j };
                    else
                        enemyHomeLocation = { i, j };
                }
            }
        }

        boolMap.clear();
        int mapHeight = mapData.size();
        int mapWidth = mapHeight > 0 ? mapData[0].size() : 0;

        for (int i = 0; i < mapHeight; ++i)
        {
            std::vector<bool> boolRow;
            for (int j = 0; j < mapWidth; ++j)
            {
                auto cell = mapData[i][j];
                boolRow.push_back(cell == THUAI8::PlaceType::Space || cell == THUAI8::PlaceType::Bush);
            }
            boolMap.push_back(boolRow);
        }
    }
    std::cout << "Get Map!" << std::endl;
}

//存储经济资源，加成资源，两方基地的位置
//布尔地图反馈是否可以通行
std::vector<std::pair<int, int>> FindPath(const std::vector<std::vector<bool>>& mapData, std::pair<int, int> start, std::pair<int, int> end)
{
    const std::vector<std::pair<int, int>> directions = { {0, 1}, {0, -1}, {-1, 0}, {1, 0} };

    if (start == end) return {};
    int n = mapData.size();
    if (n == 0) return {};
    int m = mapData[0].size();

    auto start_row = start.first;
    auto start_col = start.second;
    auto end_row = end.first;
    auto end_col = end.second;

    if (start_row < 0 || start_row >= n || start_col < 0 || start_col >= m) return {};
    if (end_row < 0 || end_row >= n || end_col < 0 || end_col >= m) return {};
    if (!mapData[start_row][start_col] || !mapData[end_row][end_col]) return {};

    std::vector<std::vector<bool>> visited(n, std::vector<bool>(m, false));
    std::vector<std::vector<std::pair<int, int>>> parent(n, std::vector<std::pair<int, int>>(m, { -1, -1 }));
    std::queue<std::pair<int, int>> q;
    q.push(start);
    visited[start_row][start_col] = true;

    while (!q.empty())
    {
        auto current_row = q.front().first;
        auto current_col = q.front().second;
        q.pop();

        if (current_row == end_row && current_col == end_col)
        {
            std::vector<std::pair<int, int>> path;
            std::pair<int, int> current = { current_row, current_col };
            while (current != start)
            {
                auto prev_row = parent[current.first][current.second].first;
                auto prev_col = parent[current.first][current.second].second;
                int dx = current.first - prev_row;
                int dy = current.second - prev_col;
                path.emplace_back(dx, dy);
                current = { prev_row, prev_col };
            }
            reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& direction : directions)
        {
            auto dr = direction.first;
            auto dc = direction.second;
            int nr = current_row + dr;
            int nc = current_col + dc;
            if (nr >= 0 && nr < n && nc >= 0 && nc < m)
            {
                if (mapData[nr][nc] && !visited[nr][nc])
                {
                    visited[nr][nc] = true;
                    parent[nr][nc] = { current_row, current_col };
                    q.emplace(nr, nc);
                }
            }
        }
    }
    return {};
}

//打印路径 调试用
void Print_Path(std::vector<std::pair<int32_t, int32_t>> path)
{
    std::cout << "Path: ";
    for (auto itr : path)
    {
        std::cout << "(" << itr.first << "," << itr.second << ")";
    }
    std::cout << "End" << std::endl;
    std::cout << std::endl;
}

//攻击范围内存在敌人时返回{敌人队伍ID，敌人ID} 存在多个敌人时只返回第一个敌人的ID
std::pair<int, int> GetEnemyToAttack(ICharacterAPI& api)
{
    auto self = api.GetSelfInfo();
    bool ableToAttack = (self->characterType != THUAI8::CharacterType::TangSeng &&
        self->characterType != THUAI8::CharacterType::JiuLing);
    if (!ableToAttack) return { -1, -1 };

    auto enemies = api.GetEnemyCharacters();
    for (const auto& enemy : enemies)
    {
        double dx = self->x - enemy->x;
        double dy = self->y - enemy->y;
        double distance = hypot(dx, dy);
        if (distance < self->commonAttackRange)
            return { enemy->teamID, enemy->playerID };
    }
    return { -1, -1 };
}

void MoveToCenter(ICharacterAPI& api, const std::pair<double, double>& location)
{
    auto x = location.first;
    auto y = location.second;
    int cellX = api.GridToCell(x);
    int cellY = api.GridToCell(y);
    double goalX = api.CellToGrid(cellX);
    double goalY = api.CellToGrid(cellY);

    double dx = goalX - x;
    double dy = goalY - y;
    double distance = hypot(dx, dy);
    if (distance < 0.01)
    {
        std::cout << "Already at center (Δ=" << distance << ")" << std::endl;
        return;
    }

    auto currentInfo = api.GetSelfInfo();
    double currentSpeed = currentInfo->speed;
    double buffRemaining = currentInfo->speedBuffTime;
    const double BASE_SPEED = 2.5;

    double theta = atan2(dy, dx);
    double totalTime = distance / currentSpeed;

    bool shouldSplit = (buffRemaining > 0 && buffRemaining < totalTime && currentSpeed > BASE_SPEED);

    if (shouldSplit)
    {
        double buffDistance = currentSpeed * buffRemaining;
        double remainingTime = (distance - buffDistance) / BASE_SPEED;
        api.Move(buffRemaining, theta);
        api.Move(remainingTime, theta);
        std::cout << "Split move: " << buffRemaining << "s@" << currentSpeed
            << " + " << remainingTime << "s@" << BASE_SPEED << std::endl;
    }
    else
    {
        api.Move(totalTime, theta);
        std::cout << "Unified move: " << totalTime << "s@" << currentSpeed << std::endl;
    }

    auto newPosX = api.GetSelfInfo()->x;
    auto newPosY = api.GetSelfInfo()->y;
    double finalError = hypot(goalX - newPosX, goalY - newPosY);
    std::cout << "Centering complete | Final error: " << finalError << std::endl;
}


void MoveFollowPath(ICharacterAPI& api, const std::vector<std::pair<int, int>>& path) {
    const double BASE_SPEED = 2.5;

    if (path.empty()) {
        return;
    }

    int one_step = std::min(10, static_cast<int>(path.size()));

    for (int i = 0; i < one_step; ++i) {
        auto current_info = api.GetSelfInfo();
        double current_speed = current_info->speed;
        double buff_remaining = current_info->speedBuffTime;
        int dx = path[i].first;
        int dy = path[i].second;

        bool is_straight = (dx * dy == 0);
        double distance = is_straight ? 1.0 : sqrt(2.0);
        double total_time = distance / current_speed;

        // 处理buff时间不足的情况
        if (buff_remaining > 0 && buff_remaining < total_time) {
            double buff_distance = current_speed * buff_remaining;
            double remaining_distance = distance - buff_distance;
            double base_time = remaining_distance / BASE_SPEED;

            if (is_straight) {
                _execute_straight(api, dx, dy, buff_remaining, base_time);
            }
            else {
                double angle = _get_angle(dx, dy);
                api.Move(buff_remaining + base_time, angle);
            }
            continue;
        }

        // 正常移动处理
        if (is_straight) {
            _execute_straight(api, dx, dy, total_time);
        }
        else {
            double angle = _get_angle(dx, dy);
            api.Move(total_time, angle);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        // 战斗中断检查
        auto enemyInfo = GetEnemyToAttack(api);
        std::pair<int, int> target = { -1, -1 };
        if (enemyInfo != target) {
            api.Common_Attack(enemyInfo.second);
        }
    }
}

// 辅助函数实现
void _execute_straight(ICharacterAPI& api, int dx, int dy, double t1, double t2) {
    if (dx > 0) {
        api.MoveDown(t1);
        if (t2 > 0) {
            api.MoveDown(t2);
        }
    }
    else if (dx < 0) {
        api.MoveUp(t1);
        if (t2 > 0) {
            api.MoveUp(t2);
        }
    }
    else if (dy > 0) {
        api.MoveRight(t1);
        if (t2 > 0) {
            api.MoveRight(t2);
        }
    }
    else {
        api.MoveLeft(t1);
        if (t2 > 0) {
            api.MoveLeft(t2);
        }
    }
}

double _get_angle(int dx, int dy) {
    if (dx == 1 && dy == 1) {
        return PI / 4;         // 右下
    }
    else if (dx == -1 && dy == -1) {
        return 5 * PI / 4;    // 左上
    }
    else if (dx == 1 && dy == -1) {
        return 7 * PI / 4;    // 左下
    }
    else if (dx == -1 && dy == 1) {
        return 3 * PI / 4;    // 右上
    }
    return 0.0; // 默认情况（理论上不会执行到这里）
}