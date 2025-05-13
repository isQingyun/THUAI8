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
bool getMapSuccess = false; //标志是否以获取布尔地图（一局游戏只获取一次）
std::pair<int, int> myHomeLocation; //己方大本营
std::pair<int, int> enemyHomeLocation; //敌方大本营
std::pair<int, int> test;
std::vector<std::pair<int, int>> economyResourceLocations; //经济资源
std::vector<std::pair<int, int>> additionResourceLocations; //加成资源
std::vector<std::pair<int, int>> constructionLocations; //建筑点
std::deque<std::deque<bool>> boolMap; //布尔地图（寻路等函数所需的实参）
std::shared_ptr<const THUAI8::Character> selfinfo; //角色信息
std::vector<std::vector<THUAI8::PlaceType>> mapinfo; //地图信息
const std::vector<std::pair<int, int>> directions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };  // 右，下，左，上 寻路等函数所用的中间量


//=================全局函数声明=============================

void GetMap(IAPI& api);
std::vector<std::pair<int, int>> FindPath(const std::vector<std::vector<bool>>& mapData, std::pair<int, int> start, std::pair<int, int> end);
std::vector<std::pair<int32_t, int32_t>> Find_Path(std::deque<std::deque<bool>>& map, std::pair<int32_t, int32_t> start, std::pair<int32_t, int32_t> end);
std::pair<int, int> GetEnemyToAttack(ICharacterAPI& api);
void Print_Path(std::vector<std::pair<int32_t, int32_t>> path);
void MoveToCenter(ICharacterAPI& api, const std::pair<double, double>& location);
void MoveFollowPath(ICharacterAPI& api, const std::vector<std::pair<int, int>>& path);
void _execute_straight(ICharacterAPI& api, int dx, int dy, double t1, double t2 = 0.0);
double _get_angle(int dx, int dy);
void Move_Follow_Path(ICharacterAPI& api, std::vector<std::pair<int32_t, int32_t>> path);
std::vector<std::pair<int32_t, int32_t>> SpaceAroundTarget(std::deque<std::deque<bool>>& map, std::pair<int32_t, int32_t> target);


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

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!getMapSuccess) {
        GetMap(api);
        getMapSuccess = true;
    }

    if (this->playerID == 1)
    {
        // player1的操作
        // std::this_thread::sleep_for(std::chrono::milliseconds(70));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto target = SpaceAroundTarget(boolMap, enemyHomeLocation);
        auto path = Find_Path(boolMap, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), target[0]);
        Print_Path(path);
        std::this_thread::sleep_for(std::chrono::milliseconds(70));
        Move_Follow_Path(api, path);

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
//可通行：Space Bush EconomicResource (AdditionRescoure Constraction待测试，目前视为不可通行)
void GetMap(IAPI& api)
{
    std::cout << "GET MAP!" << std::endl;
    if (getMapSuccess == false)
    {
        std::cout << "GET MAP SUCCESS!" << std::endl;
        mapinfo = api.GetFullMap();
        getMapSuccess = true;
        // 将资源位置存入enecomySourceLocation
        for (int i = 0; i < mapinfo.size(); i++)
        {
            for (int j = 0; j < mapinfo[0].size(); j++)
            {
                if (mapinfo[i][j] == THUAI8::PlaceType::EconomyResource)
                {
                    economyResourceLocations.push_back(std::make_pair(i, j));
                    std::cout << "Economy Resource Location: (" << i << "," << j << ")" << std::endl;
                }
                if (mapinfo[i][j] == THUAI8::PlaceType::AdditionResource)
                {
                    additionResourceLocations.push_back(std::make_pair(i, j));
                    std::cout << "Addition Resource Location: (" << i << "," << j << ")" << std::endl;
                }
                if (mapinfo[i][j] == THUAI8::PlaceType::Construction)
                {
                    constructionLocations.push_back(std::make_pair(i, j));
                }
                if (mapinfo[i][j] == THUAI8::PlaceType::Home)
                {
                    if (i < 25)
                    {
                        myHomeLocation = std::make_pair(i, j);
                        std::cout << "My Home Location: (" << i << "," << j << ")" << std::endl;
                    }
                    else
                    {
                        enemyHomeLocation = std::make_pair(i, j);
                        std::cout << "Enemy Home Location: (" << i << "," << j << ")" << std::endl;
                        test = std::make_pair(3, 3);
                        std::cout << "Test: (" << i << "," << j << ")" << std::endl;
                    }
                }
            }
        }
    }
    boolMap.resize(mapinfo.size());
    for (int i = 0; i < mapinfo.size(); i++)
    {
        boolMap[i].resize(mapinfo[0].size());
        for (int j = 0; j < mapinfo[0].size(); j++)
        {
            if (mapinfo[i][j] == THUAI8::PlaceType::Space || mapinfo[i][j] == THUAI8::PlaceType::Bush 
                || mapinfo[i][j] == THUAI8::PlaceType::EconomyResource)
            {
                boolMap[i][j] = true;
            }
            else
            {
                boolMap[i][j] = false;
            }
        }
    }
}

//寻路 注意起点和终点必须是space或bush，否则会返回空
//当目标点是障碍（如资源、大本营）时，先调用下面的SpaceAroundTarget()作为寻路终点
std::vector<std::pair<int32_t, int32_t>> Find_Path(std::deque<std::deque<bool>>& map, std::pair<int32_t, int32_t> start, std::pair<int32_t, int32_t> end)
{
    // 判断，如果start和end在同一位置，直接返回空
    if (start == end)
    {
        return {};
    }
    // 判断，如果start位置处于障碍物上，直接返回空
    if (start.first < 0 || start.first >= map.size() || start.second < 0 || start.second >= map[0].size())
    {
        return {};
    }
    if (end.first < 0 || end.first >= map.size() || end.second < 0 || end.second >= map[0].size())
    {
        return {};
    }
    if (!map[start.first][start.second] || !map[end.first][end.second])
    {
        return {};
    }
    size_t n = map.size();
    size_t m = map[0].size();

    std::vector<std::vector<bool>> visited(n, std::vector<bool>(m, false));
    std::queue<std::pair<int32_t, int32_t>> q;
    std::vector<std::vector<std::pair<int32_t, int32_t>>> parent(n, std::vector<std::pair<int32_t, int32_t>>(m, { -1, -1 }));
    // 初始化起点
    q.push(start);
    visited[start.first][start.second] = true;

    // BFS
    while (!q.empty())
    {
        auto current = q.front();
        q.pop();

        // 如果到达终点
        if (current == end)
        {
            std::vector<std::pair<int32_t, int32_t>> path;
            // 回溯路径
            while (current != start)
            {
                if (current.first < parent.size() &&
                    current.second < parent[0].size() && current.first >= 0 &&
                    current.second >= 0)
                {
                    auto prev = parent[current.first][current.second];
                    path.push_back(
                        { (current.second - prev.second), -(current.first - prev.first) }
                    );
                    current = prev;
                }
                else
                {
                    break;
                }
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // 探索四个方向
        for (const auto& dir : directions)
        {
            int nx = current.first + dir.first;
            int ny = current.second + dir.second;
            if (nx >= 0 && nx < n && ny >= 0 && ny < m)
            {
                if (map[nx][ny] && !visited[nx][ny])
                {
                    visited[nx][ny] = true;
                    parent[nx][ny] = current;
                    q.push({ nx, ny });
                }
            }
        }
    }

    // 如果没有路径，返回空
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

//当目标点是障碍时（即目标点不可通行），不可直接以目标点寻路，先调用该函数找到障碍附近的空地，以该空地为终点调用寻路
//返回值是vector，存储多个可能的空地（应对某个空地被其他角色占用的情况）
std::vector<std::pair<int32_t, int32_t>> SpaceAroundTarget(std::deque<std::deque<bool>>& map, std::pair<int32_t, int32_t> target)
{
    std::vector<std::pair<int32_t, int32_t>> space;
    for (auto itr : directions)
    {
        if (target.first + itr.first < 0 || target.first + itr.first >= map.size() || target.second + itr.second < 0 || target.second + itr.second >= map[0].size())
        {
            continue;
        }
        if (map[target.first + itr.first][target.second + itr.second])
        {
            auto temp = std::make_pair(target.first + itr.first, target.second + itr.second);
            space.push_back(temp);
        }
    }
    return space;
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
    std::pair<int, int> emptyTarget = { -1, -1 };
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

    //    // 战斗中断检查
    //    auto enemyInfo = GetEnemyToAttack(api);
    //    if (enemyInfo != emptyTarget) {
    //        api.Common_Attack(enemyInfo.second);
    //    }
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

void Move_Follow_Path(ICharacterAPI& api, std::vector<std::pair<int32_t, int32_t>> path)
{
    auto time = 10000 / 25;
    double quartePi = 3.1415926 / 4;
    if (path.size() == 0)
    {
        return;
    }
    auto one_step = path.size() >= 10 ? 10 : path.size();
    // 移动单位one_cell_time_civilian，即一个格子的时间，（3,0）向右移动三个格子
    for (int i = 0; i < one_step; i++)
    {
        auto pos = path[i];
        if (pos.first * pos.second == 0)
        {
            if (pos.first >= 1)
                api.MoveRight(time * pos.first);
            if (pos.first <= -1)
                api.MoveLeft(-time * pos.first);
            if (pos.second >= 1)
                api.MoveUp(time * pos.second);
            if (pos.second <= -1)
                api.MoveDown(-time * pos.second);
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
        // MoveToCenter(api, std::make_pair(api.GetSelfInfo()->x, api.GetSelfInfo()->y));
        std::cout << "Modified Location: (" << api.GetSelfInfo()->x << "," << api.GetSelfInfo()->y << ")" << std::endl;
    }
}