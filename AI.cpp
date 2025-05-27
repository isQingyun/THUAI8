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
#include <cstdlib>
#include <ctime>
#define PI 3.1415926535

// ---------------全局变量声明------------------------------
bool getMapSuccess = false;                                                             // 标志是否以获取布尔地图（一局游戏只获取一次）
std::pair<int, int> myHomeLocation;                                                     // 己方大本营
std::pair<int, int> enemyHomeLocation;                                                  // 敌方大本营
std::vector<std::pair<int, int>> economyResourceLocations;                              // 经济资源，
std::vector<std::pair<int, int>> economyResourceLocations_ext;                           // 按照距离己方大本营的顺序排序
int economyResourceIndex_head = 0;                                                      // 经济资源索引 从头到尾
int economyResourceIndex_tail;                                                          // 经济资源索引 从尾到头
int economyResourceIndex_ext;
std::vector<std::pair<int, int>> additionResourceLocations;                             // 加成资源，按照距离己方大本营的顺序排序
int additionResourceIndex_head = 0;                                                     // 加成资源索引 从头到尾
int additionResourceIndex_tail;                                                         // 加成资源索引 从尾到头
std::vector<std::pair<int, int>> constructionLocations;                                 // 建筑点，按照距离己方大本营的顺序排序
int constructionIndex = 0;                                                              // 建筑点索引
std::deque<std::deque<bool>> boolMap;                                                   // 布尔地图（寻路等函数所需的实参）
std::shared_ptr<const THUAI8::Character> selfinfo;                                      // 角色信息
const std::vector<std::pair<int, int>> directions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} }; // 右，下，左，上 寻路等函数所用的中间量
std::vector<std::vector<THUAI8::PlaceType>> mapinfo;                                    // 地图信息
int economyRecourseAmount;                                                              // 经济资源总数
int additionRecourseAmount;                                                             // 加成资源总数
int constractionAmount;                                                                 // 建筑点总数
std::pair<int, int> play1Location;
std::pair<int, int> play2Location;
std::pair<int, int> play3Location;
std::pair<int, int> play4Location;
std::pair<int, int> play5Location;
std::pair<int, int> play6Location;
std::vector<std::pair<int, int>> checkpoints;// { {2,2}, {1,48}, {48,48}, {48,1} };
int curr_checkpoint = 0;
bool flagForPlayer4 = true;
//=================全局函数声明=============================

void GetMap(IAPI& api);
std::vector<std::pair<int32_t, int32_t>> FindPath(std::deque<std::deque<bool>>& map, std::pair<int32_t, int32_t> start, std::pair<int32_t, int32_t> end);
std::pair<int, int> GetEnemyToAttack(ICharacterAPI& api);
void Print_Path(std::vector<std::pair<int32_t, int32_t>> path);
void MoveToCenter(ICharacterAPI& api, const std::pair<double, double>& location);
void MoveFollowPath(ICharacterAPI& api, std::vector<std::pair<int32_t, int32_t>> path, int oneStep = 10);
void MoveFollowPath2(ICharacterAPI& api, std::vector<std::pair<int32_t, int32_t>> path, int distance, int oneStep = 10);
std::vector<std::pair<int32_t, int32_t>> SpaceAroundTarget(std::deque<std::deque<bool>>& map, std::pair<int32_t, int32_t> target);
void SortSource(std::vector<std::pair<int32_t, int32_t>>& sourceLocations);
double Distance(int x1, int y1, int x2, int y2); // 欧式距离
std::deque<std::deque<double>> generate_weights(std::deque<std::deque<bool>>& map, ICharacterAPI& api);
std::vector<std::pair<int32_t, int32_t>> FindWeightedPath(std::deque<std::deque<bool>>& map, std::deque<std::deque<double>>& weights, std::pair<int32_t, int32_t> start, std::pair<int32_t, int32_t> end);
std::pair<int, int> FindClosestPoint(const std::vector<std::pair<int, int>>& points, const std::pair<int, int>& myPosition);
int DistanceSq(const std::pair<int, int>& a, const std::pair<int, int>& b);
double CalculateAngle(double x1, double y1, double x2, double y2);
void SortPointsByProximity(std::vector<std::pair<int, int>>& points, const std::pair<int, int>& myPosition);
std::pair<int32_t, int32_t> GetEnemyLocationFromID(ICharacterAPI& api, int target_playerID);


std::array<THUAI8::CharacterType, 6> CharacterTypeDict;
// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新，大致一帧更新一次
extern const bool asynchronous = true;

// 选手需要依次将player1到player5的角色类型在这里定义
extern const std::array<THUAI8::CharacterType, 6> BuddhistsCharacterTypeDict = {
	THUAI8::CharacterType::TangSeng,
	THUAI8::CharacterType::ZhuBajie,
	THUAI8::CharacterType::Monkid,
	THUAI8::CharacterType::Monkid,
	THUAI8::CharacterType::ZhuBajie,
	THUAI8::CharacterType::ShaWujing,
};

extern const std::array<THUAI8::CharacterType, 6> MonstersCharacterTypeDict = {
	THUAI8::CharacterType::JiuLing,
	THUAI8::CharacterType::NiuMowang,
	THUAI8::CharacterType::Pawn,
	THUAI8::CharacterType::Pawn,
	THUAI8::CharacterType::NiuMowang,
	THUAI8::CharacterType::TieShan,
};

// extern const std::array<THUAI8::CharacterType, 6> BuddhistsCharacterTypeDict = {
//     THUAI8::CharacterType::TangSeng,
//     THUAI8::CharacterType::SunWukong,
//     THUAI8::CharacterType::ZhuBajie,
//     THUAI8::CharacterType::ShaWujing,
//     THUAI8::CharacterType::BaiLongma,
//     THUAI8::CharacterType::Monkid,
// };
//
// extern const std::array<THUAI8::CharacterType, 6> MonstersCharacterTypeDict = {
//     THUAI8::CharacterType::JiuLing,
//     THUAI8::CharacterType::HongHaier,
//     THUAI8::CharacterType::NiuMowang,
//     THUAI8::CharacterType::TieShan,
//     THUAI8::CharacterType::ZhiZhujing,
//     THUAI8::CharacterType::Pawn,
// };

void AI::play(ICharacterAPI& api)
{
	selfinfo = api.GetSelfInfo();
	mapinfo = api.GetFullMap();

	// std::this_thread::sleep_for(std::chrono::milliseconds(100));
	if (!getMapSuccess)
	{
		GetMap(api);
		getMapSuccess = true;
		economyRecourseAmount = economyResourceLocations.size();
		additionRecourseAmount = additionResourceLocations.size();
		constractionAmount = constructionLocations.size();
		economyResourceIndex_tail = economyRecourseAmount - 1;
		economyResourceIndex_ext = economyRecourseAmount; //此处含大本营
		additionResourceIndex_tail = additionRecourseAmount - 1;
	}

	if (this->playerID == 1)
	{
		// player1的操作
		if (selfinfo->hp <= 400)
		{
			auto weight = generate_weights(boolMap, api);
			auto target = SpaceAroundTarget(boolMap, myHomeLocation);
			auto path = FindWeightedPath(boolMap, weight, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), target[0]);
			MoveFollowPath2(api, path, 1500);
		}
		else
		{
			if (!economyResourceLocations.empty())
			{
				auto target = economyResourceLocations[economyResourceIndex_tail]; // 唐僧从远往近采集
				// api.Print("Economy Resource Index: " + std::to_string(economyResourceIndex));
				// api.Print("Economy Resource Location: (" + std::to_string(target.first) + "," + std::to_string(target.second) + ")");
				auto targetState = api.GetEconomyResourceState(target.first, target.second);
				// api.Print("MAPType:" + std::to_string(static_cast<int>(mapinfo[target.first][target.second])));
				while (!targetState.has_value() || targetState.value().process) // Harvestable = 1,BeingHarvested = 2,Harvested = 3,
				{
					economyResourceIndex_tail--;
					// 完尽情况暂不考虑
					if (economyResourceIndex_tail < 0)
					{
						economyResourceIndex_tail = economyRecourseAmount - 1;
					}
					target = economyResourceLocations[economyResourceIndex_tail];
					targetState = api.GetEconomyResourceState(target.first, target.second);
				}
				api.Print("Process:" + std::to_string(targetState.value().process));
				if (Distance(selfinfo->x, selfinfo->y, target.first * 1000 + 500, target.second * 1000 + 500) < 1500)
				{
					// 开采，距离够
					api.Produce();
				}
				else
				{
					// 距离不够寻路移动
					auto locationTarget = SpaceAroundTarget(boolMap, target);

					auto weight = generate_weights(boolMap, api);
					auto path = FindWeightedPath(boolMap, weight, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), locationTarget[0]);
					Print_Path(path);
					std::this_thread::sleep_for(std::chrono::milliseconds(70));
					MoveFollowPath2(api, path, 1500);
				}
			}
			else
			{
				std::cout << "No economy resource available!" << std::endl;
			}
		}
	}
	//else if (this->playerID == 2)
	//{
	//    // player2的操作
	//    if (!economyResourceLocations.empty())
	//    {
	//        auto target = economyResourceLocations[economyResourceIndex_head];
	//        // api.Print("Economy Resource Index: " + std::to_string(economyResourceIndex));
	//        // api.Print("Economy Resource Location: (" + std::to_string(target.first) + "," + std::to_string(target.second) + ")");
	//        auto targetState = api.GetEconomyResourceState(target.first, target.second);
	//        // api.Print("MAPType:" + std::to_string(static_cast<int>(mapinfo[target.first][target.second])));
	//        while (!targetState.has_value() || targetState.value().process) // Harvestable = 1,BeingHarvested = 2,Harvested = 3,
	//        {
	//            /*if (!targetState.has_value())
	//            {
	//                api.Print("TargetState is null!");
	//            }
	//            else
	//            {
	//                api.Print("process");
	//            }*/
	//            economyResourceIndex_head++;
	//            // 完尽情况暂不考虑
	//            if (economyResourceIndex_head == economyRecourseAmount)
	//            {
	//                economyResourceIndex_head = 0;
	//            }
	//            // api.Print("MAPType:" + std::to_string(static_cast<int>(mapinfo[target.first][target.second])));
	//            target = economyResourceLocations[economyResourceIndex_head];
	//            targetState = api.GetEconomyResourceState(target.first, target.second);
	//        }
	//        api.Print("Process:" + std::to_string(targetState.value().process));
	//        if (Distance(selfinfo->x, selfinfo->y, target.first * 1000 + 500, target.second * 1000 + 500) < 1500)
	//        {
	//            // 开采，距离够
	//            api.Produce();
	//        }
	//        else
	//        {
	//            // 距离不够寻路移动
	//            auto locationTarget = SpaceAroundTarget(boolMap, target);
	//            auto path = FindPath(boolMap, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), locationTarget[0]);
	//            Print_Path(path);
	//            std::this_thread::sleep_for(std::chrono::milliseconds(70));
	//            MoveFollowPath2(api, path, 1500);
	//        }
	//    }
	//    else
	//    {
	//        std::cout << "No economy resource available!" << std::endl;
	//    }
	//}
	else if (this->playerID == 2)
	{
		// player5的操作
		// zhubajie
		if (flagForPlayer4) {
			if (!economyResourceLocations.empty())
			{
				auto target = economyResourceLocations[economyResourceIndex_head];
				// api.Print("Economy Resource Index: " + std::to_string(economyResourceIndex));
				// api.Print("Economy Resource Location: (" + std::to_string(target.first) + "," + std::to_string(target.second) + ")");
				auto targetState = api.GetEconomyResourceState(target.first, target.second);
				// api.Print("MAPType:" + std::to_string(static_cast<int>(mapinfo[target.first][target.second])));
				while (!targetState.has_value() || targetState.value().process) // Harvestable = 1,BeingHarvested = 2,Harvested = 3,
				{
					/*if (!targetState.has_value())
					{
						api.Print("TargetState is null!");
					}
					else
					{
						api.Print("process");
					}*/
					economyResourceIndex_head++;
					// 完尽情况暂不考虑
					if (economyResourceIndex_head >= economyRecourseAmount) //TEST
					{
						economyResourceIndex_head = 0;
						flagForPlayer4 = false;
						break;
					}
					// api.Print("MAPType:" + std::to_string(static_cast<int>(mapinfo[target.first][target.second])));
					target = economyResourceLocations[economyResourceIndex_head];
					targetState = api.GetEconomyResourceState(target.first, target.second);
				}
				api.Print("Process:" + std::to_string(targetState.value().process));
				if (Distance(selfinfo->x, selfinfo->y, target.first * 1000 + 500, target.second * 1000 + 500) < 1100)
				{
					// 开采，距离够
					api.Produce();
				}
				else
				{
					// 距离不够寻路移动
					auto locationTarget = SpaceAroundTarget(boolMap, target);
					auto path = FindPath(boolMap, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), locationTarget[locationTarget.size() - 1]);
					Print_Path(path);
					std::this_thread::sleep_for(std::chrono::milliseconds(70));
					MoveFollowPath2(api, path, 1100);
				}
			}
			else
			{
				std::cout << "No economy resource available!" << std::endl;
			}
		}
		else {
			auto commonAttackRange = selfinfo->commonAttackRange;
			auto enemy = GetEnemyToAttack(api);
			int mode = 0;  // 0 代表巡逻模式，1代表攻击模式
			if (enemy.first != -1) mode = 1;  // 如果找到了可以攻击的敌人，进入攻击模式
			if (mode == 0) {
				// TODO
				std::vector<std::pair<int, int>> checkpoints = { {2,2}, {25, 25}, { 2,48 }, {48,48}, {25, 25}, { 48,2 } };
				// 全局变量，该角色会依次遍历checkpoints
				// std::vector<std::pair<int, int>> checkpoints = { {2,2} };// { {2,2}, {1,48}, {48,48}, {48,1} };
				// int curr_checkpoint = 0;
				// std::pair<int, int> target = { 10, 10 };
				auto target = checkpoints[curr_checkpoint];
				auto locationTarget = SpaceAroundTarget(boolMap, target);
				if (locationTarget.size() >= 1) {
					auto path = FindPath(boolMap, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), locationTarget[0]
					);
					Print_Path(path);
					std::this_thread::sleep_for(std::chrono::milliseconds(70));
					MoveFollowPath2(api, path, commonAttackRange);
					if (Distance(selfinfo->x, selfinfo->y, target.first * 1000, target.second * 1000) < 2000)  // 如果到了目的地那么前往下一个目的地
						curr_checkpoint = (curr_checkpoint + 1) % checkpoints.size();
				}

			}
			else if (mode == 1) {
				// 攻击模式
				api.Common_Attack(enemy.second);
				std::pair<int32_t, int32_t> target = GetEnemyLocationFromID(api, enemy.second);
				api.Print("Selected Enemy ID: " + std::to_string(enemy.second) + " Location: (" + std::to_string(target.first) + "," + std::to_string(target.second) + ")");
				if (target.first != -1) {
					if (Distance(selfinfo->x, selfinfo->y, target.first * 1000, target.second * 1000) < commonAttackRange) {
						// 距离够直接攻击
						api.Common_Attack(enemy.second);
						api.Print("Attack is successful!");
					}
					else
					{
						// 距离不够寻路移动
						auto locationTarget = SpaceAroundTarget(boolMap, target);
						auto path = FindPath(boolMap, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), locationTarget[0]);
						Print_Path(path);
						std::this_thread::sleep_for(std::chrono::milliseconds(70));
						MoveFollowPath2(api, path, commonAttackRange);
					}
				}
			}
		}
	}
	else if (this->playerID == 3)
	{
		if (!economyResourceLocations.empty())
		{
			auto target = economyResourceLocations_ext[economyResourceIndex_ext];
			// api.Print("Economy Resource Index: " + std::to_string(economyResourceIndex));
			// api.Print("Economy Resource Location: (" + std::to_string(target.first) + "," + std::to_string(target.second) + ")");
			auto targetState = api.GetEconomyResourceState(target.first, target.second);
			// api.Print("MAPType:" + std::to_string(static_cast<int>(mapinfo[target.first][target.second])));
			while (!targetState.has_value() || targetState.value().process) // Harvestable = 1,BeingHarvested = 2,Harvested = 3,
			{
				/*if (!targetState.has_value())
				{
					api.Print("TargetState is null!");
				}
				else
				{
					api.Print("process");
				}*/
				economyResourceIndex_ext--;
				// 完尽情况暂不考虑
				if (economyResourceIndex_ext < 0)
				{
					economyResourceIndex_ext = 0;
					flagForPlayer4 = false;
				}
				// api.Print("MAPType:" + std::to_string(static_cast<int>(mapinfo[target.first][target.second])));
				target = economyResourceLocations_ext[economyResourceIndex_ext];
				targetState = api.GetEconomyResourceState(target.first, target.second);
			}
			api.Print("Process:" + std::to_string(targetState.value().process));
			if (Distance(selfinfo->x, selfinfo->y, target.first * 1000 + 500, target.second * 1000 + 500) < 1500)
			{
				// 开采，距离够
				// 建造伤害陷阱
				api.ConstructTrap(THUAI8::TrapType::Hole);
				economyResourceIndex_ext--;
				if (economyResourceIndex_ext < 0)
				{
					economyResourceIndex_ext = 0;
					flagForPlayer4 = false;
				}
			}
			else
			{
				// 距离不够寻路移动
				auto locationTarget = SpaceAroundTarget(boolMap, target);
				auto path = FindPath(boolMap, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), locationTarget[locationTarget.size() - 1]);
				Print_Path(path);
				std::this_thread::sleep_for(std::chrono::milliseconds(70));
				MoveFollowPath2(api, path, 1100);
			}
		}
		else
		{
			std::cout << "No economy resource available!" << std::endl;
		}
	}
	else if (this->playerID == 4)
	{
		// player3的操作
		auto target = constructionLocations[constructionIndex];
		auto targetState = api.GetConstructionState(target.first, target.second);
		if (Distance(selfinfo->x, selfinfo->y, target.first * 1000 + 500, target.second * 1000 + 500) < 1250)
		{
			auto constructionType = targetState.value().constructionType;
			if (constructionType == THUAI8::ConstructionType::NullConstructionType || (constructionType == THUAI8::ConstructionType::Farm && targetState.value().team_id == selfinfo->teamID))
			{
				// 没有建筑则修建农场

				if (targetState.value().hp >= 400)
				{
					constructionIndex++;
					if (constructionIndex == constractionAmount)
					{
						constructionIndex = 0;
					}
				}
				else
				{
					api.Construct(THUAI8::ConstructionType::Farm);
				}
			}
			else
			{
				api.Print("UNUNUNUNUN!!");
				if (targetState.value().team_id != selfinfo->teamID)
				{
					// 有敌方建筑则攻击
					api.AttackConstruction();
				}
				else
				{
					constructionIndex++;
					std::this_thread::sleep_for(std::chrono::milliseconds(70));
				}
			}
		}
		else
		{
			// 距离不够寻路移动
			auto locationTarget = SpaceAroundTarget(boolMap, target);
			auto path = FindPath(boolMap, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), locationTarget[0]);
			//auto path = FindPath(boolMap, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), target);
			Print_Path(path);
			std::this_thread::sleep_for(std::chrono::milliseconds(70));
			MoveFollowPath2(api, path, 1250);
		}
		}
	else if (this->playerID == 5)
	{
		// player4的操作
		// zhubajie
		auto commonAttackRange = selfinfo->commonAttackRange;
		if (!additionResourceLocations.empty())
		{
			auto target = additionResourceLocations[additionResourceIndex_head];
			auto targetState = api.GetAdditionResourceState(target.first, target.second);
			while (!targetState.has_value() || targetState.value().hp == 0) // Harvestable = 1,BeingHarvested = 2,Harvested = 3,
			{
				additionResourceIndex_head++;
				// 完尽情况暂不考虑
				if (additionResourceIndex_head > additionRecourseAmount)
				{
					additionResourceIndex_head = 0;
				}
				target = additionResourceLocations[additionResourceIndex_head];
				targetState = api.GetAdditionResourceState(target.first, target.second);
			}
			api.Print("HP:" + std::to_string(targetState.value().hp));
			api.Print("Type:" + std::to_string(static_cast<int>(targetState.value().additionResourceType)));
			if (Distance(selfinfo->x, selfinfo->y, target.first * 1000, target.second * 1000) < commonAttackRange)
			{
				// 开采，距离够
				api.AttackAdditionResource();
				std::this_thread::sleep_for(std::chrono::milliseconds(70));
			}
			else
			{
				// 距离不够寻路移动
				auto locationTarget = SpaceAroundTarget(boolMap, target);
				auto path = FindPath(boolMap, std::make_pair(selfinfo->x / 1000, selfinfo->y / 1000), locationTarget[1]);
				Print_Path(path);
				std::this_thread::sleep_for(std::chrono::milliseconds(70));
				MoveFollowPath2(api, path, commonAttackRange);
			}
		}
		else
		{
			std::cout << "No Additional resource available!" << std::endl;
		}
	}

}

void AI::play(ITeamAPI& api) // 默认team playerID 为0
{
	// player0的操作
	auto teamInfo = api.GetSelfInfo();
	auto gameinfo = api.GetGameInfo();
	int teamID = teamInfo->teamID;
	if (static_cast<int>(CharacterTypeDict[0]) == 0)
	{
		CharacterTypeDict = (teamID == 0) ? BuddhistsCharacterTypeDict : MonstersCharacterTypeDict;
	}
	auto HomeEconomy = std::vector{ gameinfo->buddhistsTeamEconomy, gameinfo->monstersTeamEconomy };
	auto myHomeEconomy = HomeEconomy[teamID];
	auto players = api.GetCharacters();
	auto energy = api.GetEnergy();
	api.Print("My Home Energy: " + std::to_string(energy));
	api.Print("My Home Eco " + std::to_string(myHomeEconomy));
	auto Characters = api.GetCharacters();
	int CharactersIndex = Characters.size() / 2;

	api.Print("CharactersIndex: " + std::to_string(CharactersIndex));
	for (auto itr : Characters)
	{
		api.Print("Characters: " + std::to_string(itr->playerID));
		api.Print("Characters: " + std::to_string(static_cast<int>(itr->characterType)));
	}
	auto EnemyCharacters = api.GetEnemyCharacters();
	if (myHomeEconomy > 1000 && CharactersIndex < 3)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(70));
		// api.BuildCharacter(static_cast<THUAI8::CharacterType>(6 + 6 * teamID), 0);
		api.BuildCharacter(CharacterTypeDict[CharactersIndex], 0);
		/*if (api.BuildCharacter(CharacterTypeDict[CharactersIndex], 0).get())
		{
			api.Print("Build " + std::to_string(CharactersIndex) );
		}
		else
		{
			api.Print("Build " + std::to_string(CharactersIndex)+ " failed");
		}*/
	}
	else if (myHomeEconomy >= 4000 && CharactersIndex < 4)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(70));
		// api.BuildCharacter(static_cast<THUAI8::CharacterType>(6 + 6 * teamID), 0);
		api.BuildCharacter(CharacterTypeDict[CharactersIndex], 0);
	}
	// 调试用，上传前得修改！
	else if (myHomeEconomy >= 4000 && CharactersIndex < 5)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(70));
		// api.BuildCharacter(static_cast<THUAI8::CharacterType>(6 + 6 * teamID), 0);
		api.BuildCharacter(CharacterTypeDict[CharactersIndex], 0);
	}
	//else if (myHomeEconomy >= 10000 && CharactersIndex < 5)
	//{
	//    std::this_thread::sleep_for(std::chrono::milliseconds(70));
	//    // api.BuildCharacter(static_cast<THUAI8::CharacterType>(6 + 6 * teamID), 0);
	//    api.BuildCharacter(CharacterTypeDict[CharactersIndex], 0);
	//}
}

// 获取地图，存储经济资源，加成资源，建筑点，大本营的位置
// 存储表示可通行性的bool地图，1表示可通行，0表示不可通行
// 可通行：Space  Bush  EconomicResource  AdditionRescoure（其他均不可通行）
void GetMap(IAPI& api)
{
	std::cout << "GET MAP!" << std::endl;
	if (getMapSuccess == false)
	{
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
					if (selfinfo->teamID == 0)
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
						}
					}
					else
					{
						if (i > 25)
						{
							myHomeLocation = std::make_pair(i, j);
							std::cout << "My Home Location: (" << i << "," << j << ")" << std::endl;
						}
						else
						{
							enemyHomeLocation = std::make_pair(i, j);
							std::cout << "Enemy Home Location: (" << i << "," << j << ")" << std::endl;
						}
					}
				}
			}
		}
		// 按照距离当前位置最近的原则排序
		SortPointsByProximity(economyResourceLocations, myHomeLocation);
		SortPointsByProximity(additionResourceLocations, myHomeLocation);
		SortPointsByProximity(constructionLocations, myHomeLocation);
		economyResourceLocations_ext = economyResourceLocations;

		// 直接按照距离大本营的欧式距离排序
		SortSource(economyResourceLocations_ext);
		economyResourceLocations_ext.insert(economyResourceLocations_ext.end(), enemyHomeLocation);
		// SortSource(additionResourceLocations);
		// SortSource(constructionLocations);
	}
	boolMap.resize(mapinfo.size());
	for (int i = 0; i < mapinfo.size(); i++)
	{
		boolMap[i].resize(mapinfo[0].size());
		for (int j = 0; j < mapinfo[0].size(); j++)
		{
			if (mapinfo[i][j] == THUAI8::PlaceType::Space || mapinfo[i][j] == THUAI8::PlaceType::Bush)
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

// 寻路 注意起点和终点必须是space或bush，否则会返回空
// 当目标点是障碍（如资源、大本营）时，先调用下面的SpaceAroundTarget()作为寻路终点
std::vector<std::pair<int32_t, int32_t>> FindPath(std::deque<std::deque<bool>>& map, std::pair<int32_t, int32_t> start, std::pair<int32_t, int32_t> end)
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
				if (current.first >= 0 &&
					current.second >= 0)
				{
					path.push_back({ current.first , current.second });
					current = parent[current.first][current.second];
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

// 打印路径 调试用
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

// 当目标点是障碍时（即目标点不可通行），不可直接以目标点寻路，先调用该函数找到障碍附近的空地，以该空地为终点调用寻路
// 返回值是vector，存储多个可能的空地（应对某个空地被其他角色占用的情况）
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

// 攻击范围内存在敌人时返回{敌人队伍ID，敌人ID} 存在多个敌人时只返回第一个敌人的ID
std::pair<int, int> GetEnemyToAttack(ICharacterAPI& api)
{
	bool ableToAttack = (selfinfo->characterType != THUAI8::CharacterType::TangSeng &&
		selfinfo->characterType != THUAI8::CharacterType::JiuLing);
	if (!ableToAttack)
		return { -1, -1 };

	auto enemies = api.GetEnemyCharacters();
	for (const auto& enemy : enemies)
	{
		double dx = selfinfo->x - enemy->x;
		double dy = selfinfo->y - enemy->y;
		double distance = hypot(dx, dy);
		if (distance < selfinfo->commonAttackRange * 1000)
			return { enemy->teamID, enemy->playerID };
	}
	return { -1, -1 };
}

// 通过敌方ID获得敌方坐标，配合GetEnemyToAttack使用
std::pair<int32_t, int32_t> GetEnemyLocationFromID(ICharacterAPI& api, int target_playerID)
{
	auto enemies = api.GetEnemyCharacters();
	for (const auto& enemy : enemies)
	{
		if (enemy->playerID == target_playerID) {
			return { enemy->x, enemy->y };
		}
	}
	return { -1, -1 };
}

// 把角色移动至其所在格子的中央 消除移动途中的误差
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
	double currentSpeed = currentInfo->speed;                 // 格/s
	double buffRemaining = currentInfo->speedBuffTime * 1000; // ms

	double theta = atan2(dy, dx);
	double totalTime = (distance / currentSpeed) * 1000; // ms

	if (buffRemaining > 0 && buffRemaining < totalTime)
	{
		double buffDistance = currentSpeed * buffRemaining;
		double remainingTime = (distance - buffDistance) / 2.5;
		api.Move(buffRemaining, theta);
		api.Move(remainingTime, theta);
		std::cout << "Split move: " << buffRemaining << "s@" << currentSpeed
			<< " + " << remainingTime << "s@" << 2500 << std::endl;
	}
	else
	{
		api.Move(totalTime, theta);
		std::cout << "Unified move: " << totalTime << "s@" << currentSpeed << std::endl;
	}
}

// 沿着FindPath()给出的路线运动 考虑了移速buff 包含误差修正
void MoveFollowPath(ICharacterAPI& api, std::vector<std::pair<int32_t, int32_t>> path, int oneStep)
{
	if (path.size() == 0)
	{
		return;
	}
	std::pair<int, int> playerLocation = std::make_pair(selfinfo->x, selfinfo->y);
	auto one_step = path.size() >= oneStep ? oneStep : path.size(); // 最多移动oneStep步 防止积累过多误差或长时间占用线程
	for (int i = 0; i < one_step; i++)
	{
		double current_speed = selfinfo->speed;                 // 格/s
		double buff_remaining = selfinfo->speedBuffTime * 1000; // ms
		double total_time = (1000 / current_speed) * 1000;      // ms
		std::pair<int, int> pos = path[i];

		// 处理buff时间不足的情况
		if (buff_remaining > 0 && buff_remaining < total_time)
		{
			double buff_distance = current_speed * buff_remaining;
			double remaining_distance = 1000 - buff_distance;
			total_time += remaining_distance / 2.5;
		}

		if (pos.first >= 1)
			api.MoveRight(total_time * pos.first);
		if (pos.first <= -1)
			api.MoveLeft(-total_time * pos.first);
		if (pos.second >= 1)
			api.MoveUp(total_time * pos.second);
		if (pos.second <= -1)
			api.MoveDown(-total_time * pos.second);

		std::this_thread::sleep_for(std::chrono::milliseconds(400));
		auto enemy = GetEnemyToAttack(api);
		if (enemy.first != -1)
			api.Common_Attack(enemy.second);
	}
	MoveToCenter(api, std::make_pair(api.GetSelfInfo()->x, api.GetSelfInfo()->y));
	std::cout << "Modified Location: (" << api.GetSelfInfo()->x << "," << api.GetSelfInfo()->y << ")" << std::endl;
}
void MoveFollowPath2(ICharacterAPI& api, std::vector<std::pair<int32_t, int32_t>> path, int distance, int oneStep)
{
	auto selfinfo = api.GetSelfInfo();
	double moveAngle = 0.0;
	double moveTime = 0.0;
	double speed = (double)selfinfo->speed;
	auto one_step = path.size() >= oneStep ? oneStep : path.size();
	/*api.Print("Path: selfxxxxx(" + std::to_string(selfinfo->x) + ",yyyyyyyy" + std::to_string(selfinfo->y) + ")");
	for(auto i:path)
	{
		api.Print("Path: (" + std::to_string(i.first) + "," + std::to_string(i.second) + ")");
	}*/
	if (one_step == 0)
	{
		return;
	}
	auto end = path.back();
	moveAngle = CalculateAngle(selfinfo->x, selfinfo->y, (selfinfo->x / 1000) * 1000 + 500, (selfinfo->y / 1000) * 1000 + 500);
	moveTime = Distance(selfinfo->x, selfinfo->y, (selfinfo->x / 1000) * 1000 + 500, (selfinfo->y / 1000) * 1000 + 500) * 1000 / speed;
	api.Move((int)moveTime, moveAngle);
	api.Print("moveTime:" + std::to_string(moveTime));
	std::this_thread::sleep_for(std::chrono::milliseconds((int)(moveTime))); // 370
	std::pair<int, int> current = path[0];
	std::pair<int, int> future = path[0];
	//selfinfo = api.GetSelfInfo();
	moveAngle = CalculateAngle((selfinfo->x / 1000) * 1000 + 500, (selfinfo->y / 1000) * 1000 + 500, current.first * 1000 + 500, current.second * 1000 + 500);
	moveTime = Distance((selfinfo->x / 1000) * 1000 + 500, (selfinfo->y / 1000) * 1000 + 500, current.first * 1000 + 500, current.second * 1000 + 500) * 1000 / speed;
	api.Move((int)(moveTime), moveAngle);
	api.Print("moveTime:" + std::to_string(moveTime));
	std::this_thread::sleep_for(std::chrono::milliseconds((int)(moveTime))); // 370
	for (int i = 0; i < one_step; i++)
	{
		// TODO - ATTACK ENEMY 
		auto commonAttackRange = selfinfo->commonAttackRange;
		auto enemy = GetEnemyToAttack(api);
		if (enemy.first != -1) {
			api.Common_Attack(enemy.second);
			std::pair<int32_t, int32_t> target = GetEnemyLocationFromID(api, enemy.second);
			if (Distance(selfinfo->x, selfinfo->y, target.first * 1000, target.second * 1000) < commonAttackRange) {
				// 距离够直接攻击
				api.Common_Attack(enemy.second);
				api.Print("Attack is successful!");
			}
		}
		current = path[i];
		if (Distance(current.first * 1000 + 500, current.second * 1000 + 500, end.first * 1000 + 500, end.second * 1000 + 500) < distance)
		{
			selfinfo = api.GetSelfInfo();
			if (Distance(selfinfo->x, selfinfo->y, end.first * 1000 + 500, end.second * 1000 + 500) < distance)
			{
				break;
			}

		}
		if (i + 1 == path.size())
		{
			selfinfo = api.GetSelfInfo();
			moveAngle = CalculateAngle(selfinfo->x, selfinfo->y, (selfinfo->x / 1000) * 1000 + 500, (selfinfo->y / 1000) * 1000 + 500);
			moveTime = Distance(selfinfo->x, selfinfo->y, (selfinfo->x / 1000) * 1000 + 500, (selfinfo->y / 1000) * 1000 + 500) * 1000 / speed;
			api.Move((int)moveTime, moveAngle);
			api.Print("moveTime:" + std::to_string(moveTime));
			std::this_thread::sleep_for(std::chrono::milliseconds((int)(moveTime)+50)); // 370
			break;
		}
		future = path[i + 1];
		api.Print("currrent:" + std::to_string(current.first) + "," + std::to_string(current.second) + ")");
		api.Print("future:" + std::to_string(future.first) + "," + std::to_string(future.second) + ")");
		moveAngle = CalculateAngle(current.first * 1000 + 500, current.second * 1000 + 500, future.first * 1000 + 500, future.second * 1000 + 500);
		moveTime = Distance(current.first * 1000 + 500, current.second * 1000 + 500, future.first * 1000 + 500, future.second * 1000 + 500) * 1000 / speed;
		api.Move((int)(moveTime), moveAngle);
		//std::cout << "moveTime:" << moveTime * 1000 + 1 << std::endl;
		api.Print("moveTime:" + std::to_string(moveTime));
		std::this_thread::sleep_for(std::chrono::milliseconds((int)(moveTime)));

	}
}
void SortSource(std::vector<std::pair<int32_t, int32_t>>& sourceLocations)
{
	// 将资源（建筑）按照离自己大本营的欧氏距离排序
	std::sort(sourceLocations.begin(), sourceLocations.end(), [](std::pair<int32_t, int32_t> a, std::pair<int32_t, int32_t> b)
		{ return (a.first - myHomeLocation.first) * (a.first - myHomeLocation.first) + (a.second - myHomeLocation.second) * (a.second - myHomeLocation.second) < (b.first - myHomeLocation.first) * (b.first - myHomeLocation.first) + (b.second - myHomeLocation.second) * (b.second - myHomeLocation.second); });
}

double Distance(int x1, int y1, int x2, int y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
double CalculateAngle(double x1, double y1, double x2, double y2)
{
	// 计算方向角度
	double angle = std::atan2(y2 - y1, x2 - x1);
	// 于坐标而言，地图的左上角为 (0, 0)，左下角为(50000, 0)(50000, 0)，右上角为(0, 50000)(0, 50000)，右下角为(50000, 50000)(50000, 50000)
	// 对于方向而言，竖直向下为 0 ，水平向右为π/ 2，竖直向上为 π，水平向左为 3π / 2。
	//  确保角度在0到2PI之间
	if (angle < 0)
	{
		angle += 2 * PI;
	}
	std::cout << "Angle:" << angle << std::endl;
	std::cout << "x1：" << x1 << "y1：" << y1 << "x2：" << x2 << "y2：" << y2 << std::endl;
	return angle;
}
std::deque<std::deque<double>> generate_weights(std::deque<std::deque<bool>>& map, ICharacterAPI& api)
{
	int weights_n = map.size();
	int weights_m = map[0].size();
	std::deque<std::deque<double>> weights(
		weights_n,
		std::deque<double>(weights_m, 1.0));
	auto enemies = api.GetEnemyCharacters();
	for (const auto& enemy : enemies)
	{
		int enemy_x = enemy->x;
		int enemy_y = enemy->y;
		int enemy_attack_range = enemy->commonAttackRange;
		for (int i = -enemy_attack_range; i <= enemy_attack_range; i++)
		{
			for (int j = -enemy_attack_range; j <= enemy_attack_range; j++)
			{
				if (i * i + j * j < enemy_attack_range * enemy_attack_range &&
					enemy_x + i >= 0 && enemy_x + i < weights_n &&
					enemy_y + j >= 0 && enemy_y + j < weights_m)
				{
					weights[enemy_x + i][enemy_y + j] += 5 * exp(-(i * i + j * j) / (enemy_attack_range * enemy_attack_range) / 2);
				}
			}
		}
	}
	return weights;
}

std::vector<std::pair<int32_t, int32_t>> FindWeightedPath(std::deque<std::deque<bool>>& map, std::deque<std::deque<double>>& weights, std::pair<int32_t, int32_t> start, std::pair<int32_t, int32_t> end)
{
	// 功能：根据权值地图利用Dijkstra算法范围总权值和最短的路径
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
	// 判断，如果start位置位于地图外，直接返回空
	if (!map[start.first][start.second] || !map[end.first][end.second])
	{
		return {};
	}
	size_t n = map.size();
	size_t m = map[0].size();

	// weights是全地图上的权值矩阵
	size_t weights_n = weights.size();
	size_t weights_m = weights[0].size();

	// ---------------------------------------Main Logic: 使用Dijkstra算法计算出在加权情况下的路径----------------------------------
	// 初始化距离数组，dist[r][c] 表示从起点到 (r,c) 的最小成本 (double类型)
	std::vector<std::vector<double>> dist(n, std::vector<double>(m, std::numeric_limits<double>::max()));
	// 初始化父节点数组，parent[r][c] 表示在最短路径中到达 (r,c) 的前一个节点
	std::vector<std::vector<std::pair<int32_t, int32_t>>> parent(n, std::vector<std::pair<int32_t, int32_t>>(m, { -1, -1 }));

	// 优先队列存储元素类型定义: {cost (double), {row, col}}
	using PQElement = std::pair<double, std::pair<int32_t, int32_t>>;
	std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

	// 初始化起点
	int32_t start_r = start.first;
	int32_t start_c = start.second;

	// 起点到自身的成本即为起点格子的权重 (直接从 double 类型的 weights 网格获取)
	dist[start_r][start_c] = weights[start_r][start_c];
	pq.push({ dist[start_r][start_c], {start_r, start_c} }); // 存入格式: {成本, {行, 列}}

	while (!pq.empty())
	{
		PQElement top_element = pq.top();
		pq.pop();

		double current_accumulated_cost = top_element.first; // 成本是 double
		std::pair<int32_t, int32_t> current_coords = top_element.second;
		int32_t r = current_coords.first;
		int32_t c = current_coords.second;

		// 浮点数比较：如果 current_accumulated_cost 明显大于 dist[r][c]，则跳过
		// 为避免浮点精度问题，可以加上一个小的 epsilon，但对于Dijkstra，直接比较通常也可以
		// if (current_accumulated_cost > dist[r][c] + std::numeric_limits<double>::epsilon()) {
		if (current_accumulated_cost > dist[r][c])
		{ // 直接比较通常足够
			continue;
		}

		// 如果到达终点
		if (r == end.first && c == end.second)
		{
			std::vector<std::pair<int32_t, int32_t>> path;
			std::pair<int32_t, int32_t> path_tracer_coord = end;
			while (path_tracer_coord.first != -1 && path_tracer_coord.second != -1)
			{
				// path.push_back(path_tracer_coord);
				path.push_back(
					{ (path_tracer_coord.first), path_tracer_coord.second });
				if (path_tracer_coord == start)
				{
					break;
				}
				path_tracer_coord = parent[path_tracer_coord.first][path_tracer_coord.second];
			}

			if (path.empty())
			{
				if (!(start.first == end.first && start.second == end.second))
				{
					return {};
				}
			}
			path.pop_back();
			std::reverse(path.begin(), path.end());
			return path;
		}

		// 使用 `directions` 向量探索当前节点的邻居
		for (const auto& dir : directions)
		{
			int32_t dr_val = dir.first;
			int32_t dc_val = dir.second;

			int32_t next_r = r + dr_val;
			int32_t next_c = c + dc_val;

			if (next_r >= 0 && next_r < static_cast<int32_t>(n) &&
				next_c >= 0 && next_c < static_cast<int32_t>(m))
			{

				if (map[next_r][next_c])
				{ // 检查是否可通行
					// 邻居单元格的成本 (直接从 double 类型的 weights 网格获取)
					double cost_of_next_cell = weights[next_r][next_c];
					// 新的路径总成本
					double new_cost_to_reach_next = current_accumulated_cost + cost_of_next_cell;

					if (new_cost_to_reach_next < dist[next_r][next_c])
					{
						dist[next_r][next_c] = new_cost_to_reach_next;
						parent[next_r][next_c] = { r, c };
						pq.push({ new_cost_to_reach_next, {next_r, next_c} });
					}
				}
			}
		}
	}
	// -------------------------------------- End of Main Logic ----------------------------------------------------

	// 如果没有路径，返回空
	return {};
}

// 找到一个坐标集中距离某点最近的坐标
std::pair<int, int> FindClosestPoint(const std::vector<std::pair<int, int>>& points, const std::pair<int, int>& myPosition)
{
	int min_distance_sq = INT_MAX;
	std::pair<int, int> closest_point;

	for (const auto& point : points)
	{
		int dx = myPosition.first - point.first;
		int dy = myPosition.second - point.second;
		int current_sq = dx * dx + dy * dy;

		if (current_sq < min_distance_sq)
		{
			min_distance_sq = current_sq;
			closest_point = point;
		}
	}

	return closest_point;
}

// 计算平方距离的辅助函数
int DistanceSq(const std::pair<int, int>& a, const std::pair<int, int>& b)
{
	int dx = a.first - b.first;
	int dy = a.second - b.second;
	return dx * dx + dy * dy;
}

// 按照后一个点距离前一个点最近的原则排序 得到链式结构
void SortPointsByProximity(std::vector<std::pair<int, int>>& points, const std::pair<int, int>& myPosition)
{
	if (points.empty())
		return;

	// 找到离“我的位置”最近的点作为首元素
	int closestIdx = 0;
	int minDist = DistanceSq(points[0], myPosition);
	for (int i = 1; i < points.size(); ++i)
	{
		int currDist = DistanceSq(points[i], myPosition);
		if (currDist < minDist)
		{
			minDist = currDist;
			closestIdx = i;
		}
	}
	swap(points[0], points[closestIdx]);

	// 依次找到离前一个点最近的后续元素
	for (int i = 1; i < points.size(); ++i)
	{
		closestIdx = i;
		minDist = DistanceSq(points[i], points[i - 1]);
		for (int j = i + 1; j < points.size(); ++j)
		{
			int currDist = DistanceSq(points[j], points[i - 1]);
			if (currDist < minDist)
			{
				minDist = currDist;
				closestIdx = j;
			}
		}
		swap(points[i], points[closestIdx]);
	}
}