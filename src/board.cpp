#include "board.h"
#include <random>
#include <queue>

std::map<MassInfo::status, MassData> MassInfo::statusData =
{
	{ BLANK, { 1.0f, ' '}},
	{ WALL,  {-1.0f, '#'}},
	{ WATER, { 3.0f, '~'}},
	{ ROAD,  { 0.3f, '$'}},

	// 動的な要素
	{ START,	{-1.0f, 'S'}},
	{ GOAL,		{-1.0f, 'G'}},
	{ WAYPOINT, {-1.0f, 'o'}},

	{ INVALID,  {-1.0f, '\0'}},
};


bool Board::find(const Point& StartPoint, const Point& GoalPoint, std::vector<std::vector<MassInfo>> &mass) const
{
	mass[StartPoint.y][StartPoint.x].set(MassInfo::START);
	mass[GoalPoint.y][GoalPoint.x].set(MassInfo::GOAL);

	std::mt19937 engine;

	// 経路探索
	std::multimap<float,Point> q;
	mass[StartPoint.y][StartPoint.x].visit(StartPoint, mass[StartPoint.y][StartPoint.x]);

	q.insert({ Point::distance(StartPoint,GoalPoint),StartPoint});
	
	while (!q.empty())
	{
		Point nowPoint = q.begin()->second;

		int distanse = mass[nowPoint.y][nowPoint.x].get_checkStep();

		q.erase(q.begin());
		mass[nowPoint.y][nowPoint.x].set_AddFlag();
		const static Point moveValue[] = {{-1,0},{+1,0},{0,-1},{0,+1}};
		for (const auto& movePoint : moveValue) 
		{
			Point nextPoint = nowPoint + movePoint;
			MassInfo& nextMassInfo = mass[nextPoint.y][nextPoint.x];

			bool Check_CanMoveNext = (map_[nextPoint.y][nextPoint.x].canMove());
			bool Check_ListFlagFalse = (!nextMassInfo.get_closedListAddFlag());

			if (Check_CanMoveNext && Check_ListFlagFalse)
			{
				float start_To_distanse = static_cast<float>(distanse) + nextMassInfo.getCost();
				int before_To_distanse = nextMassInfo.get_checkStep();

				bool Check_AlreadyVisit = (0 <= before_To_distanse);

				if (Check_AlreadyVisit)
				{
					bool Check_SmallBefore = (before_To_distanse <= start_To_distanse);

					if (Check_SmallBefore) continue;

					//古いキーの削除(浮動小数点のため誤差を考え全探索)

					for (auto it = q.begin(); it != q.end(); ++it)
					{
						if (it->second == nextPoint) { q.erase(it); break; }
					}
				}

				nextMassInfo.visit(nowPoint, nextMassInfo);
				q.insert({static_cast<float>(start_To_distanse) + Point::distance(nextPoint,GoalPoint) ,nextPoint});

				bool Check_GoalIn = (nextPoint == GoalPoint);

				if (Check_GoalIn)
				{//終点から遡りWAYPONTを設定
					Point& movedPoint = mass[GoalPoint.y][GoalPoint.x].get_visitedPoint();
					while (movedPoint != StartPoint)
					{//歩いた場所に印をつける(見易さのため始点は換えない)
						MassInfo& m = mass[movedPoint.y][movedPoint.x];
						m.set(MassInfo::WAYPOINT);
						movedPoint = mass[movedPoint.y][movedPoint.x].get_visitedPoint();
					}
					return true;
				}	
			}
		}






		//記録
		/*
		Point 現在 = 開覧.front(); 開覧.pop();
		const static Point 移動量[] = {{-1,0},{+1,0},{0,-1},{0,+1}};
		for (const auto& 移動 : 移動量) 
		{
			Point 次 = 現在 + 移動;
			Mass& 次点 = mass[次.y][次.x];
			if (map_[次.y][次.x].canMove() && !次点.取得_既訪())
			{
				次点.訪問(現在);
				開覧.push(次);

				if (次 == 終点) 
				{//終点から遡りWAYPONTを設定
					Point& 歩いた場所 = mass[終点.y][終点.x].取得_来訪();
					while (歩いた場所 != 始点)
					{//歩いた場所に印をつける(見易さのため始点は換えない)
						Mass& m = mass[歩いた場所.y][歩いた場所.x];
						m.set(Mass::WAYPOINT);
						歩いた場所 = mass[歩いた場所.y][歩いた場所.x].取得_来訪();
					}
					return true;
				}
			}
		}

		// 歩いた場所に印をつける(見やすさのために始点は書き換えない)
		if (現在 != 始点){mass[現在.y][現在.x].set(Mass::WAYPOINT);}
		// 終点に向かって歩く
		int 距離x = 終点.x - 現在.x;
		int 距離y = 終点.y - 現在.y;
		Point 左右 = 現在; 
		左右.x += (距離x > 0) - (距離x < 0);
		Point 上下 = 現在; 
		上下.y += (距離y > 0) - (距離y < 0);
		if (距離x * 距離x < 距離y * 距離y)
		{
			//Y軸方向ダメならX軸方向
			if (map_[上下.y][上下.x].canMove()) { 現在 = 上下; continue; }
			if (map_[左右.y][左右.x].canMove()) { 現在 = 左右; continue; }
		}
		else
		{
			//X軸方向ダメならY軸方向
			if (map_[左右.y][左右.x].canMove()) { 現在 = 左右; continue; }
			if (map_[上下.y][上下.x].canMove()) { 現在 = 上下; continue; }
		}
		//詰んだ時
		if (停止 != 現在) 
		{
			停止 = 現在;
			段階 = 1;
		}
		else 
		{段階++;}
		for (int i = 0; i < 段階; i++)
		{
			int 四方 = ランダム() % 4;//四方向の乱数
			Point 次点 = 現在;
			次点.x += (四方 == 0) - (四方 == 1);
			次点.y += (四方 == 2) - (四方 == 3);
			if (map_[次点.y][次点.x].canMove())
			{
				現在 = 次点;
				mass[現在.y][現在.x].set(Mass::WAYPOINT);
			}
		}

		if (距離x * 距離x < 距離y * 距離y)
		{
			現在.y += (距離y > 0) - (距離y < 0);
		}
		else
		{
			現在.x += (距離x > 0) - (距離x < 0);
		}
		if (現在.x < 終点.x) { 現在.x++; continue; }
		if (終点.x < 現在.x) { 現在.x--; continue; }
		if (現在.y < 終点.y) { 現在.y++; continue; }
		if (終点.y < 現在.y) { 現在.y--; continue; }
		*/
	}
	return true;
}
