#include "board.h"
#include <random>
#include <map>

std::map<Mass::status, MassInfo> Mass::statusData =
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


bool Board::find(const Point& start, const Point& goal, std::vector<std::vector<Mass>> &mass) const
{
	mass[start.y][start.x].set(Mass::START);
	mass[goal.y][goal.x].set(Mass::GOAL);

	//経路探索(A*アルゴリズム)
	std::multimap<float, Point> q;
	mass[start.y][start.x].visit(start, mass[start.y][start.x]);
	q.insert({ Point::distance(start,goal),start });
	const static Point move_direction[] = { {-1,0},{1,0},{0,-1},{0,1} };

	while (!q.empty())
	{
		Point now = q.begin()->second;
		int distance = mass[now.y][now.x].getSteps();
		q.erase(q.begin());
		mass[now.y][now.x].close();

		for (const auto& move : move_direction)//範囲for文
		{
			Point next = now + move;
			Mass& next_square = mass[next.y][next.x];

			if (map_[next.y][next.x].canMove() && !next_square.isClosed())
			{
				float start2stepcount = static_cast<float>(distance) + next_square.getCost();
				int before_stepcount = next_square.getSteps();

				if (0 <= before_stepcount)//既に訪れた
				{
					if (before_stepcount <= start2stepcount) { continue; }
					//古いキー削除(全探索)
					for (auto it = q.begin(); it != q.end(); it++)
					{
						if (it->second == next) { q.erase(it); break; }
					}
				}

				next_square.visit(now, next_square);
				q.insert({ static_cast<float>(start2stepcount) + Point::distance(next,goal), next });

				if (next == goal)
				{
					//ゴールから遡り、通った道にWAYPOINT設定＆印をつける
					Point& passed_road = mass[goal.y][goal.x].getParent();
					while (passed_road != start)
					{
						mass[passed_road.y][passed_road.x].set(Mass::WAYPOINT);
						passed_road = mass[passed_road.y][passed_road.x].getParent();
					}

					return true;
				}
			}
		}
	}

	// 経路探索(乱数)
	//Point now = start;
	//
	//std::mt19937 prng;//メルセンヌ・ツイスタ
	//int repeats = 0;
	//Point stop = goal;
	//
	//while (now != goal) {
	//	// 歩いた場所に印をつける(見やすさのために始点は書き換えない)
	//	if (now != start){mass[now.y][now.x].set(Mass::WAYPOINT);}
	//
	//	// 終点に向かって歩く
	//	int dx = goal.x - now.x;
	//	int dy = goal.y - now.y;
	//
	//	//()内がtrue(1)かfalse(0)で判定
	//	Point left_right = now; left_right.x += (dx > 0) - (dx < 0);
	//	Point up_down = now; up_down.y += (dy > 0) - (dy < 0);
	//
	//	if (dx * dx < dy * dy) 
	//	{ 
	//		//Y近づく→ダメならX動く
	//		if (map_[up_down.y][up_down.x].canMove()) { now = up_down; continue; }
	//		if (map_[left_right.y][left_right.x].canMove()) { now = left_right; continue; }
	//	}
	//	else 
	//	{ 
	//		//X近づく→ダメならY動く
	//		if (map_[left_right.y][left_right.x].canMove()) { now = left_right; continue; }
	//		if (map_[up_down.y][up_down.x].canMove()) { now = up_down; continue; }
	//	}
	//
	//	//XもYも動けない場合
	//	if (stop != now)
	//	{
	//		stop = now;
	//		repeats = 1;
	//	}
	//	else { repeats++; }
	//
	//	for (int i = 0; i < repeats; i++)
	//	{
	//		int r = prng() % 4; //上下左右
	//		Point next = now;
	//		next.x += (r == 0) - (r == 1);
	//		next.y += (r == 2) - (r == 3);
	//		if (map_[next.y][next.x].canMove())
	//		{ 
	//			now = next;
	//			mass[now.y][now.x].set(Mass::WAYPOINT);
	//		}
	//	}
	//}

	return true;
}
