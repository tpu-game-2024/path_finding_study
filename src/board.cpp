#include "board.h"

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


bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>> &mass) const
{
	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	// 経路探索
	std::multimap<float, Point> q;
	mass[始点.y][始点.x].visit(始点, mass[始点.y][始点.x]);
	q.insert({Point::distance(始点, 終点), 始点});
	while(!q.empty())
	{
		Point current = q.begin()->second;
		int distance = mass[current.y][current.x].getSteps();
		q.erase(q.begin());
		mass[current.y][current.x].close();

		const static Point move_Amount[] = {{-1, 0}, {+1, 0}, {0, -1}, {0, +1}};
		for(const auto& 移動 : move_Amount)
		{
			Point next_Point = current + 移動;
			Mass &next_Mass = mass[next_Point.y][next_Point.x];
			if(map_[next_Point.y][next_Point.x].canMove() && !next_Mass.isClosed())
			{
				float total_Step = static_cast<float>(distance) + next_Mass.getCost();
				int before_Total_Step = next_Mass.getSteps();
				if(0 <= before_Total_Step)// 訪れたことがあるとき
				{
					if(before_Total_Step <= total_Step) continue;// 以前の方が歩数が少ないとき
					for(auto it = q.begin(); it != q.end(); ++it)
					{
						if(it->second == next_Point) { q.erase(it); break;}
					}
				}
				next_Mass.visit(current, next_Mass);
				q.insert({total_Step + Point::distance(next_Point, 終点), next_Point});

				if(next_Point == 終点)
				{
					Point& walked_Place = mass[終点.y][終点.x].GetParent();
					while(walked_Place != 始点)// 始点につくまで
					{
						Mass& m = mass[walked_Place.y][walked_Place.x];
						m.set(Mass::WAYPOINT);
						walked_Place = mass[walked_Place.y][walked_Place.x].GetParent();
					}
					return true;
				}
			}
		}
	}
}
