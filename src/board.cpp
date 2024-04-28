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
	std::multimap<int, Point> q;  //multimapで優先度付きキューを実装
	mass[始点.y][始点.x].visit(始点, mass[始点.y][始点.x]);
	q.insert({0, 始点});

	while (!q.empty())
	{
		int distance = q.begin()->first;
		Point 現在 = q.begin()->second;
		q.erase(q.begin());
		mass[現在.y][現在.x].close();

		const static Point 移動量[] = { {-1, 0}, {+1, 0}, {0, -1}, {0, +1}};
		for (const auto& 移動 : 移動量)
		{
			Point 次 = 現在 + 移動;
			Mass& 次のマス = mass[次.y][次.x];

			//次のマスに移動可能かつ次のマスがオープンリストの中にある
			if (map_[次.y][次.x].canMove() && !次のマス.isClosed())
			{
				int 始点からの歩数 = distance + 1;
				int 以前の歩数 = 次のマス.getSteps();

				if (0 <= 以前の歩数)
				{
					if (以前の歩数 <= 始点からの歩数)
						continue; //以前の方が距離が少ない

					//古いキーの消去
					auto a = q.equal_range(以前の歩数);
					for (auto it = a.first; it != a.second; ++it)
					{
						if (it->second == 次)
						{
							q.erase(it); 
							break;
						}
					}
				}
				次のマス.visit(現在, 次のマス);
				q.insert({ 始点からの歩数, 次 });

				if (次 == 終点)
				{
					//終点から遡ってWAYPOINTを設定する
					Point& 歩いた場所 = mass[終点.y][終点.x].getParent();

					while (歩いた場所 != 始点)
					{
						//歩いた場所に印をつける(見やすさの為に始点は書き換えない)
						Mass& m = mass[歩いた場所.y][歩いた場所.x];
						m.set(Mass::WAYPOINT);
						歩いた場所 = mass[歩いた場所.y][歩いた場所.x].getParent();
					}
					return true;
				}
			}
		}

	}


}
