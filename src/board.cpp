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


bool Board::find(const Point& start, const Point& goal, std::vector<std::vector<Mass>>& mass) const
{
	Mass& startMass = mass[start.y][start.x];
	Mass& goalMass = mass[goal.y][goal.x];

	//スタートとゴールを設定 
	startMass.set(Mass::START);
	goalMass.set(Mass::GOAL);

	std::multimap<float, Point> openList;

	//スタートをリストにいれる
	startMass.visit(start, startMass);
	openList.insert({ Point::distance(start,goal),start });

	while (!openList.empty())
	{
		//探索するマスをpop
		Point current = openList.begin()->second;
		openList.erase(openList.begin());

		//探索するマスの歩数を取得して訪問済みにする
		Mass& currentMass = mass[current.y][current.x];
		int distance = currentMass.getSteps();
		currentMass.close();

		const static Point directions[] = { {-1,0},{+1,0},{0,-1},{0,+1} };
		for (const Point& move : directions)
		{
			Point next = current + move;
			Mass& nextMass = mass[next.y][next.x];

			//動けるマスで訪問していなかったら 
			if (map_[next.y][next.x].canMove() && !nextMass.isClosed())
			{
				float steps = static_cast<float>(distance) + nextMass.getCost();
				int prevSteps = nextMass.getSteps();

				//すでに訪れていたら 
				if (0 <= prevSteps)
				{
					//以前のほうが距離が少なかったら次へ
					if (prevSteps <= steps) continue;

					//キーの削除
					for (auto it = openList.begin(); it != openList.end(); ++it)
					{
						if (it->second == next) { openList.erase(it); break; }
					}
				}

				//訪問済みにする
				nextMass.visit(current, nextMass);
				openList.insert({ steps + Point::distance(next, goal),next });

				if (next == goal)
				{
					Point& pathPoint = goalMass.getParent();

					//親を遡ってWayPointの設定
					while (pathPoint != start)
					{
						Mass& m = mass[pathPoint.y][pathPoint.x];
						m.set(Mass::WAYPOINT);
						pathPoint = m.getParent();
					}

					return true;
				}
			}
		}
	}



	return true;
}
