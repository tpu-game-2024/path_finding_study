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
	mass[start.y][start.x].set(Mass::START);
	mass[goal.y][goal.x].set(Mass::GOAL);

	// 経路探索
	std::multimap<float, Point> q;
	mass[start.y][start.x].visit(start, mass[start.y][start.x]);
	q.insert({ Point::distance(start,goal),start });
	while (!q.empty())
	{
		Point current = q.begin()->second;
		int distance = mass[current.y][current.x].getSteps();
		q.erase(q.begin());
		mass[current.y][current.x].close();

		const static Point moveAmount[] = { {-1,0},{+1,0},{0,-1},{0,+1} };
		for (const auto& move : moveAmount)
		{
			Point next = current + move;
			Mass& nextMass = mass[next.y][next.x];
			if (!map_[next.y][next.x].canMove() || nextMass.isClosed()) continue;
			float stepsFromStart = static_cast<float>(distance) + nextMass.getCost();
			int previousSteps = nextMass.getSteps();
			if (0 <= previousSteps)
			{
				if (previousSteps <= stepsFromStart)continue;
				for (auto it = q.begin(); it != q.end(); ++it)
				{
					if (it->second == next) { q.erase(it); break; }
				}
			}
			nextMass.visit(current, nextMass);
			q.insert({ static_cast<float>(stepsFromStart) + Point::distance(next,goal),next });

			if (next != goal)continue;
			Point& walkingPosition = mass[goal.y][goal.x].getParent();
			while (walkingPosition != start)
			{
				Mass& m = mass[walkingPosition.y][walkingPosition.x];
				m.set(Mass::WAYPOINT);
				walkingPosition = mass[walkingPosition.y][walkingPosition.x].getParent();
			}
			return true;
		}
	}
}
