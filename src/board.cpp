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

// A* implementation
bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>> &mass) const
{
	std::priority_queue<Node, std::vector<Node>, std::greater<>> openList;
	std::unordered_map<Point, Node, PointHash> allNodes;

	Node startNode = {始点, 0, heuristic(始点, 終点)};
	openList.push(startNode);
	allNodes[始点] = startNode;

	while (!openList.empty()) {
		Node current = openList.top();
		openList.pop();

		if (current.position == 終点) {
			apply_root(current.position, mass);
			// 上書きされるStart&Goalを再設定
			mass[始点.y][始点.x].set(Mass::START);
			mass[終点.y][終点.x].set(Mass::GOAL);
			return true;
		}

		for (Point dir : dirs) {
			Point pos = current.position + dir;
			if (!mass[pos.y][pos.x].canMove()) continue;

			float gCost = current.gCost + mass[pos.y][pos.x].getCost();
			Node currentNode = {pos, gCost, 0};
			currentNode.fCost = gCost + heuristic(pos, 終点);

			if (allNodes.find(pos) == allNodes.end() || gCost < allNodes[pos].gCost) {
				openList.push(currentNode);
				allNodes[pos] = currentNode;
				mass[pos.y][pos.x].setParent(current.position);
			}
		}
	}

	return false;
}
