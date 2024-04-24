#include "board.h"
#include <random>

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

	std::mt19937 engine;

	// 経路探索
	Point 現在 = 始点;
	int steps = 0;
	//とりあえずあり得ない場所で初期化	
	Point 停止 = 終点;	
	while (現在 != 終点) 
	{
		// 歩いた場所に印をつける(見やすさのために始点は書き換えない)
		if (現在 != 始点){mass[現在.y][現在.x].set(Mass::WAYPOINT);}
		
		// 終点に向かって歩く
		int dx = 終点.x - 現在.x;
		int dy = 終点.y - 現在.y;
		
		Point 左右 = 現在; 左右.x += (dx > 0) - (dx < 0);
		Point 上下 = 現在; 上下.y += (dy > 0) - (dy < 0);
		
		if (dx * dx < dy * dy)
		{			
			// Y軸方向に近づこうとしてだめならX軸方向に動く
			if (map_[上下.y][上下.x].canMove())
			{ 
				現在 = 上下;
				continue;
			}
			if (map_[左右.y][左右.x].canMove()) 
			{ 
				現在 = 左右;
				continue;
			}
		}
		else
		{
			//X軸方向に近づこうとしてだめならY軸方向に動く
			if (map_[左右.y][左右.x].canMove()) 
			{ 
				現在 = 左右;
				continue;				
			}
			if (map_[上下.y][上下.x].canMove()) 
			{ 
				現在 = 上下;
				continue;
			}
		}
		//動けなかった場合
	
		if (停止 != 現在)
		{
			停止 = 現在;
			steps = 1;
		}
		else
		{
			steps++;
		}
		for (int i = 0; i < steps; i++)
		{
			int r = engine() % 4;
			Point 次 = 現在;

			次.x += (r == 0) - (r == 1);
			次.y += (r == 2) - (r == 3);
			if (map_[次.y][次.x].canMove())
			{
				現在 = 次;
				mass[現在.y][現在.x].set(Mass::WAYPOINT);			
			}
		}
	}

	return true;
}
