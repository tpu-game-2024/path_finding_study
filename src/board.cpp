#include "board.h"
#include <random>
#include <queue>
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

	std::mt19937 ランダム;

	// 経路探索
	std::multimap<float,Point> q;
	mass[始点.y][始点.x].訪問(始点, mass[始点.y][始点.x]);
	q.insert({ Point::distance(始点,終点),始点});
	//開覧.push(始点);
	//Point 現在 = 始点;
	//int 段階 = 0;
	//Point 停止 = 終点;//取り敢えずの初期化
	while (!q.empty())
	{
		
		Point 現在 = q.begin()->second;

		int 距離 = mass[現在.y][現在.x].取得_段階();

		q.erase(q.begin());
		mass[現在.y][現在.x].閉じる();
		const static Point 移動量[] = {{-1,0},{+1,0},{0,-1},{0,+1}};
		for (const auto& 移動 : 移動量) 
		{
			Point 次 = 現在 + 移動;
			Mass& 次点 = mass[次.y][次.x];
			if (map_[次.y][次.x].canMove() && !次点.取得_追加フラグ())
			{
				float 始点からの歩数距離 = static_cast<float>(距離) + 次点.getCost();
				int 以前からの歩数距離 = 次点.取得_段階();
				if (0 <= 以前からの歩数距離)
				{
					if (以前からの歩数距離 <= 始点からの歩数距離)continue;
					//古いキーの削除(浮動小数点のため誤差を考え全探索)
					//auto a = q.equal_range(以前からの歩数距離);
					for (auto it = q.begin(); it != q.end(); ++it)
					{
						if (it->second == 次) { q.erase(it); break; }
					}
				}

				次点.訪問(現在, 次点);
				q.insert({static_cast<float>(始点からの歩数距離) + Point::distance(次,終点) ,次});

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
