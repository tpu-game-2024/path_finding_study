#pragma once
#include <cassert>
#include <iostream>
#include <vector>
#include <map>

struct Point {
	int x = -1;
	int y = -1;

	bool operator == (const Point& rhs) const {
		return x == rhs.x && y == rhs.y;
	}
	// != は == の否定で定義
	bool operator != (const Point& rhs) const {return !(*this == rhs);}

	Point operator+(const Point& rhs) const { return { x + rhs.x, y + rhs.y }; }

	static float distance(const Point& p0, const Point& p1)
	{
		int dx = p1.x - p0.x;
		int dy = p1.y - p0.y;
		return sqrtf(float(dx * dx + dy * dy));
	}
};

struct MassInfo {
	float cost;	// そのマスに行くためのコスト(負ならいけない)
	char chr;	// 表示用の文字
};

class Mass {
public:
	enum status {
		// 環境
		BLANK,		// 空間
		WALL,		// 壁通れない
		WATER,		// 進むのが1/3に遅くなる
		ROAD,		// 進むのが3倍速い

		// 動的な要素
		START,		// 始点
		GOAL,		// ゴール
		WAYPOINT,	// 歩いた場所

		INVALID,	// 無効な値
	};
private:
	bool is_visited_ = false;
	Point parent_;
	static std::map<status, MassInfo> statusData;
	status s_ = BLANK;

public:
	void visit(const Point& parent) { is_visited_ = true; parent_ = parent; }
	//すでに着いたのか
	bool isVisited() const { return is_visited_; }
	//どこから来たのかを返す
	Point& getParent() { return parent_; }


	void set(status s) { s_ = s; }
	void set(char c) {// cの文字を持つstatusを検索して設定する（重い）
		s_ = INVALID;// 見つからなった際の値
		for (auto& x : statusData) { if (x.second.chr == c) { s_ = x.first; return; } }
	}

	const std::string getText() const { return std::string{ statusData[s_].chr}; }

	bool canMove() const { return 0 <= statusData[s_].cost; }
	float getCost() const { return statusData[s_].cost; }
};

class Board {
private:
	std::vector<std::vector<Mass>> map_;

	void initialize(const std::vector<std::string> &map_data)
	{
		size_t 縦 = map_data.size();
		size_t 横 = map_data[0].size();

		map_.resize(縦);
		for (unsigned int y = 0; y < 縦; y++)
		{
			map_[y].resize(横);

			assert(map_data[y].size() == 横);// 整合性チェック
			for(int x = 0; x < 横; x++) {
				map_[y][x].set(map_data[y][x]);
			}
		}
	}

public:
	Board(const std::vector<std::string>& map_data) {initialize(map_data);}
	~Board() {}

	// massの準備(サイズを設定して、map_をコピー)
	std::vector<std::vector<Mass>> setup()
	{
		std::vector<std::vector<Mass>> mass;

		size_t 縦 = map_.size();
		size_t 横 = map_[0].size();

		mass.resize(縦);
		for (unsigned int y = 0; y < 縦; y++)
		{
			mass[y].resize(横);
			std::copy(map_[y].begin(), map_[y].end(), mass[y].begin());
		}

		return mass;
	}

	void show(const std::vector<std::vector<Mass>>& mass) const
	{
		size_t 縦 = mass.size();
		size_t 横 = mass[0].size();

		std::cout << std::endl;// 上を空ける

		for (unsigned int y = 0; y < 縦; y++) {
			std::cout << " ";// 左を空ける

			// 各マスの表示
			for (unsigned int x = 0; x < 横; x++) {
				std::cout << mass[y][x].getText();
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;// 下を空ける
	}

	// 経路探索！
	bool find(const Point& start, const Point& goal, std::vector<std::vector<Mass>>& mass) const;
};
