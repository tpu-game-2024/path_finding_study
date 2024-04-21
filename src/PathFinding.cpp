#include "board.h"

int main()
{
	std::vector<std::string> map_data = {
	//   0123456789ab
		"############",// 0 枠で囲って面倒な範囲チェックを省く
		"#          #",// 1
		"#          #",// 2
		"#          #",// 3
		"#          #",// 4
		"#          #",// 5
		"#          #",// 6
		"#          #",// 7
		"#          #",// 8
		"#          #",// 9
		"#          #",// a
		"############",// b
	};
	Board board(map_data);

	// 経路探索
	Point 始点 = { 9, 9 };
	Point 終点 = { 2, 2 };
	std::vector<std::vector<Mass>> mass = board.setup();
	board.find(始点, 終点, mass);

	board.show(mass);// 経路の可視化

	system("PAUSE");// 終る前に入力をうながす
}
