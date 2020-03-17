// RRT_test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include"RRT.h"
int main()
{
	int pro = 50;
	float step = 2;
	RRT test(&pro, &step);
	point start_point = { 0,0 };
	point end_point = { 40,0 };
	obs_set Obslist;
	obstacle Obs;
	Obs.x_c = 20, Obs.y_c = 0, Obs.R_c = 5, Obs.vx = 0, Obs.vy = 0;
	Obslist.insert(&Obs);
	test.find_path_rrt(&start_point, &end_point, Obslist);
	list<float>::iterator i = test.path_x_rrt.begin();
	list<float>::iterator j = test.path_y_rrt.begin();
	while (i != test.path_x_rrt.end()) {
		cout << *i << "," << *j << endl;
		++i;
		++j;
	}
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
