#include"RRT.h"
#define rand_me(x,y) (static_cast<float>(rand()%static_cast<int>(y-x))+x)
#include <iostream>
//#include <iostream>
//#include<ctime>
//#include<iterator>
//#define random(x,y) rand()%(int)(y-x)+x
using namespace std;
bool RRT::check_crash(point* new_point, obs_set obs_list, Node* node_father) {
	//仅考虑主车为一个圆，多个圆模型后续更新
	bool res = true;
	for (auto obs : obs_list) {
		if ((abs(new_point->x - obs->x_c) >= obs->R_c + R_car) || (abs(new_point->y - obs->y_c) >= obs->R_c + R_car)) continue;
		else if (pow(new_point->x - obs->x_c, 2) + pow(new_point->y - obs->y_c, 2) >= pow(R_car + obs->R_c, 2)) continue;
		else {
			res = false;
			break;
		}
	}
	return res;
}
void RRT::generate_Node_rand(point* end_point,Node*rand_node) {
	int pp = rand_me(0, 100);
	if (pp < p) {//
		rand_node->coordinate.x = end_point->x;
		rand_node->coordinate.y = end_point->y;
	}
	else {
		rand_node->coordinate.x = rand_me(0, map_size[0]);
		rand_node->coordinate.y = rand_me((-map_size[1]), map_size[1]);
	}
	//std::cout << pp << endl;
}
bool RRT::arrive_check(Node* current, const point* end_point) {
	if (abs(current->coordinate.x - end_point->x) < 0.1 && abs(current->coordinate.y - end_point->y) < 0.05) {return true;}
	else { return false; }
}
bool RRT::find_father_node(Node*rand_) {
	list<Node>::iterator it = search_Node.begin();
	list<Node>::iterator tem;
	tem = it;
	it++;
	for (; it != search_Node.end(); ++it) {
		if (compare_dis(tem, it, rand_)) {
			tem = it;
		}
	}
	float dis_ = pow(pow(tem->coordinate.x - rand_->coordinate.x, 2) + pow(tem->coordinate.y - rand_->coordinate.y, 2), 0.5);
	if ( dis_> step) {
		rand_->father = &(*tem);
		rand_->coordinate.x = tem->coordinate.x + (-(tem->coordinate.x) + rand_->coordinate.x) * step / dis_;
		rand_->coordinate.y = tem->coordinate.y + (-(tem->coordinate.y) + rand_->coordinate.y) * step / dis_;
		return true;
	}
	else if (dis_ == 0) {
		return false;
	}
	else { 
		rand_->father = &(*tem);
		return true; 
	}
}
void RRT::find_path_rrt(point*start_,point*end_,set<obstacle*>&obs) {
	srand((int)time(NULL));  // 产生随机种子  把0换成NULL也行
	Node rand_node(*start_);
	point rand_point=*start_;
	search_Node.push_back(rand_node);
	for (int i = 0; i < 1000; ++i) {
		generate_Node_rand(end_,&rand_node);
		if (find_father_node(&rand_node)) {
			if (check_crash(&(rand_node.coordinate), obs, rand_node.father)) {
				search_Node.push_back(rand_node);
				if (arrive_check(&rand_node, end_)) { 
					break; }
			}
		}
		else { continue; }
	}
	Node* it = &search_Node.back();//
	do {
		path_x_rrt.push_back(it->coordinate.x);
		path_y_rrt.push_back(it->coordinate.y);
		it = it->father;
	} while (it->father != nullptr);
	search_Node.clear();
}
bool RRT::compare_dis(list<Node>::iterator &first, list<Node>::iterator &secend, Node_pointer rand_) {
	if (abs(first->coordinate.x - rand_->coordinate.x) + abs(first->coordinate.y - rand_->coordinate.y) > abs(secend->coordinate.x - rand_->coordinate.x) + abs(secend->coordinate.y - rand_->coordinate.y)) {
		return true;
	}
	else { return false; }
}