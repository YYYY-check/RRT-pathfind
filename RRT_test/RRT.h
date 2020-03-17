#include<set>
#include<vector>
#include<list>
#include<ctime>
#include<iterator>
using namespace std;
struct point
{
	float x, y;
	bool operator==(const point& coor_xy) {
		return((x == coor_xy.x) & (y == coor_xy.y));
	}
};
struct Node
{
	point coordinate;
	Node* father;
	Node() = default;
	Node(point coordinate_, Node* father_ = nullptr) {
		coordinate = coordinate_;
		father = father_;
	}
};
using Node_pointer=Node *;
struct obstacle {
	//�ϰ�������
	//�ϰ�����·��Լ��������˫�򷨱�ʾ
	float x_c, y_c, R_c;
	float vx, vy;
	obstacle() = default;
};
using obs_set=set<obstacle*>;
class RRT {
public:
	list<Node>search_Node;
	list<float>path_x_rrt, path_y_rrt;
	//set<Node*>vertices;//����ָ������,����search_Nodeָ��
	void generate_Node_rand(point* end_point, Node* rand_node);
	bool arrive_check(Node* current, const point* end_point);
	bool check_crash(point* new_point, obs_set obs_list, Node* node_father);
	bool find_father_node(Node*rand_);
	void find_path_rrt(point* start_, point* end_, set<obstacle*>& obs_list);
	RRT(int* p_, float* step_) :p(*p_), step(*step_) {};
	bool compare_dis(list<Node>::iterator &first, list<Node>::iterator &secend, Node_pointer rand_);
private:
	int p;
	float step;
	float map_size[2] = { 40,20 };//map_size[0]��ʾ��·���ȣ�map_size[1]��ʾ·��һ��
	float dis_len = 0.5;//�����ڳ�βԲ�ĵ���ͷԲ�ľ���
	float R_car = 1;//����Բ�İ뾶

};