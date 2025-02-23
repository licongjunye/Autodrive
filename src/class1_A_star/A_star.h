#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <matplotlibcpp.h>

class AStar
{
private:
    // 定义节点代价
    typedef struct {
        double f = 0;
        double g = 0;
        double h = 0;
    } NodesCost;

    // 定义行列坐标
    typedef struct {
        int row = -1;
        int col = -1;
    } SubPos;

    int rows_ = 20;
    int cols_ = 20;
    int start_node_ind_ = 3;
    int goal_node_ind_ = rows_ * cols_ - 3;
    SubPos start_node_sub_;
    SubPos goal_node_sub_;
    std::shared_ptr<std::vector<std::vector<int>>> obs_matrix_;
    std::shared_ptr<std::unordered_map<int, NodesCost>> open_list_;
    std::shared_ptr<std::unordered_map<int, NodesCost>> close_list_;
    std::shared_ptr<std::unordered_map<int, std::vector<int>>> path_list_;

public:
    AStar() {}
    ~AStar() {}

    // 主函数
    bool Excute();

    // 定义障碍物区域
    void DefObsZone();

    // 线性坐标转为行列坐标
    bool Ind2Sub(int ind, SubPos *sub);

    // 行列坐标转为线性坐标
    bool Sub2Ind(int *ind, SubPos sub);

    // 预处理
    void PreProcess();

    // 路径搜索
    void SearchPath();

    // 找到最新的f所对应的节点
    int FindFminNode();

    // 打印和画图
    void PrintAndPLot();

    // 获得某节点周边可行子节点
    std::vector<int> GetChildNodes(int parent_node);
};
