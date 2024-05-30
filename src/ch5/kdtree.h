//
// Created by xiang on 2021/9/22.
//

#ifndef SLAM_IN_AUTO_DRIVING_KDTREE_H
#define SLAM_IN_AUTO_DRIVING_KDTREE_H

#include "common/eigen_types.h"
#include "common/point_types.h"

#include <glog/logging.h>
#include <map>
#include <queue>

namespace sad {

/// Kd树节点，二叉树结构，内部用祼指针，对外一个root的shared_ptr
struct KdTreeNode {
    int id_ = -1;
    int point_idx_ = 0;            // 点的索引
    int axis_index_ = 0;           // 分割轴
    float split_thresh_ = 0.0;     // 分割位置
    KdTreeNode* left_ = nullptr;   // 左子树
    KdTreeNode* right_ = nullptr;  // 右子树

    bool IsLeaf() const { return left_ == nullptr && right_ == nullptr; }  // 是否为叶子
};

/// 用于记录knn结果
struct NodeAndDistance {
    NodeAndDistance(KdTreeNode* node, float dis2) : node_(node), distance2_(dis2) {}
    KdTreeNode* node_ = nullptr;
    float distance2_ = 0;  // 平方距离，用于比较

    bool operator<(const NodeAndDistance& other) const { return distance2_ < other.distance2_; }
};

// /**
//  * 手写kd树
//  * 测试这个kd树的召回!
//  */
// // 将K-d树的点云类型拓展至模板类
// template <typename T>
// class KdTree{
// public:
//     // explicit KdTree() = default;
//     // ~KdTree() { Clear(); }
//     bool BuildTree(const typename::pcl::PointCloud<T>::Ptr& cloud);
//     /// 获取k最近邻
//     bool GetClosestPoint(const T& pt, std::vector<int>& closest_idx, int k = 5);
//     /// 并行为点云寻找最近邻
//     bool GetClosestPointMT(const typename::pcl::PointCloud<T>::Ptr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k = 5);
//     /// 这个被用于计算最近邻的倍数
//     void SetEnableANN(bool use_ann = true, float alpha = 0.1) {
//         approximate_ = use_ann;
//         alpha_ = alpha;
//     }
//     /// 返回节点数量
//     size_t size() const { return size_; }
//     /// 清理数据
//     void Clear();
//     /// 打印所有节点信息
//     void PrintAll();
//     private:
//     /// kdtree 构建相关
//     /**
//      * 在node处插入点
//      * @param points
//      * @param node
//      */
//     void Insert(const IndexVec& points, KdTreeNode* node);
//     /**
//      * 计算点集的分割面
//      * @param points 输入点云
//      * @param axis   轴
//      * @param th     阈值
//      * @param left   左子树
//      * @param right  右子树
//      * @return
//      */
//     bool FindSplitAxisAndThresh(const IndexVec& point_idx, int& axis, float& th, IndexVec& left, IndexVec& right);
//     void Reset();
//     /// 两个点的平方距离
//     static inline float Dis2(const Vec3f& p1, const Vec3f& p2) { return (p1 - p2).squaredNorm(); }
//     // Knn 相关
//     /**
//      * 检查给定点在kdtree node上的knn，可以递归调用
//      * @param pt     查询点
//      * @param node   kdtree 节点
//      */
//     void Knn(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;
//     /**
//      * 对叶子节点，计算它和查询点的距离，尝试放入结果中
//      * @param pt    查询点
//      * @param node  Kdtree 节点
//      */
//     void ComputeDisForLeaf(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;
//     /**
//      * 检查node下是否需要展开
//      * @param pt   查询点
//      * @param node Kdtree 节点
//      * @return true if 需要展开
//      */
//     bool NeedExpand(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const;

//     int k_ = 5;                                   // knn最近邻数量
//     std::shared_ptr<KdTreeNode> root_ = nullptr;  // 根节点，其实这里维护的是树数据结构
//     std::vector<Vec3f> cloud_;                    // 输入点云
//     std::unordered_map<int, KdTreeNode*> nodes_;  // for bookkeeping，这里维护的是hash map数据结构

//     size_t size_ = 0;       // 叶子节点数量
//     int tree_node_id_ = 0;  // 为kdtree node 分配id

//     // 近似最近邻
//     bool approximate_ = true;
//     float alpha_ = 0.1;
// };

/*************************原来的类****************************/
class KdTree {
   public:
    explicit KdTree() = default;
    ~KdTree() { Clear(); }

    bool BuildTree(const CloudPtr& cloud);

    /// 获取k最近邻
    bool GetClosestPoint(const PointType& pt, std::vector<int>& closest_idx, int k = 5);

    /// 并行为点云寻找最近邻
    bool GetClosestPointMT(const CloudPtr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k = 5);

    /// 这个被用于计算最近邻的倍数
    void SetEnableANN(bool use_ann = true, float alpha = 0.1) {
        approximate_ = use_ann;
        alpha_ = alpha;
    }

    /// 返回节点数量
    size_t size() const { return size_; }

    /// 清理数据
    void Clear();

    /// 打印所有节点信息
    void PrintAll();

   private:
    /// kdtree 构建相关
    /**
     * 在node处插入点
     * @param points
     * @param node
     */
    void Insert(const IndexVec& points, KdTreeNode* node);

    /**
     * 计算点集的分割面
     * @param points 输入点云
     * @param axis   轴
     * @param th     阈值
     * @param left   左子树
     * @param right  右子树
     * @return
     */
    bool FindSplitAxisAndThresh(const IndexVec& point_idx, int& axis, float& th, IndexVec& left, IndexVec& right);

    void Reset();

    /// 两个点的平方距离
    static inline float Dis2(const Vec3f& p1, const Vec3f& p2) { return (p1 - p2).squaredNorm(); }

    // Knn 相关
    /**
     * 检查给定点在kdtree node上的knn，可以递归调用
     * @param pt     查询点
     * @param node   kdtree 节点
     */
    void Knn(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    /**
     * 对叶子节点，计算它和查询点的距离，尝试放入结果中
     * @param pt    查询点
     * @param node  Kdtree 节点
     */
    void ComputeDisForLeaf(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    /**
     * 检查node下是否需要展开
     * @param pt   查询点
     * @param node Kdtree 节点
     * @return true if 需要展开
     */
    bool NeedExpand(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const;

    int k_ = 5;                                   // knn最近邻数量
    std::shared_ptr<KdTreeNode> root_ = nullptr;  // 根节点，其实这里维护的是树数据结构
    std::vector<Vec3f> cloud_;                    // 输入点云
    std::unordered_map<int, KdTreeNode*> nodes_;  // for bookkeeping，这里维护的是hash map数据结构

    size_t size_ = 0;       // 叶子节点数量
    int tree_node_id_ = 0;  // 为kdtree node 分配id

    // 近似最近邻
    bool approximate_ = true;
    float alpha_ = 0.1;
};



// template <typename T>
// bool KdTree<T>::BuildTree(const typename::pcl::PointCloud<T>::Ptr& cloud) {
//     if (cloud->empty()) {
//         return false;
//     }

//     cloud_.clear();
//     cloud_.resize(cloud->size());
//     for (size_t i = 0; i < cloud->points.size(); ++i) {
//         cloud_[i] = ToVec3f(cloud->points[i]);
//     }

//     Clear();
//     Reset();

//     IndexVec idx(cloud->size());
//     for (int i = 0; i < cloud->points.size(); ++i) {
//         idx[i] = i;
//     }

//     Insert(idx, root_.get());
//     return true;
// }

// template <typename T>
// void KdTree<T>::Insert(const IndexVec &points, KdTreeNode *node) {
//     nodes_.insert({node->id_, node});

//     if (points.empty()) {
//         return;
//     }

//     if (points.size() == 1) {
//         size_++;
//         node->point_idx_ = points[0];
//         return;
//     }

//     IndexVec left, right;
//     if (!FindSplitAxisAndThresh(points, node->axis_index_, node->split_thresh_, left, right)) {
//         size_++;
//         node->point_idx_ = points[0];
//         return;
//     }

//     const auto create_if_not_empty = [&node, this](KdTreeNode *&new_node, const IndexVec &index) {
//         if (!index.empty()) {
//             new_node = new KdTreeNode;
//             new_node->id_ = tree_node_id_++;
//             Insert(index, new_node);
//         }
//     };

//     create_if_not_empty(node->left_, left);
//     create_if_not_empty(node->right_, right);
// }
// template <typename T>
// bool KdTree<T>::GetClosestPoint(const T &pt, std::vector<int> &closest_idx, int k) {
//     if (k > size_) {
//         LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << size_;
//         return false;
//     }
//     k_ = k;

//     std::priority_queue<NodeAndDistance> knn_result;      // 重载的小于规则，是从大到小排序的
//     Knn(ToVec3f(pt), root_.get(), knn_result);

//     // 排序并返回结果
//     closest_idx.resize(knn_result.size());
//     for (int i = closest_idx.size() - 1; i >= 0; --i) {
//         // 倒序插入
//         closest_idx[i] = knn_result.top().node_->point_idx_;
//         knn_result.pop();
//     }
//     return true;
// }

// template <typename T>
// bool KdTree<T>::GetClosestPointMT(const typename::pcl::PointCloud<T>::Ptr &cloud, std::vector<std::pair<size_t, size_t>> &matches, int k) {
//     matches.resize(cloud->size() * k);

//     // 索引
//     std::vector<int> index(cloud->size());
//     for (int i = 0; i < cloud->points.size(); ++i) {
//         index[i] = i;
//     }

//     std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &cloud, &matches, &k](int idx) {
//         std::vector<int> closest_idx;
//         GetClosestPoint(cloud->points[idx], closest_idx, k);
//         // 记录每一个激光点的k近邻
//         for (int i = 0; i < k; ++i) {
//             matches[idx * k + i].second = idx;
//             if (i < closest_idx.size()) {
//                 matches[idx * k + i].first = closest_idx[i];
//             } else {
//                 matches[idx * k + i].first = math::kINVALID_ID;
//             }
//         }
//     });

//     return true;
// }

// template <typename T>
// void KdTree<T>::Knn(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const {
//     if (node->IsLeaf()) {
//         // 如果是叶子，检查叶子是否能插入
//         ComputeDisForLeaf(pt, node, knn_result);
//         return;
//     }

//     // 看pt落在左还是右，优先搜索pt所在的子树
//     // 然后再看另一侧子树是否需要搜索
//     KdTreeNode *this_side, *that_side;
//     if (pt[node->axis_index_] < node->split_thresh_) {
//         this_side = node->left_;
//         that_side = node->right_;
//     } else {
//         this_side = node->right_;
//         that_side = node->left_;
//     }

//     Knn(pt, this_side, knn_result);
//     if (NeedExpand(pt, node, knn_result)) {  // 注意这里是跟自己比
//         Knn(pt, that_side, knn_result);
//     }
// }

// template <typename T>
// bool KdTree<T>::NeedExpand(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const {
//     if (knn_result.size() < k_) {
//         return true;
//     }

//     if (approximate_) {
//         float d = pt[node->axis_index_] - node->split_thresh_;
//         if ((d * d) < knn_result.top().distance2_ * alpha_) {
//             return true;
//         } else {
//             return false;
//         }
//     } else {
//         // 检测切面距离，看是否有比现在更小的
//         float d = pt[node->axis_index_] - node->split_thresh_;
//         if ((d * d) < knn_result.top().distance2_) {
//             return true;
//         } else {
//             return false;
//         }
//     }
// }

// template <typename T>
// void KdTree<T>::ComputeDisForLeaf(const Vec3f &pt, KdTreeNode *node,
//                                std::priority_queue<NodeAndDistance> &knn_result) const {
//     // 比较与结果队列的差异，如果优于最远距离，则插入
//     float dis2 = Dis2(pt, cloud_[node->point_idx_]);
//     if (knn_result.size() < k_) {
//         // results 不足k
//         knn_result.emplace(node, dis2);
//     } else {
//         // results等于k，比较current与max_dis_iter之间的差异
//         if (dis2 < knn_result.top().distance2_) {
//             knn_result.emplace(node, dis2);
//             knn_result.pop();
//         }
//     }
// }

// template <typename T>
// bool KdTree<T>::FindSplitAxisAndThresh(const IndexVec &point_idx, int &axis, float &th, IndexVec &left, IndexVec &right) {
//     // 计算三个轴上的散布情况，我们使用math_utils.h里的函数
//     Vec3f var;
//     Vec3f mean;
//     math::ComputeMeanAndCovDiag(point_idx, mean, var, [this](int idx) { return cloud_[idx]; });
//     int max_i, max_j;
//     // note: Eigen使用maxCoeff和minCoeff函数计算矩阵中的最大值和最小值，但是若想返回矩阵中的最大值和最小值的位置，需要给定相关参数index
//     // 计算各轴方差并取最大方差那个轴作为分割轴，并将平均数作为阈值
//     var.maxCoeff(&max_i, &max_j);
//     axis = max_i;
//     th = mean[axis];

//     for (const auto &idx : point_idx) {
//         if (cloud_[idx][axis] < th) {
//             // 中位数可能向左取整
//             left.emplace_back(idx);
//         } else {
//             right.emplace_back(idx);
//         }
//     }
    
//     // 这里是避免传入的各个点云一样
//     // 边界情况检查：输入的points等于同一个值，上面的判定是>=号，所以都进了右侧
//     // 这种情况不需要继续展开，直接将当前节点设为叶子就行
//     if (point_idx.size() > 1 && (left.empty() || right.empty())) {
//         return false;
//     }

//     return true;
// }

// template <typename T>
// void KdTree<T>::Reset() {
//     tree_node_id_ = 0;
//     root_.reset(new KdTreeNode());
//     root_->id_ = tree_node_id_++;
//     size_ = 0;
// }

// template <typename T>
// void KdTree<T>::Clear() {
//     for (const auto &np : nodes_) {
//         if (np.second != root_.get()) {
//             delete np.second;
//         }
//     }

//     nodes_.clear();
//     root_ = nullptr;
//     size_ = 0;
//     tree_node_id_ = 0;
// }

// template <typename T>
// void KdTree<T>::PrintAll() {
//     for (const auto &np : nodes_) {
//         auto node = np.second;
//         if (node->left_ == nullptr && node->right_ == nullptr) {
//             LOG(INFO) << "leaf node: " << node->id_ << ", idx: " << node->point_idx_;
//         } else {
//             LOG(INFO) << "node: " << node->id_ << ", axis: " << node->axis_index_ << ", th: " << node->split_thresh_;
//         }
//     }
// }
}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_KDTREE_H

