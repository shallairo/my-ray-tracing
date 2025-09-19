#pragma once
#include "hittable.h" // 必须在最前面
#include <vector>
#include <algorithm>
class hittable;
// Bounds3.hpp
struct Bounds3 {
    vec3 pMin, pMax; // 最小/最大顶点坐标

    Bounds3():pMin(point3(FLT_MAX, FLT_MAX, FLT_MAX)), pMax(point3(-FLT_MAX, -FLT_MAX, -FLT_MAX)) {}
    Bounds3(const vec3&u,const vec3&v):pMin(u),pMax(v){}
    // 计算包围盒表面积（SAH核心）
    float SurfaceArea() const {
        vec3 d = pMax - pMin;
        return 2.0f * (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    }
    inline int maxExtent() {
        auto temp = std::max(abs(pMax[0] - pMin[0]), std::max(abs(pMax[1] - pMin[1]), abs(pMax[2] - pMin[2])));
        if (abs(pMax[0] - pMin[0]) == temp)
        {
            return 0;
        }
        else if (abs(pMax[1] - pMin[1]) == temp)
        {
            return 1;
        }
        else {
            return 2;
        }
    }
    // 返回光线与包围盒的相交距离（tMin）和是否相交
    std::pair<float, bool> IntersectT(const ray& ray) const {
        float tMin = -std::numeric_limits<float>::max();           // 光线进入包围盒的时间
        float tMax = std::numeric_limits<float>::max(); // 光线离开包围盒的时间

        for (int i = 0; i < 3; ++i) {
            float invD = 1.0f / ray.direction()[i];
            float tEnter = (pMin[i] - ray.origin()[i]) * invD;
            float tExit = (pMax[i] - ray.origin()[i]) * invD;

            if (invD < 0) std::swap(tEnter, tExit); // 处理反向光线
            tMin = std::max(tMin, tEnter);        // 更新最大进入时间
            tMax = std::min(tMax, tExit);        // 更新最小离开时间
            if (tMax < tMin) return { 0.0f, false }; // 无相交
        }
        return { tMin, tMax>0 }; // 返回进入距离和相交标志
    }
};

// BVH节点结构
struct BVHNode {
    Bounds3 bounds;
    BVHNode* left = nullptr;
    BVHNode* right = nullptr;
    bool isLeaf = false;
    std::vector<std::shared_ptr<hittable>> objects; // 叶子节点存储物体
// 析构函数（防止内存泄漏）
    BVHNode(){}
~BVHNode() {
    delete left;
    delete right;
}
};
inline Bounds3 uni( Bounds3 b1,  Bounds3 b2)
{
    return Bounds3(Min(b1.pMin, b2.pMin), Max(b1.pMax, b2.pMax));
}
// 计算物体列表的包围盒
Bounds3 ComputeBbox(const std::vector<std::shared_ptr<hittable>>& objects, int start, int end) 
{
    Bounds3 bbox = objects[start]->bounding_box();
    for (int i = start + 1; i < end; i++) {
        bbox = uni(bbox, objects[i]->bounding_box());
    }
    return bbox;
}

// SAH分割：选择最小成本的分割点
int FindBestSplitWithSAH(
    const std::vector<shared_ptr<hittable>>& objects,
    int start, int end, int axis,
    float& minCost, float axisLen
) {
    const int BIN_COUNT = 32; // 空间桶数量
    struct Bin {
        Bounds3 bbox;
        int count = 0;
    } bins[BIN_COUNT];
    // 初始化桶

    int bestSplit = start + (end - start) / 2; // 默认物体索引的中位数分割
    float binWidth = axisLen / BIN_COUNT;
	if (binWidth <= 0.1f) return bestSplit; // 防止除以0
    if (axisLen == 0) return start + (end - start) / 2; // 防止死循环
    // 将物体分配到桶中
    for (int i = start; i < end; i++) {
        float center = objects[i]->getCenter()[axis];
        int binIdx = std::max(0, std::min(BIN_COUNT - 1, (int)((center - objects[start]->bounding_box().pMin[axis]) / binWidth)));
        bins[binIdx].bbox = uni(bins[binIdx].bbox, objects[i]->bounding_box());
        bins[binIdx].count++;
    }

    // 计算每个分割点的SAH成本
    minCost = std::numeric_limits<float>::max();

    for (int split = 1; split < BIN_COUNT; split++) {
        Bounds3 leftBox;
        Bounds3 rightBox;
        int leftCount = 0, rightCount = 0;

        // 左子树包围盒
        for (int i = 0; i < split; i++) {
            if (bins[i].count > 0) { // 只合并有物体的 bin
                leftBox = uni(leftBox, bins[i].bbox);
                leftCount += bins[i].count;
            }
        }

        // 右子树包围盒
        for (int i = split; i < BIN_COUNT; i++) {
            if (bins[i].count > 0) {
                rightBox = uni(rightBox, bins[i].bbox);
                rightCount += bins[i].count;
            }
        }

        // SAH成本 = 左子树成本 + 右子树成本
        float cost = leftBox.SurfaceArea() * leftCount + rightBox.SurfaceArea() * rightCount;
        if (cost < minCost) {
            minCost = cost;
            bestSplit =  split; // 映射回最小成本的包围盒索引
        }
    }
	//将桶索引转换为物体索引
    int splitIndex = start;
    for (int i = start; i < end; i++) {
        float center = objects[i]->getCenter()[axis];
        int binIdx = std::min(BIN_COUNT - 1,
            static_cast<int>((center - objects[start]->bounding_box().pMin[axis]) / binWidth));
        if (binIdx >= bestSplit) { // 此处用bestSplit（桶索引）
            splitIndex = i;
            break;
        }
    }
    return splitIndex;
}

// 递归构建BVH（SAH优化核心）
BVHNode* BuildBVH(
    std::vector<std::shared_ptr<hittable>>& objects,
    int start, int end,
    int maxLeafSize 
) {
    if (start >= end) return nullptr;

    // 1. 计算当前节点包围盒
    Bounds3 bbox = ComputeBbox(objects, start, end);

    // 2. 叶子节点终止条件
    if (end - start <= maxLeafSize) {
        auto node = new BVHNode;
        node->bounds = bbox;
        node->isLeaf = true;
        node->objects.assign(objects.begin() + start, objects.begin() + end);
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }

    // 3. 选择最长轴
    int axis = bbox.maxExtent();
	float minCost = std::numeric_limits<float>::max();
	float axisLen = bbox.pMax[axis] - bbox.pMin[axis];
    // 4. 分割点，确保不会死循环
	//int bestSplit = FindBestSplitWithSAH(objects, start, end, axis, minCost, axisLen);
    int bestSplit = start + (end - start) / 2;
    if (bestSplit == start || bestSplit == end) {
        bestSplit = start + 1;
    }

    auto comparator = [axis](const auto& a, const auto& b) {
        return a->getCenter()[axis] < b->getCenter()[axis];
    };
    std::sort(objects.begin() + start, objects.begin() + end, comparator);

    // 5. 递归构建子树
    BVHNode* left = BuildBVH(objects, start, bestSplit, maxLeafSize);
    BVHNode* right = BuildBVH(objects, bestSplit, end, maxLeafSize);

    auto node = new BVHNode;
    node->bounds = bbox;
    node->left = left;
    node->right = right;
    node->isLeaf = false;
    return node;
}
bool BVHIntersect( 
    BVHNode* node,
    const ray& ray,
    hit_record& rec,
    float tMin = 0.001f,
    float tMax = std::numeric_limits<float>::max()
) {
    // 1. 检查光线是否与节点包围盒相交
    std::pair<float, bool> t = node->bounds.IntersectT(ray);
    auto tEnter = t.first;
    auto hit = t.second;
    if (!hit || tEnter > tMax)
        return false; // 快速剪枝

    // 2. 叶子节点：与所有物体精确求交
    if (node->isLeaf) {
        bool hitAny = false;
        for (const auto& obj : node->objects) {
            if (obj->hit(ray, interval(tMin,tMax), rec)) {
                tMax = rec.t; // 更新最近交点距离（关键剪枝）
                hitAny = true;
            }
        }
        return hitAny;
    }

    // 3. 内部节点：按子节点距离排序
    std::pair<float,bool> a= node->left->bounds.IntersectT(ray);
    auto leftT = a.first;
    auto leftHit = a.second;

    std::pair<float, bool> b = node->right->bounds.IntersectT(ray);
    auto rightT = b.first;
    auto rightHit = b.second;

    // 确保左右子树均有效
    if (!leftHit && !rightHit) return false;

    // 按距离排序：优先遍历更近的子树
    BVHNode* first = node->left;
    BVHNode* second = node->right;
    if (rightHit && (!leftHit || rightT < leftT)) {
        std::swap(first, second);
        std::swap(leftT, rightT);
    }

    // 4. 递归遍历子树（利用tMax剪枝远处子树）
    bool hitFirst = false;
    if (first) {
        hitFirst = BVHIntersect(first, ray, rec, tMin, tMax);
        if (hitFirst) tMax = rec.t; // 更新最近交点距离
    }

    // 若首个子树命中且距离足够近，跳过第二子树
    bool hitSecond = false;
    if (second && tMax > rightT) { // 检查第二子树是否可能更近
        hitSecond = BVHIntersect(second, ray, rec, tMin, tMax);
    }

    return hitFirst || hitSecond;
}





