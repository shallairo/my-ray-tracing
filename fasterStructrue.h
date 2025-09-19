#pragma once
#include "hittable.h" // ��������ǰ��
#include <vector>
#include <algorithm>
class hittable;
// Bounds3.hpp
struct Bounds3 {
    vec3 pMin, pMax; // ��С/��󶥵�����

    Bounds3():pMin(point3(FLT_MAX, FLT_MAX, FLT_MAX)), pMax(point3(-FLT_MAX, -FLT_MAX, -FLT_MAX)) {}
    Bounds3(const vec3&u,const vec3&v):pMin(u),pMax(v){}
    // �����Χ�б������SAH���ģ�
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
    // ���ع������Χ�е��ཻ���루tMin�����Ƿ��ཻ
    std::pair<float, bool> IntersectT(const ray& ray) const {
        float tMin = -std::numeric_limits<float>::max();           // ���߽����Χ�е�ʱ��
        float tMax = std::numeric_limits<float>::max(); // �����뿪��Χ�е�ʱ��

        for (int i = 0; i < 3; ++i) {
            float invD = 1.0f / ray.direction()[i];
            float tEnter = (pMin[i] - ray.origin()[i]) * invD;
            float tExit = (pMax[i] - ray.origin()[i]) * invD;

            if (invD < 0) std::swap(tEnter, tExit); // ���������
            tMin = std::max(tMin, tEnter);        // ����������ʱ��
            tMax = std::min(tMax, tExit);        // ������С�뿪ʱ��
            if (tMax < tMin) return { 0.0f, false }; // ���ཻ
        }
        return { tMin, tMax>0 }; // ���ؽ��������ཻ��־
    }
};

// BVH�ڵ�ṹ
struct BVHNode {
    Bounds3 bounds;
    BVHNode* left = nullptr;
    BVHNode* right = nullptr;
    bool isLeaf = false;
    std::vector<std::shared_ptr<hittable>> objects; // Ҷ�ӽڵ�洢����
// ������������ֹ�ڴ�й©��
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
// ���������б�İ�Χ��
Bounds3 ComputeBbox(const std::vector<std::shared_ptr<hittable>>& objects, int start, int end) 
{
    Bounds3 bbox = objects[start]->bounding_box();
    for (int i = start + 1; i < end; i++) {
        bbox = uni(bbox, objects[i]->bounding_box());
    }
    return bbox;
}

// SAH�ָѡ����С�ɱ��ķָ��
int FindBestSplitWithSAH(
    const std::vector<shared_ptr<hittable>>& objects,
    int start, int end, int axis,
    float& minCost, float axisLen
) {
    const int BIN_COUNT = 32; // �ռ�Ͱ����
    struct Bin {
        Bounds3 bbox;
        int count = 0;
    } bins[BIN_COUNT];
    // ��ʼ��Ͱ

    int bestSplit = start + (end - start) / 2; // Ĭ��������������λ���ָ�
    float binWidth = axisLen / BIN_COUNT;
	if (binWidth <= 0.1f) return bestSplit; // ��ֹ����0
    if (axisLen == 0) return start + (end - start) / 2; // ��ֹ��ѭ��
    // ��������䵽Ͱ��
    for (int i = start; i < end; i++) {
        float center = objects[i]->getCenter()[axis];
        int binIdx = std::max(0, std::min(BIN_COUNT - 1, (int)((center - objects[start]->bounding_box().pMin[axis]) / binWidth)));
        bins[binIdx].bbox = uni(bins[binIdx].bbox, objects[i]->bounding_box());
        bins[binIdx].count++;
    }

    // ����ÿ���ָ���SAH�ɱ�
    minCost = std::numeric_limits<float>::max();

    for (int split = 1; split < BIN_COUNT; split++) {
        Bounds3 leftBox;
        Bounds3 rightBox;
        int leftCount = 0, rightCount = 0;

        // ��������Χ��
        for (int i = 0; i < split; i++) {
            if (bins[i].count > 0) { // ֻ�ϲ�������� bin
                leftBox = uni(leftBox, bins[i].bbox);
                leftCount += bins[i].count;
            }
        }

        // ��������Χ��
        for (int i = split; i < BIN_COUNT; i++) {
            if (bins[i].count > 0) {
                rightBox = uni(rightBox, bins[i].bbox);
                rightCount += bins[i].count;
            }
        }

        // SAH�ɱ� = �������ɱ� + �������ɱ�
        float cost = leftBox.SurfaceArea() * leftCount + rightBox.SurfaceArea() * rightCount;
        if (cost < minCost) {
            minCost = cost;
            bestSplit =  split; // ӳ�����С�ɱ��İ�Χ������
        }
    }
	//��Ͱ����ת��Ϊ��������
    int splitIndex = start;
    for (int i = start; i < end; i++) {
        float center = objects[i]->getCenter()[axis];
        int binIdx = std::min(BIN_COUNT - 1,
            static_cast<int>((center - objects[start]->bounding_box().pMin[axis]) / binWidth));
        if (binIdx >= bestSplit) { // �˴���bestSplit��Ͱ������
            splitIndex = i;
            break;
        }
    }
    return splitIndex;
}

// �ݹ鹹��BVH��SAH�Ż����ģ�
BVHNode* BuildBVH(
    std::vector<std::shared_ptr<hittable>>& objects,
    int start, int end,
    int maxLeafSize 
) {
    if (start >= end) return nullptr;

    // 1. ���㵱ǰ�ڵ��Χ��
    Bounds3 bbox = ComputeBbox(objects, start, end);

    // 2. Ҷ�ӽڵ���ֹ����
    if (end - start <= maxLeafSize) {
        auto node = new BVHNode;
        node->bounds = bbox;
        node->isLeaf = true;
        node->objects.assign(objects.begin() + start, objects.begin() + end);
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }

    // 3. ѡ�����
    int axis = bbox.maxExtent();
	float minCost = std::numeric_limits<float>::max();
	float axisLen = bbox.pMax[axis] - bbox.pMin[axis];
    // 4. �ָ�㣬ȷ��������ѭ��
	//int bestSplit = FindBestSplitWithSAH(objects, start, end, axis, minCost, axisLen);
    int bestSplit = start + (end - start) / 2;
    if (bestSplit == start || bestSplit == end) {
        bestSplit = start + 1;
    }

    auto comparator = [axis](const auto& a, const auto& b) {
        return a->getCenter()[axis] < b->getCenter()[axis];
    };
    std::sort(objects.begin() + start, objects.begin() + end, comparator);

    // 5. �ݹ鹹������
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
    // 1. �������Ƿ���ڵ��Χ���ཻ
    std::pair<float, bool> t = node->bounds.IntersectT(ray);
    auto tEnter = t.first;
    auto hit = t.second;
    if (!hit || tEnter > tMax)
        return false; // ���ټ�֦

    // 2. Ҷ�ӽڵ㣺���������徫ȷ��
    if (node->isLeaf) {
        bool hitAny = false;
        for (const auto& obj : node->objects) {
            if (obj->hit(ray, interval(tMin,tMax), rec)) {
                tMax = rec.t; // �������������루�ؼ���֦��
                hitAny = true;
            }
        }
        return hitAny;
    }

    // 3. �ڲ��ڵ㣺���ӽڵ��������
    std::pair<float,bool> a= node->left->bounds.IntersectT(ray);
    auto leftT = a.first;
    auto leftHit = a.second;

    std::pair<float, bool> b = node->right->bounds.IntersectT(ray);
    auto rightT = b.first;
    auto rightHit = b.second;

    // ȷ��������������Ч
    if (!leftHit && !rightHit) return false;

    // �������������ȱ�������������
    BVHNode* first = node->left;
    BVHNode* second = node->right;
    if (rightHit && (!leftHit || rightT < leftT)) {
        std::swap(first, second);
        std::swap(leftT, rightT);
    }

    // 4. �ݹ��������������tMax��֦Զ��������
    bool hitFirst = false;
    if (first) {
        hitFirst = BVHIntersect(first, ray, rec, tMin, tMax);
        if (hitFirst) tMax = rec.t; // ��������������
    }

    // ���׸����������Ҿ����㹻���������ڶ�����
    bool hitSecond = false;
    if (second && tMax > rightT) { // ���ڶ������Ƿ���ܸ���
        hitSecond = BVHIntersect(second, ray, rec, tMin, tMax);
    }

    return hitFirst || hitSecond;
}





