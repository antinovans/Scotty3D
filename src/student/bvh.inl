
#include "../rays/bvh.h"
#include "debug.h"
#include <algorithm>
#include <iostream>
#include <stack>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);
    // std::cout << "primitives: " << primitives.size() << std::endl;
    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    // Replace these
    BBox box;
    for(const Primitive& prim : primitives) box.enclose(prim.bbox());

    recurseBuild(new_node(box, 0, primitives.size(), 0, 0), max_leaf_size);
    root_idx = 0;
}
template<typename Primitive>
void BVH<Primitive>::recurseBuild(size_t node_idx, size_t max_leaf_size) {
    // step 1: find the best line to partition the primitives
    if(nodes[node_idx].size <= max_leaf_size) {
        return;
    }
    BBox bestLeftBBox;
    BBox bestRightBBox;
    size_t bestLeftSize = 0;
    size_t bestRightSize = 0;
    // big box for partition
    BBox box = nodes[node_idx].bbox;
    int bestD;
    float bestSlice;
    float minCost = FLT_MAX;
    int bucketNum = 8;
    //recurse through three dimentions
    for(int d = 0; d < 3; d++) {
        float bucketLen = (box.max[d] - box.min[d]) / (float)bucketNum;
        float bottom = box.min[d];
        //recurse through number of buckets
        for(int i = 0; i < bucketNum; i++) {
            BBox leftBBox;
            BBox rightBBox;
            size_t leftSize = 0;
            size_t rightSize = 0;
            //checking every primitive one by one
            for(size_t j = nodes[node_idx].start; j < nodes[node_idx].start + nodes[node_idx].size;
                j++) {
                if(primitives[j].bbox().center()[d] <= bottom + bucketLen * i) {
                    leftSize++;
                    leftBBox.enclose(primitives[j].bbox());
                } else {
                    rightSize++;
                    rightBBox.enclose(primitives[j].bbox());
                }
            }
            //save the best solution
            if(leftBBox.surface_area() * leftSize + rightBBox.surface_area() * rightSize <=
               minCost) {
                bestLeftSize = leftSize;
                bestRightSize = rightSize;
                bestLeftBBox = leftBBox;
                bestRightBBox = rightBBox;
                bestD = d;
                bestSlice = bottom + bucketLen * i;
                minCost = leftBBox.surface_area() * leftSize + rightBBox.surface_area() * rightSize;
            }
        }
    }
    // step 2: rearrange the primitives vector according to the partition
    std::partition(primitives.begin() + nodes[node_idx].start,
                   primitives.begin() + nodes[node_idx].start + nodes[node_idx].size,
                   [bestD, bestSlice](Primitive& x) {
                       bool isLeft = x.bbox().center()[bestD] <= bestSlice;
                       return isLeft;
                   });
    // step 3: recurse and assign new node
    nodes[node_idx].l = new_node(bestLeftBBox, nodes[node_idx].start, bestLeftSize, 0, 0);
    nodes[node_idx].r =
        new_node(bestRightBBox, nodes[node_idx].start + bestLeftSize, bestRightSize, 0, 0);
    recurseBuild(nodes[node_idx].l, max_leaf_size);
    recurseBuild(nodes[node_idx].r, max_leaf_size);
}
template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.
    return find_closest_hit(ray, 0);
}
template<typename Primitive>
Trace BVH<Primitive>::find_closest_hit(const Ray& ray, size_t node) const {
    //check is the node is a leaf node
    if(nodes[node].is_leaf()) {
        Trace ret;
        for(size_t i = nodes[node].start; i < nodes[node].start + nodes[node].size; i++) {
            Trace hit = primitives[i].hit(ray);
            ret = Trace::min(ret, hit);
        }
        return ret;
    }
    else {
        //check whether the ray hit the big box
        Trace ret;
        Vec2 t;
        bool totalHit = nodes[node].bbox.hit(ray, t);
        if(!totalHit) {
            return ret;
        } else {
            Vec2 tl;
            Vec2 tr;
            bool leftHit = nodes[nodes[node].l].bbox.hit(ray, tl);
            bool rightHit = nodes[nodes[node].r].bbox.hit(ray, tr);
            if(leftHit) {
                if(rightHit) {
                    //special case: check if the ray hit the intersection of left part and right part
                    //both parts need to be recursed to get the shortest trace in the leaf node
                    size_t first = (tl.x <= tr.x) ? nodes[node].l : nodes[node].r;
                    size_t second = (tl.x <= tr.x) ? nodes[node].r : nodes[node].l;
                    float secondMin = (tl.x <= tr.x) ? tr.x : tl.x;
                    ret = find_closest_hit(ray, first);
                    if(secondMin < ray.dist_bounds.y) {
                        Trace hit = find_closest_hit(ray, second);
                        ret = Trace::min(ret, hit);
                    }
                    return ret;
                } else {
                    return find_closest_hit(ray, nodes[node].l);
                }
            } else if(rightHit)
                return find_closest_hit(ray, nodes[node].r);
            else
                return ret;
        }
    }
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {

    // A node is a leaf if l == r, since all interior nodes must have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c + lvl, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
