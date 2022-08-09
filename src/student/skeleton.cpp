
#include "../scene/skeleton.h"
#include <queue>

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3
    Vec3 start2point = point - start;
    Vec3 start2end = end - start;
    float scalarProjection = dot(start2point, start2end.unit());
    //not in the range of the current bone
    if(scalarProjection < 0)
    {
        return start;
    }
    if(scalarProjection > start2end.norm())
    {
        return end;
    }
    Vec3 projectionPoint = scalarProjection * start2end.unit() + start;
    // Return the closest point to 'point' on the line segment from start to end
    return projectionPoint;
}

Mat4 Joint::joint_to_bind() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space in bind position.
    Mat4 j2b = Mat4::I;
    // Bind position implies that all joints have pose = Vec3{0.0f}
    
    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    Joint* p = parent;
    while(p != nullptr)
    {
        j2b = Mat4::translate(p->extent) * j2b;
        p = p->parent;
    }
    // return Mat4::I;
    return j2b;
}

Mat4 Joint::joint_to_posed() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space, taking into account joint poses.
    Mat4 j2p = Mat4::euler(pose);
    Joint* p = parent;
    while(p != nullptr)
    {
        j2p = Mat4::euler(p->pose) * Mat4::translate(p->extent) * j2p;
        p = p->parent;
    }
    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    // return Mat4::I;
    return j2p;
}

Vec3 Skeleton::end_of(Joint* j) {

    // TODO(Animation): Task 2
    Vec3 v = j->joint_to_bind() * j->extent + base_pos;
    // Return the bind position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    return v;
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // TODO(Animation): Task 2
    Vec3 v = j->joint_to_posed() * j->extent + base_pos;
    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    // return Vec3{};
    return v;
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // TODO(Animation): Task 2
    Mat4 j2b = j->joint_to_bind();
    j2b = Mat4::translate(base_pos) * j2b;
    return j2b;
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // TODO(Animation): Task 2

    Mat4 j2p = j->joint_to_posed();
    j2p = Mat4::translate(base_pos) * j2p;
    return j2p;
}

void Skeleton::find_joints(const GL::Mesh& mesh, std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping: vertex index -> list of joints that should effect the vertex.
    // A joint should effect a vertex if it is within Joint::radius distance of the
    // bone's line segment in bind position.

    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();
    map.resize(verts.size());

    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.
    for_joints([&](Joint* j) {
        // What vertices does joint j effect?
        for(size_t i = 0; i < verts.size(); i++)
        {
            //translating vert's position from world space to joint space
            Vec3 vertJP = joint_to_bind(j).inverse() * verts[i].pos;
            // Vec3 vertJP = joint_to_posed(j) * Mat4::inverse(joint_to_bind(j)) * verts[i].pos;
            // float distance = cross(vertJP, j->extent).norm()/(j->extent).norm();
            // float distance = cross(verts[i].pos - joint_to_bind(j)*Vec3(0.0f), joint_to_bind(j)*j->extent - joint_to_bind(j)*Vec3(0.0f)).norm()
            // /(joint_to_bind(j)*j->extent- joint_to_bind(j)*Vec3(0.0f)).norm();
            // float distance = (closest_on_line_segment(joint_to_bind(j)* Vec3(0.0f), joint_to_bind(j) * j->extent, verts[i].pos)-joint_to_bind(j)* Vec3(0.0f)).norm();
            float distance = (closest_on_line_segment(Vec3(0.0f), j->extent, vertJP)-vertJP).norm();
            // float distance = (closest_on_line_segment(joint_to_posed(j)* Vec3(0.0f), joint_to_posed(j) * j->extent, vertJP)-joint_to_posed(j)* Vec3(0.0f)).norm();
            if(distance <= j->radius)
            {
                map[i].push_back(j);
            }
        }
    });
}

void Skeleton::skin(const GL::Mesh& input, GL::Mesh& output,
                    const std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Apply bone poses & weights to the vertices of the input (bind position) mesh
    // and store the result in the output mesh. See the task description for details.
    // map was computed by find_joints, hence gives a mapping from vertex index to
    // the list of bones the vertex should be effected by.

    // Currently, this just copies the input to the output without modification.

    std::vector<GL::Mesh::Vert> verts = input.verts();

    for(size_t i = 0; i < verts.size(); i++) {

        std::vector<float> weights;
        float totalWeight = 0;
        // Skin vertex i. Note that its position is given in object bind space.
        for(auto& j : map[i])
        {
            // Vec3 vertJP = joint_to_posed(j) * Mat4::inverse(joint_to_bind(j)) * verts[i].pos;
            Vec3 vertPW = joint_to_bind(j).inverse() * verts[i].pos;
            float weight = 1.0f/(closest_on_line_segment(Vec3(0.0f), (j->extent), vertPW) - vertPW).norm();
            // float weight = 1.0f/closest_on_line_segment(Vec3(0.0f), j->extent, Mat4::inverse(joint_to_bind(j)) *verts[i].pos).norm();
            weights.push_back(weight);
            totalWeight += weight;
        }
        Vec3 finalvertWP;
        Vec3 finalNorm;
        for(size_t k = 0; k < map[i].size(); k++)
        {
            finalvertWP +=  (weights[k]/totalWeight) * (joint_to_posed(map[i][k]) * joint_to_bind(map[i][k]).inverse() * verts[i].pos);
            finalNorm +=  (weights[k]/totalWeight) * (joint_to_posed(map[i][k]) * joint_to_bind(map[i][k]).inverse() * verts[i].norm);
        }
        verts[i].pos = finalvertWP;
        verts[i].norm = finalNorm;
    }

    std::vector<GL::Mesh::Index> idxs = input.indices();
    output.recreate(std::move(verts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // TODO(Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.
    Vec3 p = current - joint_to_posed() * Vec3(0,0,0);
    Vec3 gradient = Vec3(0,0,0);
    for(int i = 0; i < 3; i++)
    {
        Vec3 axis = Vec3(0,0,0);
        axis[i] = 1;
        Vec3 ri = joint_to_posed() * axis;
        Vec3 jacobiani = cross(ri,p);
        gradient[i] += dot(jacobiani,current - target);
    }
    angle_gradient = gradient;
    //recursively upward in the heirarchy
    if(parent != nullptr)
    {
        parent->compute_gradient(target, current);
        angle_gradient += parent->angle_gradient;
    }

    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // TODO(Animation): Task 2

    // Do several iterations of Jacobian Transpose gradient descent for IK
    float tau = 0.01f;
    float t = 0.0f;
    while(t < 1.0f)
    {
        //updating gradient
        for(auto handle: active_handles)
        {
            handle->joint->compute_gradient(handle->target, posed_end_of(handle->joint) - base_pos);
        }
        //update pose
        for(auto handle: active_handles)
        {
            Joint* iter = handle->joint;
            while(iter !=nullptr)
            {
                iter->pose -= tau* iter->angle_gradient;
                iter = iter->parent;
            }
        }
        t += tau;
    }
}
