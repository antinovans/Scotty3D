
#include <queue>
#include <set>
#include <unordered_map>
#include <iostream>

#include "../geometry/halfedge.h"
#include "debug.h"

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/* 
    This method splits the given edge in half, but does not split the
    adjacent faces. Returns an iterator to the new vertex which splits
    the original edge.

    Example function for how to go about implementing local operations
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {

    // Phase 1: collect all elements
    HalfedgeRef h = (e->halfedge()->is_boundary()) ? e->halfedge()->twin() : e->halfedge();
    HalfedgeRef ht = h->twin();
    HalfedgeRef preh = h;
    HalfedgeRef nexht = ht->next();
    do {
        preh = preh->next();
    } while (preh->next() != h);
    Vec3 vpos = (h->vertex()->pos + ht->vertex()->pos)/2;

    // Phase 2: Allocate new elements
    VertexRef c = new_vertex();
    c->pos = vpos;
    HalfedgeRef hn = new_halfedge();
    HalfedgeRef hnt = new_halfedge();
    EdgeRef e0 = new_edge();

    // The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = new_face();
    HalfedgeRef h_not_used = new_halfedge();

    // Phase 3: Reassign elements
    e0->halfedge() = hn;
    hn->twin() = hnt;
    hn->edge() = e0;
    hn->vertex() = h->vertex();
    hn->face() = h->face();
    preh->next() = hn;
    hn->next() = h;
    h->vertex() = c;
    ht->next() = hnt;
    c->halfedge() = h;
    hn->vertex()->halfedge() = hn;
    c->is_new = true;

    // example of set_neighbors:
    // condenses hnt->next() = nexht; hnt->twin() = hn; hnt->vertex() = c; hnt->edge() = e0; hnt->face() = ht->face(); into one line
    hnt->set_neighbors(nexht, hn, c, e0, ht->face());

    // Phase 4: Delete unused elements
    erase(f_not_used);
    erase(h_not_used);

    // Phase 5: Return the correct iterator
    return c;
}

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {
    bool onBoundry= e->on_boundary();
    //get the involving vertices
    HalfedgeRef h = (e->halfedge()->is_boundary()) ? e->halfedge()->twin() : e->halfedge();
    HalfedgeRef ht  = e->halfedge()->twin();
    if(h->face()->degree() == 3 && (h->next()->edge()->on_boundary() || h->next()->next()->edge()->on_boundary()))
    {
        return std::nullopt;
    }
    if(ht->face()->degree() == 3 && (ht->next()->edge()->on_boundary() || ht->next()->next()->edge()->on_boundary()))
    {
        return std::nullopt;
    }
    VertexRef v0 = h->vertex();
    VertexRef v1 = ht->vertex();
    //instantiate the midpoint
    Vec3 vpos = (v0->pos + v1->pos)/2;
    VertexRef c = new_vertex();
    c->pos = vpos;
    if(onBoundry)
    {
        HalfedgeRef hn = h->next();
        h->face()->halfedge() = hn;
        HalfedgeRef hi = hn;
        HalfedgeRef preh = h;
        c->halfedge() = hn;
        while(preh->next() != h)
        {
            preh = preh->next();
        }
        //getting all the halfedges connected to v1
        while(!hi->twin()->next()->is_boundary())
        {
            hi->vertex() = c;
            hi = hi->twin()->next();
        }
        hi->vertex() = c;
        //getting all the halfedges connected to v0
        HalfedgeRef hj = preh->twin();
        while(!hj->is_boundary())
        {
            hj->vertex() = c;
            HalfedgeRef prehj = hj;
            while(prehj->next() != hj)
            {
                prehj = prehj->next();
            }
            hj = prehj->twin();
        }
        hj->vertex() = c;
        hi->twin()->next() = hj;
        preh->next() = hn;
        erase(v0);
        erase(v1);
        erase(e);
        erase(h);
        erase(ht);
        //special case: when the edge is inside a triangle, two edges need to be reduced to one
        if(hn->next() == preh)
        {
            HalfedgeRef pht = preh->twin();
            HalfedgeRef nht = hn->twin();
            pht->twin() = nht;
            nht->twin() = pht;
            nht->edge() = pht->edge();
            preh->edge()->halfedge() = nht;
            c->halfedge() = pht;
            erase(hn->face());
            erase(hn->edge());
            erase(preh);
            erase(hn);
        }
        // std::cout << "after hi twin" << hi->twin()->id()<<std::endl;
        return c;
    }
    if(!onBoundry)
    {
        HalfedgeRef hn = h->next();
        HalfedgeRef htn = ht->next();
        h->face()->halfedge() = hn;
        ht->face()->halfedge() = htn;
        HalfedgeRef hi = hn;
        HalfedgeRef hj = htn;
        HalfedgeRef preh = h;
        HalfedgeRef preht = ht;
        c->halfedge() = hn;
        while(preh->next() != h)
        {
            preh = preh->next();
        }
        while(preht->next() != ht)
        {
            preht = preht->next();
        }
        //getting all the halfedges connected to v1
        while(hi->twin()->next() != hn)
        {
            hi->vertex() = c;
            hi = hi->twin()->next();
        }
        hi->vertex() = c;
        //getting all the halfedges connected to v0
        while(hj->twin()->next() != htn)
        {
            hj->vertex() = c;
            hj = hj->twin()->next();
        }
        hj->vertex() = c;
        //reset next halfedges for the interfaces
        preh->next() = hn;
        preht->next() = htn;
        erase(v0);
        erase(v1);
        erase(e);
        erase(h);
        erase(ht);
        //special case: when the edge is inside a triangle, two edges need to be reduced to one
        if(hn->next() == preh)
        {
            HalfedgeRef pht = preh->twin();
            HalfedgeRef nht = hn->twin();
            pht->twin() = nht;
            nht->twin() = pht;
            nht->edge() = pht->edge();
            preh->edge()->halfedge() = nht;
            preh->vertex()->halfedge() = nht;
            c->halfedge() = pht;
            erase(hn->face());
            erase(hn->edge());
            erase(preh);
            erase(hn);
        }
        if(htn->next() == preht)
        {
            HalfedgeRef pht = preht->twin();
            HalfedgeRef nht = htn->twin();
            pht->twin() = nht;
            nht->twin() = pht;
            nht->edge() = pht->edge();
            preht->edge()->halfedge() = nht;
            preht->vertex()->halfedge() = nht;
            c->halfedge() = pht;
            erase(htn->face());
            erase(htn->edge());
            erase(preht);
            erase(htn);
        }
        return c;
    }
    return std::nullopt;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge counter-clockwise and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {
    if(e->on_boundary()){
        return std::nullopt;
    }
    // HALFEDGES
    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->next();
    HalfedgeRef h2 = h1->next();
    HalfedgeRef h3 = h0->twin();
    HalfedgeRef h4 = h3->next();
    HalfedgeRef h5 = h4->next();
    // HalfedgeRef h6 = h1->twin();
    // HalfedgeRef h7 = h2->twin();
    // HalfedgeRef h8 = h4->twin();
    // HalfedgeRef h9 = h5->twin();
    HalfedgeRef hl = h1;
    HalfedgeRef hr = h4;

    // VERTICES
    VertexRef v0 = h0->vertex();
    VertexRef v1 = h3->vertex();
    VertexRef v2 = h5->vertex();
    VertexRef v3 = h2->vertex();

    // FACES
    FaceRef f0 = h0->face();
    FaceRef f1 = h3->face();
    int d0 = f0->degree();
    int d1 = f1->degree();
    if(d0 >= 3){
        while(hl->next() != h0){
            hl = hl->next();
        }
    }
    if(d1 >= 3){
        while(hr->next() != h3){
            hr = hr->next();
        }
    }

    //REASSIGN
    h0->next() = h2;
    h0->twin() = h3;
    h0->vertex() = v2;
    h0->edge() = e;
    h0->face() = f0;
    h1->next() = h3;
    h1->face() = f1;
    if(d0 <= 3){
        h2->next() = h4;
    }
    else{
        hl->next() = h4;
    }
    h3->next() = h5;
    h3->vertex() = v3;
    h3->edge() = e;
    h4->next() = h0;
    h4->face() = f0;
    if(d1 <= 3)
    {
        h5->next() = h1;
    }
    else{
        hr->next() = h1;
    }
    // VERTICES
    v0->halfedge() = h4;
    v1->halfedge() = h1;
    v2->halfedge() = h5;
    v3->halfedge() = h2;
    // FACES
    f0->halfedge() = h0;
    f1->halfedge() = h1;
    return e;

}
/*
    This method is used to connect two vertices
          *
        * *
      *   *
    v0    v1
      *   *
        * *
          * 
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::connect_vertices(Halfedge_Mesh::HalfedgeRef h0, Halfedge_Mesh::HalfedgeRef h1) {
    HalfedgeRef h0x = h0;
    HalfedgeRef h1x = h1;
    while (h0x->next() != h1)
    {
        h0x = h0x->next();
    } 
    while (h1x->next() != h0)
    {
        h1x = h1x->next();
    } 
    VertexRef v0 = h0->vertex();
    VertexRef v1 = h1->vertex();
    HalfedgeRef hc = new_halfedge();
    HalfedgeRef hct = new_halfedge();
    EdgeRef ec = new_edge();
    FaceRef of = h0->face();
    FaceRef nf = new_face();
    of->halfedge() = h0;
    ec->halfedge() = hc;
    h0x->next() = hc;
    h1x->next() = hct;

    nf->halfedge() = hct;
    hct->face() = nf;
    hc->set_neighbors(h0, hct, v1, ec, h0->face());
    hct->set_neighbors(h1, hc, v0, ec, nf);
    h1->face() = nf;
    h1x->face() = nf;
    ec->is_new = true;
    return ec;
}
/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    HalfedgeRef h = e->halfedge();
    HalfedgeRef ht = h->twin();
    int nh = h->face()->degree();
    int nht = ht->face()->degree();
    //discard the case when split_edge is invalid
    if(h->face()->degree() != 3 && ht->face()->degree() != 3){
        return std::nullopt;
    }
    VertexRef v1 = *bisect_edge(e);
    if(nh == 3){
        HalfedgeRef h1 = h->next()->next();
        EdgeRef ec = *connect_vertices(h1,h);
        ec->is_new = true;
    }
    if(nht == 3){
        HalfedgeRef h1 = ht->next()->next()->next();
        EdgeRef ec = *connect_vertices(h1,ht->next());
        ec->is_new = true;
    }
    return v1;
}


/*
    This method should insets a vertex into the given face, returning a pointer to the new center vertex
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
    (void)f;
    return std::nullopt;
}

/*
    This method should inset a face into the given face, returning a pointer to the new face.
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::inset_face(Halfedge_Mesh::FaceRef f) {

    // hint: use bevel_face positions as a helper function here
    (void)f;
    return std::nullopt;
}

/*
    This method should bevel a vertex and inserts a vertex into the new vertex, returning a pointer to that vertex
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::extrude_vertex(VertexRef v) {
    (void)v;
    return std::nullopt;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for bevel_vertex, it has one element, the original vertex position,
    for bevel_edge, two for the two vertices, and for bevel_face, it has the original
    position of each vertex in order starting from face->halfedge. You should use these 
    positions, as well as the normal and tangent offset fields to assign positions to 
    the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    only responsible for updating the *connectivity* of the mesh---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."
    std::vector<HalfedgeRef> original_halfedges;
    HalfedgeRef h = f->halfedge();
    do {
        original_halfedges.push_back(h);
        h = h->next();
    } while(h != f->halfedge());
    for(auto hi : original_halfedges)
    {
        VertexRef vn = hi->next()->vertex();
        //instantiating new halfedges
        HalfedgeRef hn = new_halfedge();
        HalfedgeRef hnn = new_halfedge();
        HalfedgeRef hnnt = new_halfedge();
        HalfedgeRef preh = new_halfedge();
        //instantiating new vertices
        VertexRef vnn = new_vertex();
        //testing positioning
        Vec3 vpos = vn->pos;
        // vpos += 0.25*vpos;
        vnn->pos = vpos;
        //instantiating new edges
        EdgeRef en = new_edge();
        EdgeRef enn = new_edge();
        //instantiating new faces
        FaceRef nf = new_face();
        //assigning halfedges next
        hi->next() = hn;
        hn->next() = hnn;
        hnn->next() = preh;
        preh->next() = hi;
        //assigning halfedges twin
        hnn->twin() = hnnt;
        hnnt->twin() = hnn;
        //assigning vertices
        hn->vertex() = vn;
        hnn->vertex() = vnn;
        vnn->halfedge() = hnn;
        //assigning edges
        hn->edge() = en;
        en->halfedge() = hn;
        hnn->edge() = enn;
        hnnt->edge() = enn;
        enn->halfedge() = hnnt;
        //assigning faces
        hnnt->face() = f;
        f->halfedge() = hnnt;
        hi->face() = nf;
        nf->halfedge() = hi;
        hn->face() = nf;
        hnn->face() = nf;
        preh->face() = nf;
        en->is_new = true;
        enn->is_new = true;
    }
    for(int i = 0; i < static_cast<int>(original_halfedges.size()); i++)
    {
        //assigning twin, vertex, edge for preh and hn
        // auto h = original_halfedges[i];
        auto preh = original_halfedges[i]->next()->next()->next();
        auto preIterator = (i - 1 < 0) ? original_halfedges.size() - 1 : i - 1;
        auto oh = original_halfedges[preIterator];
        auto ohn = oh->next();
        auto ohnn = ohn->next();
        preh->twin() = ohn;
        ohn->twin() = preh;
        preh->vertex() = ohnn->vertex();
        preh->edge() = ohn->edge();
        //assigning next, vertex for hnnt
        auto hnnt = original_halfedges[i]->next()->next()->twin();
        int nextIterator = (i + 1 == static_cast<int>(original_halfedges.size())) ? 0 : i + 1;
        auto nh = original_halfedges[nextIterator];
        auto nhnnt = nh->next()->next()->twin();
        hnnt->next() = nhnnt;
        hnnt->vertex() = ohnn->vertex();
    }

    return f;

    // (void)f;
    // return std::nullopt;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset, float last_tangent_offset, float last_normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    Vec3 faceNormal = face->normal();
    Vec3 faceCenter = face->center();
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());
    // std::cout<<"face normal: "<< face->normal().x<< ","<<face->normal().y<< ","<<face->normal().z << "," <<std::endl;
    auto n = new_halfedges.size();
    float t_offset = tangent_offset - last_tangent_offset;
    float n_offset = normal_offset - last_normal_offset;
	for (size_t i = 0; i < n; i++) {
		Vec3 horizontal = start_positions[i] - faceCenter;
        new_halfedges[i]->vertex()->pos += (t_offset * horizontal - n_offset * faceNormal);
	}

}

/*
    Updates the position of v using the given start_position
*/
void Halfedge_Mesh::extrude_vertex_position(const Vec3& start_positions, Halfedge_Mesh::FaceRef face) {
    (void)start_positions;
    (void)face;
}

/******************************************************************
*********************** Global Operations *************************
******************************************************************/

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    for(FaceRef f = faces_begin(); f != faces_end(); f++)
    {
        if(f->degree() == 3)
        {
            continue;
        }
        if(f->is_boundary())
        {
            continue;
        }
        std::queue<HalfedgeRef> newfaceh;
        int degree = f->degree();
        HalfedgeRef hIterator = f->halfedge()->next()->next();
        HalfedgeRef hpreIterator = f->halfedge()->next();
        HalfedgeRef hprepreIterator = f->halfedge();
        int distance = 2;
        while(distance < degree - 1)
        {
            newfaceh.push(hIterator);
            //instantiating new edge and halfedges
            HalfedgeRef h = new_halfedge();
            HalfedgeRef ht = new_halfedge();
            EdgeRef e = new_edge();
            h->twin() = ht;
            ht->twin() = h;
            h->vertex() = hIterator->vertex();
            ht->vertex() = f->halfedge()->vertex();
            e->halfedge() = ht;
            ht->edge() = e;
            h->edge() = e;

            //assigning next for halfedges
            h->next() = hprepreIterator;
            ht->next() = hIterator;
            hpreIterator->next() = h;
            h->face() = hprepreIterator->face();
            if(distance == degree - 2)
            {
                hIterator->next()->next() = ht;
            }
            hprepreIterator = h;
            hpreIterator = hIterator;
            hIterator = hIterator->next();
            distance++;
        }
        while(newfaceh.size() > 0)
        {
            HalfedgeRef h = newfaceh.front();
            newfaceh.pop();
            HalfedgeRef hIter = h;
            FaceRef nf = new_face();
            nf->halfedge() = h;
            do
            {
                hIter->face() = nf;
                hIter = hIter->next();
            } while (hIter->next() != h);
            hIter->face() = nf;
        }
        // std::cout<< f->degree() <<std::endl;
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided) mesh. They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {
    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for(auto v = vertices_begin(); v != vertices_end(); v++)
    {
        v->new_pos = v->pos;
    }
    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for(auto e = edges_begin(); e != edges_end(); e++)
    {
        e->new_pos = e->center();
    }
    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for(auto f = faces_begin(); f != faces_end(); f++)
    {
        f->new_pos = f->center();
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos. The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)
    // Faces
    //new
    for(auto f = faces_begin(); f != faces_end(); f++)
    {
        f->new_pos = f->center();
    }
    for(auto e = edges_begin(); e != edges_end(); e++)
    {
        e->new_pos = 0.25f * (e->halfedge()->face()->new_pos
				+ e->halfedge()->twin()->face()->new_pos
				+ e->halfedge()->vertex()->pos
				+ e->halfedge()->twin()->vertex()->pos);
    }
    for(auto v = vertices_begin(); v != vertices_end(); v++)
    {
        Vec3 Q = Vec3(0,0,0);
        Vec3 R = Vec3(0,0,0);
        std::vector<FaceRef> neighborFs;
        std::vector<EdgeRef> neighborEs;
        //finding all the faces that has v
        HalfedgeRef hI = v->halfedge();
        do
        {
            neighborFs.push_back(hI->face());
            neighborEs.push_back(hI->edge());
            hI = hI->twin()->next();
        }while(hI != v->halfedge());
        //Calculating Q
        for(auto neighborF : neighborFs)
            Q += neighborF->new_pos;
        Q *= 1.0f/neighborFs.size();
        //Calculating R
        for(auto neighborE : neighborEs)
            R += neighborE->center();
        R *= 1.0f/neighborEs.size();

        Vec3 S = v->pos;
        float n = (float)v->degree();
        v->new_pos = (Q + 2 * R + (n - 3) * S) / n;
    }
}

/*
    This routine should increase the number of triangles in the mesh
    using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

        // Each vertex and edge of the original mesh can be associated with a
    // vertex in the new (subdivided) mesh.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh. Navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.
    
    // Compute new positions for all the vertices in the input mesh using
    // the Loop subdivision rule and store them in Vertex::new_pos.
    //    At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    for(auto v = vertices_begin(); v != vertices_end(); v++)
    {
        std::vector<VertexRef> neighborVs;
        HalfedgeRef h = v->halfedge();
        do {
            neighborVs.push_back(h->next()->vertex());
            h = h->twin()->next();
        } while(h != v->halfedge());
        size_t n = neighborVs.size();
        float u = n == 3 ? 3.0f / 16.0f : 3.0f / (8.0f * n);
        v->new_pos = (1 - n * u) * v->pos;
        Vec3 sum = Vec3(0,0,0);
        for(auto neighborV : neighborVs)
            sum += neighborV->pos;
        v->new_pos += u*sum;
        v->is_new = false;
    }
    // // Next, compute the subdivided vertex positions associated with edges, and
    // // store them in Edge::new_pos.
    for(auto e = edges_begin(); e != edges_end(); e++)
    {
        Vec3 A = e->halfedge()->vertex()->pos;
        Vec3 B = e->halfedge()->twin()->vertex()->pos;
        Vec3 C = e->halfedge()->next()->next()->vertex()->pos;
        Vec3 D = e->halfedge()->twin()->next()->next()->vertex()->pos;
        e->new_pos = (3.0f * (A + B) + C + D) / 8.0f;
        e->is_new = false;
    }

    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting 
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new. 
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)
    size_t originalEs = edges.size();
    EdgeRef e = edges.begin();
    for(size_t i = 0; i < originalEs; i++)
    {
        // get the next edge NOW!
        EdgeRef nextEdge = e;
        nextEdge++;
        //reassigning position for bisect vertex
        VertexRef v = *split_edge(e);
        v->new_pos = e->new_pos;
        v->is_new = true;

        e = nextEdge;
    }
    // Now flip any new edge that connects an old and new vertex.
    for(auto edge = edges_begin(); edge != edges_end(); edge++)
    {
        if(edge->is_new == false)
            continue;
        if(edge->halfedge()->vertex()->is_new + edge->halfedge()->twin()->vertex()->is_new != 1)
            continue;
        flip_edge(edge);
    }
    // Finally, copy new vertex positions into the Vertex::pos.
    for(auto vertex = vertices_begin(); vertex != vertices_end(); vertex++)
    {
        vertex->pos = vertex->new_pos;
    }
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate is called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
