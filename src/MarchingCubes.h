/**
 * @file    MarchingCubes.h
 * @author  Thomas Lewiner <thomas.lewiner@polytechnique.org>
 * @author  Math Dept, PUC-Rio
 * @version 0.2
 * @date    12/08/2002
 *
 * @brief   MarchingCubes Algorithm
 */
//________________________________________________

#ifndef _MARCHINGCUBES_H_
#define _MARCHINGCUBES_H_

#include <vector>
#include <unordered_map>

struct Vertex {
    glm::vec3 pos;
    glm::vec3 n;
};

struct Triangle {
    glm::ivec3 ids;
};

class MarchingCubes {
   public:
    enum Algorithm { OriginalMarchingCubes = 0, TopologicalMarchingCubes };

    MarchingCubes(const glm::ivec3 &size,
                  Algorithm algorithm = OriginalMarchingCubes);

    /** accesses the number of vertices of the generated mesh */
    inline const int nverts() const { return vertices_.size(); }
    /** accesses the number of triangles of the generated mesh */
    inline const int ntrigs() const { return triangles_.size(); }
    /** accesses a specific vertex of the generated mesh */

    inline const Vertex &vert(int i) const { return vertices_[i]; }
    inline const Triangle &trig(int i) const { return triangles_[i]; }
    /** accesses the vertex buffer of the generated mesh */
    inline Vertex *vertices() { return vertices_.data(); }
    /** accesses the triangle buffer of the generated mesh */
    inline Triangle *triangles() { return triangles_.data(); }
    glm::ivec3 size() const { return size_; }
    /**
     *
     *
     * \param originalMC true for the original Marching Cubes
     */

    // selects wether the algorithm will use the enhanced topologically
    // controlled lookup table or the original MarchingCubes
    inline void SetAlgorithm(Algorithm algorithm) { algorithm_ = algorithm; }
    void Setup();
    void run(float iso = 0.f);

    inline const float get_data(const glm::ivec3 &grid_coord) const
    {
        return data_[index(grid_coord)];
    }
    inline void set_data(const float val, const glm::ivec3 &grid_coord)
    {
        data_[index(grid_coord)] = val;
    }

   protected:
    /** tesselates one cube */
    void process_cube(const glm::ivec3 &grid_coord, uint8_t lut_entry,
                      float *cube);

    //-----------------------------------------------------------------------------
    // Operations
   protected:
    /**
     * computes almost all the vertices of the mesh by interpolation along the
     * cubes edges
     * \param iso isovalue
     */
    void compute_intersection_points(float iso);

    /**
     * routine to add a triangle to the mesh
     * \param trig the code for the triangle as a sequence of edges index
     * \param n    the number of triangles to produce
     * \param v12  the index of the interior vertex to use, if necessary
     */
    void add_triangle(const glm::ivec3 &grid_coord, const char *trig, char n,
                      int v12 = -1);

    int add_vertex(const glm::ivec3 &grid_coord, const glm::ivec3 &dir,
                   int corner, float *cube);
    /** adds a vertex inside the current cube */
    int add_c_vertex(const glm::ivec3 &grid_coord);

    // gradient of the implicit function at the lower vertex of the specified
    // cube
    float get_grad(const glm::ivec3 &grid_coord, int dim);

    int index(const int i, const int j, const int k) const
    {
        return i + size_.x * (j + size_.y * k);
    }

    int index(const glm::ivec3 &grid_coord) const
    {
        return grid_coord.x + size_.x * (grid_coord.y + size_.y * grid_coord.z);
    }

    // accesses the pre-computed vertex index on the lower horizontal edge of a
    // specific cube
    inline int get_x_vert(const glm::ivec3 &grid_coord) const
    {
        auto iter = x_verts_.find(index(grid_coord));
        return (iter == x_verts_.end()) ? -1 : iter->second;
    }

    // accesses the pre-computed vertex index on the lower longitudinal edge of
    // a specific cube
    inline int get_y_vert(const glm::ivec3 &grid_coord) const
    {
        auto iter = y_verts_.find(index(grid_coord));
        return (iter == y_verts_.end()) ? -1 : iter->second;
    }

    // accesses the pre-computed vertex index on the lower vertical edge of a
    // specific cube
    inline int get_z_vert(const glm::ivec3 &grid_coord) const
    {
        auto iter = z_verts_.find(index(grid_coord));
        return (iter == z_verts_.end()) ? -1 : iter->second;
    }

    // sets the pre-computed vertex index on the lower horizontal edge of a
    // specific cube
    inline void set_x_vert(const int val, const glm::ivec3 &grid_coord)
    {
        x_verts_[index(grid_coord)] = val;
    }

    // sets the pre-computed vertex index on the lower longitudinal edge of a
    // specific cube
    inline void set_y_vert(const int val, const glm::ivec3 &grid_coord)
    {
        y_verts_[index(grid_coord)] = val;
    }

    // sets the pre-computed vertex index on the lower vertical edge of a
    // specific cube
    inline void set_z_vert(const int val, const glm::ivec3 &grid_coord)
    {
        z_verts_[index(grid_coord)] = val;
    }

    //-----------------------------------------------------------------------------
    // Elements
   protected:
    Algorithm algorithm_;
    glm::ivec3 size_;
    std::vector<float> data_;

    std::unordered_map<int, int> x_verts_;
    std::unordered_map<int, int> y_verts_;
    std::unordered_map<int, int> z_verts_;

    std::vector<Vertex> vertices_;
    std::vector<Triangle> triangles_;
};
//_____________________________________________________________________________

#endif  // _MARCHINGCUBES_H_
