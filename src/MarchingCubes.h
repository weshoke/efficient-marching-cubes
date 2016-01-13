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
#include "glm/glm.hpp"

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

    inline const size_t nverts() const { return vertices_.size(); }
    inline const size_t ntrigs() const { return triangles_.size(); }
    inline const Vertex &vert(int i) const { return vertices_[i]; }
    inline const Triangle &trig(int i) const { return triangles_[i]; }
    inline Vertex *vertices() { return vertices_.data(); }
    inline Triangle *triangles() { return triangles_.data(); }
    glm::ivec3 size() const { return size_; }
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
    inline void set_data(const float val, int i, int j, int k)
    {
        data_[index(glm::ivec3(i, j, k))] = val;
    }

   protected:
    /** tesselates one cube */
    void ProcessCube(const glm::ivec3 &grid_coord, uint8_t lut_entry,
                     float *cube);

    void ProcessCubeTiling13(const glm::ivec3 &grid_coord, uint8_t config_case,
                             uint8_t config, float *cube);

    //-----------------------------------------------------------------------------
    // Operations
    // protected:
   public:
    // computes almost all the vertices of the mesh by interpolation along the
    // cubes edges
    void compute_intersection_points(float iso);

    /**
     * routine to add a triangle to the mesh
     * \param trig the code for the triangle as a sequence of edges index
     * \param n    the number of triangles to produce
     * \param v12  the index of the interior vertex to use, if necessary
     */
    void add_triangle(const glm::ivec3 &grid_coord, const char *trig, char n,
                      int v12 = -1);

    size_t add_vertex(const glm::ivec3 &grid_coord, const glm::ivec3 &dir,
                      int corner, float *cube);
    /** adds a vertex inside the current cube */
    size_t add_c_vertex(const glm::ivec3 &grid_coord);

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
        return (iter == x_verts_.end()) ? -1 : int(iter->second);
    }

    // accesses the pre-computed vertex index on the lower longitudinal edge of
    // a specific cube
    inline int get_y_vert(const glm::ivec3 &grid_coord) const
    {
        auto iter = y_verts_.find(index(grid_coord));
        return (iter == y_verts_.end()) ? -1 : int(iter->second);
    }

    // accesses the pre-computed vertex index on the lower vertical edge of a
    // specific cube
    inline int get_z_vert(const glm::ivec3 &grid_coord) const
    {
        auto iter = z_verts_.find(index(grid_coord));
        return (iter == z_verts_.end()) ? -1 : int(iter->second);
    }

    // sets the pre-computed vertex index on the lower horizontal edge of a
    // specific cube
    inline void set_x_vert(const size_t val, const glm::ivec3 &grid_coord)
    {
        x_verts_[index(grid_coord)] = val;
    }

    // sets the pre-computed vertex index on the lower longitudinal edge of a
    // specific cube
    inline void set_y_vert(const size_t val, const glm::ivec3 &grid_coord)
    {
        y_verts_[index(grid_coord)] = val;
    }

    // sets the pre-computed vertex index on the lower vertical edge of a
    // specific cube
    inline void set_z_vert(const size_t val, const glm::ivec3 &grid_coord)
    {
        z_verts_[index(grid_coord)] = val;
    }

    const std::unordered_map<int, size_t> x_verts() const { return x_verts_; }
    const std::unordered_map<int, size_t> y_verts() const { return y_verts_; }
    const std::unordered_map<int, size_t> z_verts() const { return z_verts_; }
    //-----------------------------------------------------------------------------
    // Elements
   protected:
    Algorithm algorithm_;
    glm::ivec3 size_;
    std::vector<float> data_;

    std::unordered_map<int, size_t> x_verts_;
    std::unordered_map<int, size_t> y_verts_;
    std::unordered_map<int, size_t> z_verts_;

    std::vector<Vertex> vertices_;
    std::vector<Triangle> triangles_;
};
//_____________________________________________________________________________

#endif  // _MARCHINGCUBES_H_
