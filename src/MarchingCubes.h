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
    /**
*Main and default constructor
*\brief constructor
*\param size_x width  of the grid
*\param size_y depth  of the grid
*\param size_z height of the grid
*/
    MarchingCubes(const int size_x = -1, const int size_y = -1,
                  const int size_z = -1);

    //-----------------------------------------------------------------------------
    // Accessors
   public:
    /** accesses the number of vertices of the generated mesh */
    inline const int nverts() const { return _vertices.size(); }
    /** accesses the number of triangles of the generated mesh */
    inline const int ntrigs() const { return _triangles.size(); }
    /** accesses a specific vertex of the generated mesh */

    inline const Vertex &vert(int i) const { return _vertices[i]; }
    inline const Triangle &trig(int i) const { return _triangles[i]; }
    /*
    inline const Vertex *vert(const int i) const
{
    if (i < 0 || i >= nverts()) return (Vertex *)NULL;
    return _vertices.data() + i;
}
inline const Triangle *trig(const int i) const
{
    if (i < 0 || i >= ntrigs()) return (Triangle *)NULL;
    return _triangles.data() + i;
}
    */

    /** accesses the vertex buffer of the generated mesh */
    inline Vertex *vertices() { return _vertices.data(); }
    /** accesses the triangle buffer of the generated mesh */
    inline Triangle *triangles() { return _triangles.data(); }
    /**  accesses the width  of the grid */
    inline const int size_x() const { return _size.x; }
    /**  accesses the depth  of the grid */
    inline const int size_y() const { return _size.y; }
    /**  accesses the height of the grid */
    inline const int size_z() const { return _size.z; }
    /**
     * changes the size of the grid
     * \param size_x width  of the grid
     * \param size_y depth  of the grid
     * \param size_z height of the grid
     */
    inline void set_resolution(const glm::ivec3 &size) { _size = size; }
    /**
     * selects wether the algorithm will use the enhanced topologically
     * controlled lookup table or the original MarchingCubes
     * \param originalMC true for the original Marching Cubes
     */
    inline void set_method(const bool originalMC = false)
    {
        _originalMC = originalMC;
    }

    // Data access
    /**
     * accesses a specific cube of the grid
     * \param i abscisse of the cube
     * \param j ordinate of the cube
     * \param k height of the cube
     */
    inline const float get_data(const int i, const int j, const int k) const
    {
        return _data[index(i, j, k)];
    }
    /**
     * sets a specific cube of the grid
     * \param val new value for the cube
     * \param i abscisse of the cube
     * \param j ordinate of the cube
     * \param k height of the cube
     */
    inline void set_data(const float val, const int i, const int j, const int k)
    {
        _data[index(i, j, k)] = val;
    }

    // Data initialization
    /** inits temporary structures (must set sizes before call) : the grid and
     * the vertex index per cube */
    void init_temps();
    /** inits all structures (must set sizes before call) : the temporary
     * structures and the mesh buffers */
    void init_all();

    //-----------------------------------------------------------------------------
    // Algorithm
   public:
    /**
     * Main algorithm : must be called after init_all
     * \param iso isovalue
     */
    void run(float iso = 0.f);

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
        return i + _size.x * (j + _size.y * k);
    }

    int index(const glm::ivec3 &grid_coord) const
    {
        return grid_coord.x + _size.x * (grid_coord.y + _size.y * grid_coord.z);
    }

    // accesses the pre-computed vertex index on the lower horizontal edge of a
    // specific cube
    inline int get_x_vert(const glm::ivec3 &grid_coord) const
    {
        auto iter = _x_verts.find(index(grid_coord));
        return (iter == _x_verts.end()) ? -1 : iter->second;
    }

    // accesses the pre-computed vertex index on the lower longitudinal edge of
    // a specific cube
    inline int get_y_vert(const glm::ivec3 &grid_coord) const
    {
        auto iter = _y_verts.find(index(grid_coord));
        return (iter == _y_verts.end()) ? -1 : iter->second;
    }

    // accesses the pre-computed vertex index on the lower vertical edge of a
    // specific cube
    inline int get_z_vert(const glm::ivec3 &grid_coord) const
    {
        auto iter = _z_verts.find(index(grid_coord));
        return (iter == _z_verts.end()) ? -1 : iter->second;
    }

    /**
     * sets the pre-computed vertex index on the lower horizontal edge of a
     * specific cube
     * \param val the index of the new vertex
     * \param i abscisse of the cube
     * \param j ordinate of the cube
     * \param k height of the cube
     */
    inline void set_x_vert(const int val, const int i, const int j, const int k)
    {
        _x_verts[index(i, j, k)] = val;
    }
    /**
     * sets the pre-computed vertex index on the lower longitudinal edge of a
     * specific cube
     * \param val the index of the new vertex
     * \param i abscisse of the cube
     * \param j ordinate of the cube
     * \param k height of the cube
     */
    inline void set_y_vert(const int val, const int i, const int j, const int k)
    {
        _y_verts[index(i, j, k)] = val;
    }
    /**
     * sets the pre-computed vertex index on the lower vertical edge of a
     * specific cube
     * \param val the index of the new vertex
     * \param i abscisse of the cube
     * \param j ordinate of the cube
     * \param k height of the cube
     */
    inline void set_z_vert(const int val, const int i, const int j, const int k)
    {
        _z_verts[index(i, j, k)] = val;
    }

    //-----------------------------------------------------------------------------
    // Elements
   protected:
    // selects wether the algorithm will use the enhanced topologically
    // controlled lookup table or the original MarchingCubes
    bool _originalMC;

    glm::ivec3 _size;
    std::vector<float> _data;

    std::unordered_map<int, int> _x_verts;
    std::unordered_map<int, int> _y_verts;
    std::unordered_map<int, int> _z_verts;

    std::vector<Vertex> _vertices;
    std::vector<Triangle> _triangles;
};
//_____________________________________________________________________________

#endif  // _MARCHINGCUBES_H_
