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

//_____________________________________________________________________________
// types
/** unsigned char alias */
typedef unsigned char uchar ;
/** signed char alias */
typedef   signed char schar ;
/** isovalue alias */
typedef        float real  ;

//-----------------------------------------------------------------------------
// Vertex structure
/** \struct Vertex "MarchingCubes.h" MarchingCubes
 * Position and normal of a vertex
 * \brief vertex structure
 * \param x X coordinate
 * \param y Y coordinate
 * \param z Z coordinate
 * \param nx X component of the normal
 * \param ny Y component of the normal
 * \param nz Z component of the normal
 */
typedef struct
{
  real  x,  y,  z ;  /**< Vertex coordinates */
  real nx, ny, nz ;  /**< Vertex normal */
} Vertex ;

//-----------------------------------------------------------------------------
// Triangle structure
/** \struct Triangle "MarchingCubes.h" MarchingCubes
 * Indices of the oriented triange vertices
 * \brief triangle structure
 * \param v1 First vertex index
 * \param v2 Second vertex index
 * \param v3 Third vertex index
 */
typedef struct
{
  int v1,v2,v3 ;  /**< Triangle vertices */
} Triangle ;
//_____________________________________________________________________________



//_____________________________________________________________________________
/** Marching Cubes algorithm wrapper */
/** \class MarchingCubes
  * \brief Marching Cubes algorithm.
  */
class MarchingCubes
//-----------------------------------------------------------------------------
{
// Constructors
public :
	/**
   * Main and default constructor
   * \brief constructor
   * \param size_x width  of the grid
   * \param size_y depth  of the grid
   * \param size_z height of the grid
   */
  MarchingCubes ( const int size_x = -1, const int size_y = -1, const int size_z = -1 ) ;

//-----------------------------------------------------------------------------
// Accessors
public :
  /** accesses the number of vertices of the generated mesh */
  inline const int nverts() const { return _vertices.size() ; }
  /** accesses the number of triangles of the generated mesh */
  inline const int ntrigs() const { return _triangles.size() ; }
  /** accesses a specific vertex of the generated mesh */
  inline const Vertex   * vert( const int i ) const { if( i < 0  || i >= nverts() ) return ( Vertex *)NULL ; return _vertices.data()  + i ; }
  /** accesses a specific triangle of the generated mesh */
  inline const Triangle * trig( const int i ) const { if( i < 0  || i >= ntrigs() ) return (Triangle*)NULL ; return _triangles.data() + i ; }

  /** accesses the vertex buffer of the generated mesh */
  inline Vertex   *vertices () { return _vertices.data()  ; }
  /** accesses the triangle buffer of the generated mesh */
  inline Triangle *triangles() { return _triangles.data() ; }

  /**  accesses the width  of the grid */
  inline const int size_x() const { return _size_x ; }
  /**  accesses the depth  of the grid */
  inline const int size_y() const { return _size_y ; }
  /**  accesses the height of the grid */
  inline const int size_z() const { return _size_z ; }

  /**
   * changes the size of the grid
   * \param size_x width  of the grid
   * \param size_y depth  of the grid
   * \param size_z height of the grid
   */
  inline void set_resolution( const int size_x, const int size_y, const int size_z ) { _size_x = size_x ;  _size_y = size_y ;  _size_z = size_z ; }
  /**
   * selects wether the algorithm will use the enhanced topologically controlled lookup table or the original MarchingCubes
   * \param originalMC true for the original Marching Cubes
   */
  inline void set_method    ( const bool originalMC = false ) { _originalMC = originalMC ; }

  // Data access
  /**
   * accesses a specific cube of the grid
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline const real get_data  ( const int i, const int j, const int k ) const { return _data[ i + j*_size_x + k*_size_x*_size_y] ; }
  /**
   * sets a specific cube of the grid
   * \param val new value for the cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline void  set_data  ( const real val, const int i, const int j, const int k ) { _data[ i + j*_size_x + k*_size_x*_size_y] = val ; }

  // Data initialization
  /** inits temporary structures (must set sizes before call) : the grid and the vertex index per cube */
  void init_temps () ;
  /** inits all structures (must set sizes before call) : the temporary structures and the mesh buffers */
  void init_all   () ;


//-----------------------------------------------------------------------------
// Algorithm
public :
  /**
   * Main algorithm : must be called after init_all
   * \param iso isovalue
   */
  void run( real iso = (real)0.0 ) ;

protected :
  /** tesselates one cube */
  void process_cube (const glm::ivec3 &grid_coord, uchar lut_entry, float *cube);


//-----------------------------------------------------------------------------
// Operations
protected :
  /**
   * computes almost all the vertices of the mesh by interpolation along the cubes edges
   * \param iso isovalue
   */
  void compute_intersection_points( real iso ) ;

  /**
   * routine to add a triangle to the mesh
   * \param trig the code for the triangle as a sequence of edges index
   * \param n    the number of triangles to produce
   * \param v12  the index of the interior vertex to use, if necessary
   */
  void add_triangle ( const glm::ivec3 &grid_coord, const char* trig, char n, int v12 = -1 ) ;

  int add_vertex(const glm::ivec3 &grid_coord, const glm::ivec3 &dir, int corner, float *cube);
  /** adds a vertex inside the current cube */
  int add_c_vertex(const glm::ivec3 &grid_coord) ;

  /**
   * interpolates the horizontal gradient of the implicit function at the lower vertex of the specified cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  real get_x_grad( const int i, const int j, const int k ) const ;
  /**
   * interpolates the longitudinal gradient of the implicit function at the lower vertex of the specified cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  real get_y_grad( const int i, const int j, const int k ) const ;
  /**
   * interpolates the vertical gradient of the implicit function at the lower vertex of the specified cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  real get_z_grad( const int i, const int j, const int k ) const ;

  /**
   * accesses the pre-computed vertex index on the lower horizontal edge of a specific cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline int   get_x_vert( const int i, const int j, const int k ) const { return _x_verts[ i + j*_size_x + k*_size_x*_size_y] ; }
  /**
   * accesses the pre-computed vertex index on the lower longitudinal edge of a specific cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline int   get_y_vert( const int i, const int j, const int k ) const { return _y_verts[ i + j*_size_x + k*_size_x*_size_y] ; }
  /**
   * accesses the pre-computed vertex index on the lower vertical edge of a specific cube
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline int   get_z_vert( const int i, const int j, const int k ) const { return _z_verts[ i + j*_size_x + k*_size_x*_size_y] ; }

  /**
   * sets the pre-computed vertex index on the lower horizontal edge of a specific cube
   * \param val the index of the new vertex
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline void  set_x_vert( const int val, const int i, const int j, const int k ) { _x_verts[ i + j*_size_x + k*_size_x*_size_y] = val ; }
  /**
   * sets the pre-computed vertex index on the lower longitudinal edge of a specific cube
   * \param val the index of the new vertex
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline void  set_y_vert( const int val, const int i, const int j, const int k ) { _y_verts[ i + j*_size_x + k*_size_x*_size_y] = val ; }
  /**
   * sets the pre-computed vertex index on the lower vertical edge of a specific cube
   * \param val the index of the new vertex
   * \param i abscisse of the cube
   * \param j ordinate of the cube
   * \param k height of the cube
   */
  inline void  set_z_vert( const int val, const int i, const int j, const int k ) { _z_verts[ i + j*_size_x + k*_size_x*_size_y] = val ; }

//-----------------------------------------------------------------------------
// Elements
protected :
  bool      _originalMC ;   /**< selects wether the algorithm will use the enhanced topologically controlled lookup table or the original MarchingCubes */

	glm::ivec3 _size;
  std::vector<float> _data;

	std::vector<int> _x_verts    ;  /**< pre-computed vertex indices on the lower horizontal   edge of each cube */
	std::vector<int> _y_verts    ;  /**< pre-computed vertex indices on the lower longitudinal edge of each cube */
	std::vector<int> _z_verts    ;  /**< pre-computed vertex indices on the lower vertical     edge of each cube */

  std::vector<Vertex> _vertices   ;  /**< vertex   buffer */
	std::vector<Triangle> _triangles  ;  /**< triangle buffer */
};
//_____________________________________________________________________________


#endif // _MARCHINGCUBES_H_
