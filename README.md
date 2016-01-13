# Efficient Marching Cubes with Topological Guarantees

This is an update to the original code implementing the paper [Efficient Implementation of Marching Cubes' Cases With Topological Guarantees](https://www-s.ks.uiuc.edu/Research/vmd/projects/ece498/surf/lewiner.pdf).  The [project page](http://www.matmidia.mat.puc-rio.br/tomlew/publication_page.php?pubkey=marching_cubes_jgt) contains a link to the paper as well as original [source code](http://www.matmidia.mat.puc-rio.br/tomlew/srcs/marching_cubes_jgt.zip) written by the authors.

As noted in the author statement in the source code:

```
 * @author  Thomas Lewiner <thomas.lewiner@polytechnique.org>
 * @author  Math Dept, PUC-Rio
 * @version 0.2
 * @date    12/08/2002
```

the original source code was last edited in 2002 and was a mix of C and C++ styles.  I've updated the source code with the following goals in mind:

* Use modern C++11 coding standards
* Modularize the interface for reusability as a library
* Remove global state
* Reduce the dependence on OOP style for better readability of the dependencies of the various methods implementing the main MC algorithm

