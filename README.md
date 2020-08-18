# Cork
Cork Computational Library forked from source code by Gilbert Bernstein.

This adds an integer id to each triangle that is preserved through the CSG operation, and given to any triangle subdivided from an input triangle.

If you just want to preseve the source mesh, put a mesh ID in there. If you need to preserve the material (for example for physics simulation) you can put a material ID.
To transfer per-vertex attribute (normal, vertex colour, uv, etc.) you can put a unique triangle ID in there to map back to the source mesh and triangle. You can then interpolate the vertex parameters for each new vertex.
