# 3DMeshEditor
A C++ standalone program that uses OpenGL to render and edit multiple 3D objects.

This project is titled ‘3D Mesh Editor’. The project is an extended implementation of the
‘Mesh Editor’ project template and has implemented all the functionalities present in the
project proposal. The implemented functionalities of the program are explained below.

## Render complex meshes on the screen:
The program reads .off files titled mesh0, mesh1, mesh2 up to mesh9. The code for
reading the files has been generalized so the program is compatible with any mesh.
Multiple meshes of different models can be loaded onto the screen at once. The keys to
load the mesh are 1 to 9.
## Perform basic transformations (Translate, rotate, scale) on the meshes:
The program implements application of Eigen Matrixes to perform transformations on
matrixes. Model selection is done with the mouse. The selected model and its vertex are
pointed to by a red line from the mouse cursor to the vertex of the model that was clicked
on. Clockwise and anticlockwise rotations in X, Y, Z axis can be done using the T, Y, U, I,
O and P keys. X, Y Translations with the arrow keys, Z Translation with the ‘+’ and ‘-’
keys, and scaling is done with the K and L keys.
## Enable vertex selection and translation:
Vertexes are selected the same way as objects. Once a mouse is clicked, the object with
the selected vertex is selected as well as the vertex itself. This way, vertexes of any
object can be selected. Vertex translations can be done with the numpad keys 8, 4, 6, 2,
7 and 9 for X, Y and Z axes respectively. The selected vertex is highlighted in blue. It has
been ensured that the vertex highlight does not get affected by scene lighting.
## Perform Vertex color manipulation:
The selected vertex can be given 4 different colors by pressing F1, F2, F3 and F4 keys.
The colors of vertexes are managed properly so selecting and unselecting a vertex changes
its color from the original color to the highlight color and back to the original.
## Vertex deletion via edge Collapse:
The selected vertex can be removed from the mesh (Mesh Simplification) by pressing the X
key. The program selects the closest vertex to it to select an edge, then it performs an
edge collapse operation that removes the edge and selected vertex.
## Dynamic Lighting:
Dynamic Lighting via the shader can be enabled and disabled with the Space bar.
## Camera Control:
The camera control is done via transformations in the shader and is done with the W, S, A,
D keys for Translation and Z, C keys for Rotation.
