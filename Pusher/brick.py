import numpy as np
from stl import mesh

"""
h(oogte)
b(reedte)
l(lengte)

voor 1 kant meerdere triangles maken

## course
h = 0.1
b = 7
l = 40

## boat
h = 0.05
b = 0.6
l = 6

## blade
h = 0.2
b = 0.45
l = 0.45

In vlak z = 0 meerdere triangles maken
Zijkanten ook meerd
Bovenkant 2
aanname l > b
en brick minstens 0.5*0.5
"""
h = 0.05
b = 0.45
l = 0.45


n = 2 * int(l/0.4)
m = 2 * int(b/0.4)
bdiff = b/m
ldiff = l/n

print("n m", n, " ", m, type(bdiff))

# create vertices of base plane
vertices = np.zeros(((n+1)*(m+1) + 4, 3))
print("vertices.shape ", vertices.shape, "diff ", bdiff, ldiff)
row = 0
for i in range(m+1):
    for j in range(n+1):
        #print(j,i, " ", row*n+j+row)
        vertices[row*n + j+row] = (j*ldiff, i*bdiff, 0)
    row += 1

# now the other planes
restv = (n+1)*(m+1)
vertices[restv] = (0, 0, h)
vertices[restv+1] = (l, 0, h)
vertices[restv+2] = (0, b, h)
vertices[restv+3] = (l, b, h)

# center brick on origin
for i in range(vertices.shape[0]):
    vertices[i] = (vertices[i, 0]-l/2, vertices[i, 1]-b/2, vertices[i, 2]-h/2)
#print(vertices)

# create faces base plane
#  make sure al facet normals are pointing outward
faces = np.zeros((2*n * m + 2*(n+1) + 2*(m+1) + 2, 3))
row = 0
index = 0
for i in range(0,m,2):
    for j in range(0,n,2):
        # fill a square of 2x2 vertices with 8 triangles
        startindex =  row*(2*n+1) + j + row
        #print("(", j, ", ", i, ") index:", startindex)
        # face 1
        faces[index] = (startindex, startindex+n+1, startindex+n+2)
        index += 1
        # face 2
        faces[index] = (startindex, startindex+n+2, startindex+1)
        index += 1
        # face 3
        faces[index] = (startindex+2, startindex+1, startindex+n+2)
        index += 1
        # face 4
        faces[index] = (startindex+2, startindex+n+2, startindex+n+3)
        index += 1
        # face 5
        faces[index] = (startindex+2*(n+1), startindex+n+2, startindex+n+1)
        index += 1
        # face 6
        faces[index] = (startindex+2*(n+1), startindex+2*(n+1)+1, startindex+n+2)
        index += 1
        # face 7
        faces[index] = (startindex+2*(n+1)+1, startindex+2*(n+1)+2, startindex+n+2)
        index += 1
        # face 8
        faces[index] = (startindex+2*(n+1)+2, startindex+n+3, startindex+n+2)
        index += 1
        
    row += 1

# now the other faces

# side 1
faces[index]   = (0, restv+1, restv)
for i in range(n):
    faces[index+1+i] = (n-1-i, n-i, restv+1)    # divide into faces n to avoid simbody restriction
    #print("f ", n-1-i, n-i, restv+1)

# side 2
index = index+n+1
faces[index] = (n, restv+3, restv+1)
for i in range(m):
    faces[index+1+i] = (n+i*(n+1), n+(i+1)*(n+1), restv+3)
    #print("f2 ", n+i*(n+1), n+(i+1)*(n+1), restv+3)

# side 3
index = index+m+1
faces[index] = (restv+2, restv+3, (n+1)*(m))
print("fff ", restv+2, restv+3, (n+1)*(m))
for i in range(n):
    faces[index+1+i] = ((n+1)*(m+1)-1-i, (n+1)*(m+1)-2-i, restv+3)
    #print("f3 ", (n+1)*(m+1)-i, (n+1)*(m+1)-1-i, restv+3)

# side 4
index = index+n+1
faces[index] = (0, restv, restv+2)
for i in range(m):
    faces[index+1+i] = (restv+2, (n+1)*(m-i), (n+1)*(m-i-1))
    #print("f4 ", restv+2, (n+1)*(m), (n+1)*(m-i-1))

# side above
index = index+m+1
print(faces.shape, index)
faces[index] = (restv, restv+1, restv+3)
faces[index+1] = (restv, restv+3, restv+2)


#print(faces.astype(int))
print("onder, iedere zijkant, bovenkant: ", 2*n * m, 2*n+2, 2*m+2, 2)


"""
vertices = np.array([\
    [0, 0, 0],
    [l, 0, 0],
    [l, b, 0],
    [0, b, 0],

    # knooppunten andere kant
    [0, 0, h],
    [l, 0, h],
    [l, b, h],
    [0, b, h]])



# Define the 12 triangles composing the cube
faces = np.array([\
    [0,3,1],
    [1,3,2],
    [0,4,7],
    [0,7,3],
    [4,5,6],
    [4,6,7],
    [5,1,2],
    [5,2,6],
    [2,3,6],
    [3,7,6],
    [0,1,5],
    [0,5,4]])
"""

# Create the mesh
brick = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
for i, f in enumerate(faces):
    for j in range(3):
        brick.vectors[i][j] = vertices[int(f[j]), :]

brick.save('brick.stl')
