
import gmsh
import numpy as np

L = 20
center = [0, 0, -33] 
x0 = center[0] - L/2
y0 = center[1] - L/2
z0 = center[2] - L/2

gmsh.initialize()

gmsh.merge("A.step")
gmsh.option.setNumber("Mesh.CharacteristicLengthMin", 18)
gmsh.option.setNumber("Mesh.CharacteristicLengthMax", 18)
gmsh.model.mesh.generate(3)
gmsh.write("A.vtk")
gmsh.fltk.run()
gmsh.clear()

gmsh.merge("B.step")
gmsh.model.mesh.generate(2)
gmsh.write("B.stl")
gmsh.fltk.run()
gmsh.clear()

gmsh.model.occ.addBox(x0, y0, z0, L, L, L)
gmsh.model.occ.synchronize()
gmsh.option.setNumber("Mesh.CharacteristicLengthMin", 8)
gmsh.option.setNumber("Mesh.CharacteristicLengthMax", 8)
gmsh.model.mesh.generate(3)
gmsh.write("S.vtk")
gmsh.fltk.run()
gmsh.clear()

gmsh.model.occ.addBox(x0, y0, z0, L, L, L)
gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("S_visu.stl")
gmsh.fltk.run()
gmsh.clear()

gmsh.merge("A.step")
gmsh.model.mesh.generate(2)
gmsh.write("A_visu.stl")
gmsh.fltk.run()
gmsh.clear()

gmsh.finalize()