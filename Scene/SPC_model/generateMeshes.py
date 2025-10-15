import gmsh
import numpy as np

gmsh.initialize()

gmsh.merge("Dedo_v1.step")
gmsh.model.mesh.generate(3)
gmsh.write("dedo_v1.vtk")
gmsh.fltk.run()
gmsh.clear()

gmsh.merge("A.step")
gmsh.model.mesh.generate(2)
gmsh.write("A_visu.stl")
gmsh.fltk.run()
gmsh.clear()

gmsh.merge("B.step")
gmsh.model.mesh.generate(2)
gmsh.write("B.stl")
gmsh.fltk.run()
gmsh.clear()


gmsh.finalize()