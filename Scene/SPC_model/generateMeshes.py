import gmsh

gmsh.initialize()

gmsh.merge("Dedo_v1_mod.step")
gmsh.model.mesh.generate(3)
gmsh.write("a.vtk")
gmsh.fltk.run()
gmsh.clear()

gmsh.merge("Dedo_v1_mod.step")
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.model.mesh.refine()
gmsh.write("a_visu.stl")
gmsh.fltk.run()
gmsh.clear()

gmsh.merge("Capsula_v1_mod.step")
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("b.stl")
gmsh.fltk.run()
gmsh.clear()

gmsh.finalize()