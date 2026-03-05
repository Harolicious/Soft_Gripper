"""Created on Wed Dec 17 17:33:57 2025

@author: lab
"""

import Constants
import gmsh

gmsh.initialize()

CantidadCavities = Constants.CantidadCavities
AlturaCavities = Constants.AlturaCavities
AnchoCavities = Constants.AnchoCavities
LadoCavities = Constants.LadoCavities

SeparacionCavities = Constants.SeparacionCavities

LadoMold = Constants.LadoMold
AlturaMold = Constants.AlturaMold
AnchoMold = Constants.AnchoMold

cilindrolargo = LadoMold
cilindroradius = 2

BoxTag = gmsh.model.occ.addBox(-LadoMold/2, -AlturaMold/2, -AnchoMold/2, LadoMold, AlturaMold, AnchoMold)
DimTagBox = (3, BoxTag)

# BoxTag2 = gmsh.model.occ.addBox(-LadoCavities/2, -AlturaCavities/2, -AnchoCavities/2, LadoCavities/2, AlturaCavities/2, AnchoCavities/2)
# DimTagBox2 = (3,BoxTag2)

# CilinderTag = gmsh.model.occ.addCylinder(-LadoMold, 0, 0, LadoMold, 0, 0, cilindroradius) 
# DimTagCilider = (3,CilinderTag)

gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(3)
gmsh.fltk.run()

gmsh.finalize()