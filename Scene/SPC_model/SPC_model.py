import Sofa
import os
import numpy as np

path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'


# ----------------- CREAR ESCENA -----------------
def createScene(rootNode):

    # ---------- Plugins y estilo ----------
    rootNode.addObject(
        "RequiredPlugin",
        pluginName="""SofaPython3
        SoftRobots
        SoftRobots.Inverse
        Sofa.Component.Collision
        Sofa.Component.Collision.Detection.Algorithm
        Sofa.Component.Collision.Detection.Intersection
        Sofa.Component.Collision.Response.Contact
        Sofa.Component.Collision.Geometry
        Sofa.Component.AnimationLoop
        Sofa.Component.Constraint.Lagrangian.Correction
        Sofa.Component.Constraint.Lagrangian.Solver
        Sofa.Component.Engine.Select
        Sofa.Component.IO.Mesh
        Sofa.Component.LinearSolver.Direct
        Sofa.Component.LinearSolver.Iterative
        Sofa.Component.Mapping.Linear
        Sofa.Component.Mapping.MappedMatrix
        Sofa.Component.Mapping.NonLinear
        Sofa.Component.Mass
        Sofa.Component.ODESolver.Backward
        Sofa.Component.Setting
        Sofa.Component.SolidMechanics.FEM.Elastic
        Sofa.Component.SolidMechanics.Spring
        Sofa.Component.StateContainer
        Sofa.Component.Topology.Container.Constant
        Sofa.Component.Topology.Container.Dynamic
        Sofa.Component.Topology.Mapping
        Sofa.Component.Visual
        Sofa.GL.Component.Rendering3D
        Sofa.GL.Component.Shader"""
    )

    rootNode.addObject(
        "VisualStyle",
        displayFlags="""
            hideWireframe
            showBehaviorModels
            hideCollisionModels
            hideBoundingCollisionModels
            showForceFields
            showInteractionForceFields"""
    )

    
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.gravity = [0.0, 0.0, -9.8]
    rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 1e-7)
    rootNode.dt = 0.1

    # ---------- Colisión ----------
    rootNode.addObject("CollisionPipeline", depth=20, verbose=0)
    rootNode.addObject("BruteForceBroadPhase")
    rootNode.addObject("BVHNarrowPhase")
    rootNode.addObject("DiscreteIntersection")   # 👈 este es el que te falta
    rootNode.addObject("DefaultContactManager", name="contactManager", response="FrictionContactConstraint")
    rootNode.addObject("LocalMinDistance", name="Proximity", alarmDistance=1.0, contactDistance=0.5)
    
    # -----------Mouse-------------
    mouse = rootNode.addChild("Mouse")
    mouse.addObject("MechanicalObject", name="MousePosition", template="Vec3d", position="0 0 0")

    # ---------- GRIPPER ----------
    gripper = rootNode.addChild('gripper')
    gripper.addObject('EulerImplicitSolver', name='odesolver')
    gripper.addObject('SparseLDLSolver', name='preconditioner', template="CompressedRowSparseMatrixMat3x3d")
    gripper.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver',
                      tolerance=1e-5, preconditioners='@preconditioner', use_precond=True, update_step=1)

    gripper.addObject('MeshVTKLoader', name='loader', filename='A.vtk')
    gripper.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container', edges=[])
    gripper.addObject('TetrahedronSetTopologyModifier')
    gripper.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
    gripper.addObject('UniformMass', totalMass=0.5)
    gripper.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM',
                      method='large', poissonRatio=0.4,  youngModulus=1e8)
    gripper.addObject('BoxROI', name='boxROI', box=[20, 20, 50, -20, -20, 35],
                      drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
    gripper.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
    gripper.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')

    # ---- Cavity ----
    cavity = gripper.addChild('cavity')
    cavity.addObject('MeshSTLLoader', name='loader', filename='B.stl')
    cavity.addObject('MeshTopology', src='@loader', name='topo')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=280000, valueType=0)
    cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

    # ---------- BOX ----------
    box = rootNode.addChild('box')
    box.addObject('EulerImplicitSolver', name='odesolver')
    box.addObject('SparseLDLSolver', name='preconditioner', template="CompressedRowSparseMatrixMat3x3d")
    box.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver',
                  tolerance=1e-5, preconditioners='@preconditioner', use_precond=True, update_step=1)
    box.addObject('MeshVTKLoader', name='loader3', filename='S.vtk')
    box.addObject('TetrahedronSetTopologyContainer', src='@loader3', name='container', edges=[])
    box.addObject('TetrahedronSetTopologyModifier')
    box.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
    box.addObject('UniformMass', totalMass=5)
    box.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM_box',
                      method='large', poissonRatio=0.4,  youngModulus=1e4)
    box.addObject('BoxROI', name='boxROI', box=[-20, -20, -38, 20, 20, -50],
                      drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
    box.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e10)
    box.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')

    # ---------- VISUALIZACIÓN ----------
    gripperVisu = gripper.addChild('visu')
    gripperVisu.addObject("MeshSTLLoader", filename="A_visu.stl", name="loader")
    gripperVisu.addObject("OglModel", src="@loader")
    gripperVisu.addObject("BarycentricMapping")

    boxVisu = box.addChild('visu')
    boxVisu.addObject("MeshSTLLoader", filename="S_visu.stl", name="loader")
    boxVisu.addObject("OglModel", src="@loader")
    boxVisu.addObject("BarycentricMapping")

    # ---------- COLLISIONES ----------
    gripperCollis = gripper.addChild("collision")
    gripperCollis.addObject("MeshSTLLoader", filename="A_visu.stl", name="loader")
    gripperCollis.addObject("MeshTopology", src="@loader", name="topo")
    gripperCollis.addObject("MechanicalObject")
    gripperCollis.addObject("TriangleCollisionModel", moving=1, simulated=1)
    gripperCollis.addObject("LineCollisionModel", moving=1, simulated=1)
    gripperCollis.addObject("PointCollisionModel", moving=1, simulated=1)
    gripperCollis.addObject("BarycentricMapping")

    boxCollis = box.addChild("collision")
    boxCollis.addObject("MeshSTLLoader", filename="S_visu.stl", name="loader")
    boxCollis.addObject("MeshTopology", src="@loader", name="topo")
    boxCollis.addObject("MechanicalObject")
    boxCollis.addObject("TriangleCollisionModel", moving=1, simulated=1)
    boxCollis.addObject("LineCollisionModel", moving=1, simulated=1)
    boxCollis.addObject("PointCollisionModel", moving=1, simulated=1)
    boxCollis.addObject("BarycentricMapping")

    return rootNode
