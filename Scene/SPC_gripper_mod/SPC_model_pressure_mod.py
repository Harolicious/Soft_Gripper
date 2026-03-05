import Sofa
import os
import numpy as np

# ruta relativa al directorio del script
path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'

PSI = 10.0

class Controller(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # atributos esperados via kwargs
        self.root = kwargs['RootNode']
        self.spc = kwargs['SPC']
        self.end_effector_mo = kwargs['EndEffectorMO']

        # parámetros de actuation
        self.max_pressure = 6.89 * PSI * 1000# Pa
        self.increment = self.max_pressure / 1000.0
        self.pressure = 0.0

        # estados de animación
        self.decreasing = False
        self.animation_finished = False
        self._dt_zeroed = False  # marca para no reasignar dt repetidamente

        print(f"Controller inicializado: {self.name.value}")

    def _set_spc_pressure(self, pressure):
        """Helper para actualizar el valor del SPC de forma consistente."""
        # SurfacePressureConstraint espera un valor escalar en .value (a veces lista)
        try:
            # algunos bindings usan .value o .value.value; intentamos ambas
            self.spc.value.value = [pressure]
        except Exception:
            try:
                self.spc.value = [pressure]
            except Exception:
                # como último recurso, intentar asignar directamente (no ideal, pero robusto)
                setattr(self.spc, 'value', [pressure])

    def update_pressure_increase(self):
        self.pressure += self.increment
        if self.pressure > self.max_pressure and not self.animation_finished:
            self.pressure = self.max_pressure
        self._set_spc_pressure(self.pressure)

    def update_pressure_decrease(self):
        self.pressure -= self.increment
        if self.pressure < 0.0 and not self.animation_finished:
            self.pressure = 0.0
        self._set_spc_pressure(self.pressure)

    def onAnimateBeginEvent(self, eventType):
        # si ya terminó la animación, fijar dt a 0 una sola vez y salir
        if self.animation_finished:
            if not self._dt_zeroed:
                # detener avances de tiempo en la simulación
                try:
                    self.root.dt = 0.0
                except Exception:
                    pass
                self._dt_zeroed = True
            # dejar la presión en 0 por seguridad
            self.pressure = 0.0
            self._set_spc_pressure(0.0)
            return

        # ciclo de subida / bajada
        if not self.decreasing:
            self.update_pressure_increase()
            if self.pressure >= self.max_pressure:
                self.decreasing = True
        else:
            self.update_pressure_decrease()
            if self.pressure <= 0.0:
                self.decreasing = False
                self.animation_finished = True


def createScene(rootNode):
    # Plugins necesarios (agrupados en una sola llamada)
    plugins = [
        "SofaPython3",
        "SoftRobots",
        "SoftRobots.Inverse",
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.Engine.Select",
        "Sofa.Component.IO.Mesh",
        "Sofa.Component.LinearSolver.Direct",
        "Sofa.Component.LinearSolver.Iterative",
        "Sofa.Component.Mapping.Linear",
        "Sofa.Component.Mapping.MappedMatrix",
        "Sofa.Component.Mapping.NonLinear",
        "Sofa.Component.Mass",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.Setting",
        "Sofa.Component.SolidMechanics.FEM.Elastic",
        "Sofa.Component.SolidMechanics.Spring",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Topology.Container.Dynamic",
        "Sofa.Component.Visual",
        "Sofa.GL.Component.Rendering3D",
        "Sofa.GL.Component.Shader",
        "Sofa.Component.Topology.Mapping"
    ]
    rootNode.addObject("RequiredPlugin", pluginName=" ".join(plugins))

    rootNode.addObject(
        "VisualStyle",
        displayFlags="""
            hideWireframe
            showBehaviorModels
            hideCollisionModels
            hideBoundingCollisionModels
            showForceFields
            showInteractionForceFields
        """,
    )
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-6)
    rootNode.dt = 0.1

    # gripper
    gripper = rootNode.addChild('gripper')
    gripper.addObject('EulerImplicitSolver', name='odesolver')
    gripper.addObject('SparseLDLSolver', name='preconditioner', template="CompressedRowSparseMatrixMat3x3d")
    gripper.addObject('ShewchukPCGLinearSolver',
                      iterations=15,
                      name='linearsolver',
                      tolerance=1e-5,
                      preconditioner='@preconditioner',
                      use_precond=True,
                      update_step=1)

    gripper.addObject('MeshVTKLoader', name='loader', filename='a.vtk')
    gripper.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    gripper.addObject('TetrahedronSetTopologyModifier')
    gripper.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices=False)
    gripper.addObject('UniformMass', totalMass=0.5)

    # BoxROI para elementos más rígidos: coordenadas [xmin, ymin, zmin, xmax, ymax, zmax]
    gripper.addObject('BoxROI',
                      name='boxROIStiffness',
                      box=[-45, -10, 26, 45, 10, 32],
                      drawBoxes=True,
                      position="@tetras.rest_position",
                      tetrahedra="@container.tetrahedra")
    
    gripper.addObject('TetrahedronFEMForceField',
                      template='Vec3d',
                      name='FEM',
                      method='large',
                      poissonRatio=0.49,
                      youngModulus=2000000.0)

    gripper.addObject('BoxROI', name='boxROI',
                      box=[-45, -10, 18, -40, 10, 32],
                      drawBoxes=True,
                      position="@tetras.rest_position",
                      tetrahedra="@container.tetrahedra")
    
    gripper.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
    gripper.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')

    # gripper/cavity
    cavity = gripper.addChild('cavity')
    cavity.addObject('MeshSTLLoader', name='loader', filename='b.stl')
    cavity.addObject('MeshTopology', src='@loader', name='topo')
    cavity.addObject('MechanicalObject', name='cavity')
    # SurfacePressureConstraint: inicializamos con 0
    SPC = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=True, mapMasses=True)

    # End-effector
    EndEffectorNode = gripper.addChild("EndEffectorNode")
    EndEffectorMO = EndEffectorNode.addObject("MechanicalObject", position=[[40, 0, 30]], showObject=True, showObjectScale=10)
    EndEffectorNode.addObject("BarycentricMapping")

    # visualization child
    gripperVisu = gripper.addChild('visu')
    gripperVisu.addObject("MeshSTLLoader", filename="a_visu.stl", name="loader")
    gripperVisu.addObject("OglModel", src="@loader")
    gripperVisu.addObject("BarycentricMapping")

    # añadir controlador (pasando referencias)
    rootNode.addObject(Controller(name="ActuationController",
                                  RootNode=rootNode,
                                  SPC=SPC,
                                  EndEffectorMO=EndEffectorMO))

    return rootNode
