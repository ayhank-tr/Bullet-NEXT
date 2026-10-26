# Interface audit (C0.11) — Blender 2.79b game engine

Source: Blender v2.79b (commit f4dc9f9d68bd), source/gameengine.

## PHY interface methods (pure virtual surface a backend must implement)

### PHY_ICharacter.h
GetGravity GetJumpCount GetMaxJumps GetWalkDirection Jump OnGround PHY_ICharacter SetGravity SetMaxJumps SetWalkDirection 
### PHY_IController.h
GetNewClientInfo PHY_IController SetNewClientInfo SetPhysicsEnvironment 
### PHY_IGraphicController.h
Activate GetReplica SetGraphicTransform SetLocalAabb 
### PHY_IMotionState.h
CalculateWorldTransformations GetWorldOrientation GetWorldPosition GetWorldScaling PHY_IMotionState SetWorldOrientation SetWorldPosition 
### PHY_IPhysicsController.h
AddCompoundChild ApplyForce ApplyImpulse ApplyTorque CalcXform GetAngularDamping GetAngularVelocity GetAngularVelocityMax GetAngularVelocityMin GetLinVelocityMax GetLinVelocityMin GetLinearDamping GetLinearVelocity GetLocalInertia GetMargin GetMass GetMotionState GetOrientation GetPosition GetRadius GetReplica GetReplicaForSensors GetVelocity IsCompound IsDynamic IsSuspended PHY_IPhysicsController PostProcessReplica RefreshCollisions ReinstancePhysicsShape RelativeRotate RelativeTranslate RemoveCompoundChild ReplicateConstraints ResolveCombinedVelocities RestoreDynamics SetActive SetAngularDamping SetAngularVelocity SetAngularVelocityMax SetAngularVelocityMin SetDamping SetLinVelocityMax SetLinVelocityMin SetLinearDamping SetLinearVelocity SetMargin SetMass SetOrientation SetPhysicsEnvironment SetPosition SetRadius SetRigidBody SetScaling SetTransform SuspendDynamics SynchronizeMotionStates WriteDynamicsToMotionState WriteMotionStateToDynamics 
### PHY_IPhysicsEnvironment.h
AddSensor AddTouchCallback BeginFrame ConvertObject CreateConeController CreateConstraint CreateSphereController CullingTest DebugDrawWorld EndFrame ExportFile GetAppliedImpulse GetCharacterController GetConstraintParam GetDebugMode GetFixedTimeStep GetGravity GetNumTimeSubSteps GetVehicleConstraint MergeEnvironment PHY_IPhysicsEnvironment PHY_IRayCastFilterCallback ProceedDeltaTime RayTest RemoveCollisionCallback RemoveConstraintById RemoveSensor RequestCollisionCallback SetCcdMode SetConstraintParam SetContactBreakingTreshold SetDeactivationAngularTreshold SetDeactivationLinearTreshold SetDeactivationTime SetDebugMode SetFixedTimeStep SetGravity SetLinearAirDamping SetNumIterations SetNumTimeSubSteps SetSolverDamping SetSolverSorConstant SetSolverTau SetSolverType SetUseEpa SetupObjectConstraints needBroadphaseRayCast reportHit 
### PHY_IVehicle.h
AddWheel ApplyBraking ApplyEngineForce GetNumWheels GetUserConstraintId GetUserConstraintType GetWheelOrientationQuaternion GetWheelPosition GetWheelRotation PHY_IVehicle SetCoordinateSystem SetRollInfluence SetSteeringValue SetSuspensionCompression SetSuspensionDamping SetSuspensionStiffness SetWheelFriction 

## Places that bypass the PHY interface

| File | Direct Bullet use | Needed for a second backend |
| --- | --- | --- |
| Converter/KX_SoftBodyDeformer.cpp | Reads btSoftBody::m_nodes to deform the render mesh | Move behind a PHY soft-body vertex query |
| Ketsji/KX_PyConstraintBinding.cpp | btIDebugDraw::DBG_* constants for PhysicsConstraints debug flags | Mirror the constants in PHY_DynamicTypes.h |
| Ketsji/KX_Scene.h | Forward declaration of btCollisionShape | Remove or replace with an opaque PHY handle |
| Converter/BL_BlenderDataConversion.cpp, KX_BlenderSceneConverter.cpp | Construct CcdPhysicsEnvironment / CcdPhysicsController directly | Backend factory chosen per scene |

Everything else in the game engine talks to physics only through PHY_I* interfaces.
