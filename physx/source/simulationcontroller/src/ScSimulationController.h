// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved. 

#ifndef SC_SIMULATION_CONTROLLER_H
#define	SC_SIMULATION_CONTROLLER_H

#include "PxsSimulationController.h"

namespace physx
{

class PxsHeapMemoryAllocator;

namespace Dy
{
	class FeatherstoneArticulation;
	struct ArticulationJointCore;
	class ParticleSystem;
	class FEMCloth;
}

namespace Sc
{

	class SimulationController : public PxsSimulationController
	{
		PX_NOCOPY(SimulationController)
	public:
		SimulationController(PxsSimulationControllerCallback* callback) : PxsSimulationController(callback)
		{
		}

		virtual ~SimulationController() {}
		virtual void addJoint(const PxU32 /*edgeIndex*/, Dy::Constraint* /*constraint*/, IG::IslandSim& /*islandSim*/, PxArray<PxU32>& /*jointIndices*/,
		PxPinnedArray<PxgSolverConstraintManagerConstants>& /*managerIter*/, PxU32 /*uniqueId*/){}
		virtual void removeJoint(const PxU32 /*edgeIndex*/, Dy::Constraint* /*constraint*/, PxArray<PxU32>& /*jointIndices*/, IG::IslandSim& /*islandSim*/){}
		virtual void addShape(PxsShapeSim* /*shapeSim*/, const PxU32 /*index*/){}
		virtual void reinsertShape(PxsShapeSim* /*shapeSim*/, const PxU32 /*index*/) {}
		virtual void removeShape(const PxU32 /*index*/){}
		virtual void addDynamic(PxsRigidBody* /*rigidBody*/, const PxNodeIndex& /*nodeIndex*/){}
		virtual void addDynamics(PxsRigidBody** /*rigidBody*/, const PxU32* /*nodeIndex*/, PxU32 /*nbBodies*/) {}
		virtual void addArticulation(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void releaseArticulation(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void releaseDeferredArticulationIds() {}

		virtual void flush() {}
		virtual void updateDynamic(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void updateJoint(const PxU32 /*edgeIndex*/, Dy::Constraint* /*constraint*/){}
		virtual void updateBodies(PxsRigidBody** /*rigidBodies*/,  PxU32* /*nodeIndices*/, const PxU32 /*nbBodies*/) {}
		virtual void updateBody(PxsRigidBody* /*rigidBody*/, const PxU32 /*nodeIndex*/) {}
		virtual void updateBodies(PxBaseTask* /*continuation*/){}
		virtual void updateShapes(PxBaseTask* /*continuation*/) {}
		virtual	void preIntegrateAndUpdateBound(PxBaseTask* /*continuation*/, const PxVec3 /*gravity*/, const PxReal /*dt*/){}
		virtual void updateParticleSystemsAndSoftBodies(){}
		virtual void sortContacts(){}
		virtual void update(PxBitMapPinned& /*changedHandleMap*/){}
		virtual void mergeChangedAABBMgHandle(const PxU32 /*maxAABBMgHandles*/, const bool /*suppressedReadback*/) {}
		virtual void gpuDmabackData(PxsTransformCache& /*cache*/, Bp::BoundsArray& /*boundArray*/, PxBitMapPinned& /*changedAABBMgrHandles*/, bool /*suppress*/){}
		virtual void updateScBodyAndShapeSim(PxsTransformCache& /*cache*/, Bp::BoundsArray& /*boundArray*/, PxBaseTask* /*continuation*/);
		virtual void updateArticulation(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void updateArticulationJoint(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void updateArticulationTendon(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void updateArticulationExtAccel(Dy::FeatherstoneArticulation* /*articulation*/, const PxNodeIndex& /*nodeIndex*/) {}
		virtual void updateArticulationAfterIntegration(PxsContext*	/*llContext*/, Bp::AABBManagerBase* /*aabbManager*/,
			PxArray<Sc::BodySim*>& /*ccdBodies*/, PxBaseTask* /*continuation*/, IG::IslandSim& /*islandSim*/, const float /*dt*/);
		virtual PxU32* getActiveBodies() { return NULL; }
		virtual PxU32* getDeactiveBodies() { return NULL; }
		virtual void** getRigidBodies() { return NULL; }
		virtual PxU32	getNbBodies() { return 0; }
		virtual PxU32	getNbFrozenShapes() { return 0; }
		virtual PxU32	getNbUnfrozenShapes() { return 0; }

		virtual PxU32*	getUnfrozenShapes() { return NULL; }
		virtual PxU32*	getFrozenShapes() { return NULL; }
		virtual PxsShapeSim** getShapeSims() { return NULL; }
		virtual PxU32	getNbShapes()	{ return 0; }

		virtual void	clear() { }
		virtual void	setBounds(Bp::BoundsArray* /*boundArray*/){}
		virtual void	reserve(const PxU32 /*nbBodies*/) {}

		
		virtual PxU32   getArticulationRemapIndex(const PxU32 /*nodeIndex*/) { return PX_INVALID_U32;}
		
		//virtual void	setParticleSystemManager(PxgParticleSystemCore* /* psCore*/) {}

		virtual void	copyArticulationData(void* /*jointData*/, void* /*index*/, PxArticulationGpuDataType::Enum /*flag*/, const PxU32 /*nbUpdatedArticulations*/, void* /*copyEvent*/) {}
		virtual void	applyArticulationData(void* /*data*/, void* /*index*/, PxArticulationGpuDataType::Enum /*flag*/, const PxU32 /*nbUpdatedArticulations*/, void* /*waitEvent*/, void* /*signalEvent*/) {}

		virtual void	copySoftBodyData(void** /*data*/, void* /*dataSizes*/, void* /*softBodyIndices*/, PxSoftBodyDataFlag::Enum /*flag*/, const PxU32 /*nbCopySoftBodies*/, const PxU32 /*maxSize*/, void* /*copyEvent*/) {}
		virtual void	applySoftBodyData(void** /*data*/, void* /*dataSizes*/, void* /*softBodyIndices*/, PxSoftBodyDataFlag::Enum /*flag*/, const PxU32 /*nbUpdatedSoftBodies*/, const PxU32 /*maxSize*/, void* /*applyEvent*/) {}

		virtual	void	copyContactData(Dy::Context* /*dyContext*/, void* /*data*/, const PxU32 /*maxContactPairs*/, void* /*numContactPairs*/, void* /*copyEvent*/) {}
		virtual	void	copyBodyData(PxGpuBodyData* /*data*/, PxGpuActorPair* /*index*/, const PxU32 /*nbCopyActors*/, void* /*copyEvent*/){}
		
		virtual	void	applyActorData(void* /*data*/, PxGpuActorPair* /*index*/, PxActorCacheFlag::Enum /*flag*/, const PxU32 /*nbUpdatedActors*/, void* /*waitEvent*/, void* /*signalEvent*/) {}

		virtual void*	getMPMDataPointer(const Dy::ParticleSystem& /*psLL*/, PxMPMParticleDataFlag::Enum /*flags*/) { return NULL; }
		virtual void*	getSparseGridDataPointer(const Dy::ParticleSystem& /*psLL*/, PxSparseGridDataFlag::Enum /*flags*/, PxParticleSolverType::Enum /*type*/) { return NULL; }

		virtual void    allocatePBDMaterialsBuffer(PxPhysXGpu* /*physxGpu*/, Dy::ParticleSystemCore& /*core*/, PxU64* /*gpuMemStat*/) {}
		virtual void    allocateFLIPMaterialsBuffer(PxPhysXGpu* /*physxGpu*/, Dy::ParticleSystemCore& /*core*/, PxU64* /*gpuMemStat*/) {}
		virtual void    allocateMPMMaterialsBuffer(PxPhysXGpu* /*physxGpu*/, Dy::ParticleSystemCore& /*core*/, PxU64* /*gpuMemStat*/) {}

		virtual void	syncParticleData() {}

		virtual void    updateBoundsAndShapes(Bp::AABBManagerBase& /*aabbManager*/, const bool/* useGpuBp*/, const bool /*useDirectApi*/){}

		virtual	void	computeDenseJacobians(const PxIndexDataPair* /*indices*/, PxU32 /*nbIndices*/, void* /*computeEvent*/){}
		virtual	void	computeGeneralizedMassMatrices(const PxIndexDataPair* /*indices*/, PxU32 /*nbIndices*/, void* /*computeEvent*/){}
		virtual	void	computeGeneralizedGravityForces(const PxIndexDataPair* /*indices*/, PxU32 /*nbIndices*/, const PxVec3& /*gravity*/, void* /*computeEvent*/){}
		virtual	void	computeCoriolisAndCentrifugalForces(const PxIndexDataPair* /*indices*/, PxU32 /*nbIndices*/, void* /*computeEvent*/) {}
		virtual void    applyParticleBufferData(const PxU32* /*indices*/, const PxGpuParticleBufferIndexPair* /*indexPairs*/, const PxParticleBufferFlags* /*flags*/, PxU32 /*nbUpdatedBuffers*/, void* /*waitEvent*/, void* /*signalEvent*/) {}

		virtual void flushInsertions() {}

	};
}

}

#endif
