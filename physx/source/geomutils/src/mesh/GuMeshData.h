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

#ifndef GU_MESH_DATA_H
#define GU_MESH_DATA_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "foundation/PxBounds3.h"
#include "geometry/PxTriangleMesh.h"

#include "foundation/PxUserAllocated.h"
#include "foundation/PxAllocator.h"
#include "GuRTree.h"
#include "GuBV4.h"
#include "GuBV32.h"
#include "GuSDF.h"

namespace physx
{
namespace Gu {
	
// 1: support stackless collision trees for non-recursive collision queries
// 2: height field functionality not supported anymore
// 3: mass struct removed
// 4: bounding sphere removed
// 5: RTree added, opcode tree still in the binary image, physx 3.0
// 6: opcode tree removed from binary image
// 7: convex decomposition is out
// 8: adjacency information added
// 9: removed leaf triangles and most of opcode data, changed rtree layout
// 10: float rtrees
// 11: new build, isLeaf added to page
// 12: isLeaf is now the lowest bit in ptrs
// 13: TA30159 removed deprecated convexEdgeThreshold and bumped version
// 14: added midphase ID
// 15: GPU data simplification
// 16: vertex2Face mapping enabled by default if using GPU

#define PX_MESH_VERSION 16

// these flags are used to indicate/validate the contents of a cooked mesh file
enum InternalMeshSerialFlag
{
	IMSF_MATERIALS		=	(1<<0),	//!< if set, the cooked mesh file contains per-triangle material indices
	IMSF_FACE_REMAP		=	(1<<1),	//!< if set, the cooked mesh file contains a remap table
	IMSF_8BIT_INDICES	=	(1<<2),	//!< if set, the cooked mesh file contains 8bit indices (topology)
	IMSF_16BIT_INDICES	=	(1<<3),	//!< if set, the cooked mesh file contains 16bit indices (topology)
	IMSF_ADJACENCIES	=	(1<<4),	//!< if set, the cooked mesh file contains adjacency structures
	IMSF_GRB_DATA		=	(1<<5),	//!< if set, the cooked mesh file contains GRB data structures
	IMSF_SDF			=	(1<<6),	//!< if set, the cooked mesh file contains SDF data structures
	IMSF_VERT_MAPPING	=   (1<<7), //!< if set, the cooked mesh file contains vertex mapping information
	IMSF_GRB_INV_REMAP	=	(1<<8),	//!< if set, the cooked mesh file contains vertex inv mapping information. Required for cloth
	IMSF_INERTIA		=	(1<<9)	//!< if set, the cooked mesh file contains inertia tensor for the mesh
};

#if PX_VC
#pragma warning(push)
#pragma warning(disable: 4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	class MeshDataBase : public PxUserAllocated
	{
	public:
		PxMeshMidPhase::Enum	mType;
		PxU8					mFlags;
		PxU32					mNbVertices;
		PxVec3*					mVertices;

		PxReal					mMass;					//this is mass assuming a unit density that can be scaled by instances!
		PxMat33					mInertia;				//in local space of mesh!
		PxVec3					mLocalCenterOfMass;		//local space com

		PxBounds3				mAABB;
		PxReal					mGeomEpsilon;

		PxU32*					mFaceRemap;
		
		// GRB data -------------------------
		void*					mGRB_primIndices;		//!< GRB: GPU-friendly primitive indices(either triangle or tetrahedron)
		PxU32*					mGRB_faceRemap;			//!< GRB: this remap the GPU triangle indices to CPU triangle indices
		PxU32*					mGRB_faceRemapInverse;	//
		// End of GRB data ------------------

		// SDF data
		SDF						mSdfData;

		//Cloth data : each vert has a list of associated triangles in the mesh, this is for attachement constraints to enable default filtering
		PxU32*					mAccumulatedTrianglesRef;//runsum
		PxU32*					mTrianglesReferences;
		PxU32					mNbTrianglesReferences;

		MeshDataBase() :
			mFlags					(0),
			mNbVertices				(0),
			mVertices				(NULL),
			mMass					(0.f),
			mInertia				(PxZero),
			mLocalCenterOfMass		(0.f),
			mAABB					(PxBounds3::empty()),
			mGeomEpsilon			(0.0f),
			mFaceRemap				(NULL),
			mGRB_primIndices		(NULL),
			mGRB_faceRemap			(NULL),
			mGRB_faceRemapInverse	(NULL),
			mSdfData				(PxZero),
			mAccumulatedTrianglesRef(NULL),
			mTrianglesReferences	(NULL),
			mNbTrianglesReferences	(0)
		{
		}

		virtual ~MeshDataBase()
		{
			PX_FREE(mVertices);
			PX_FREE(mFaceRemap);
			PX_FREE(mGRB_primIndices);	
			PX_FREE(mGRB_faceRemap);
			PX_FREE(mGRB_faceRemapInverse);
			
			PX_FREE(mAccumulatedTrianglesRef);

			PX_FREE(mTrianglesReferences);
		}

		PX_NOINLINE PxVec3* allocateVertices(PxU32 nbVertices)
		{
			PX_ASSERT(!mVertices);
			// PT: we allocate one more vertex to make sure it's safe to V4Load the last one
			const PxU32 nbAllocatedVerts = nbVertices + 1;
			mVertices = PX_ALLOCATE(PxVec3, nbAllocatedVerts, "PxVec3");
			mNbVertices = nbVertices;
			return mVertices;
		}

		PX_FORCE_INLINE	bool	has16BitIndices()	const
		{
			return (mFlags & PxTriangleMeshFlag::e16_BIT_INDICES) ? true : false;
		}
	};

	class TriangleMeshData : public MeshDataBase
	{
		public:
		
		PxU32			mNbTriangles;
		void*			mTriangles;

		PxU32*			mAdjacencies;
		PxU8*			mExtraTrigData;
		PxU16*			mMaterialIndices;

		// GRB data -------------------------
		void*			mGRB_primAdjacencies;	//!< GRB: adjacency data, with BOUNDARY and NONCONVEX flags (flags replace adj indices where applicable) [uin4]
		Gu::BV32Tree*	mGRB_BV32Tree;
		// End of GRB data ------------------

		TriangleMeshData() :
			mNbTriangles			(0),
			mTriangles				(NULL),
			mAdjacencies			(NULL),
			mExtraTrigData			(NULL),
			mMaterialIndices		(NULL),
			mGRB_primAdjacencies	(NULL),
			mGRB_BV32Tree			(NULL)
		{
		}

		virtual ~TriangleMeshData()
		{
			PX_FREE(mTriangles);
			PX_FREE(mAdjacencies);
			PX_FREE(mMaterialIndices);
			PX_FREE(mExtraTrigData);
			PX_FREE(mGRB_primAdjacencies);
			PX_DELETE(mGRB_BV32Tree);
		}

		PX_NOINLINE PxU32* allocateAdjacencies()
		{
			PX_ASSERT(mNbTriangles);
			PX_ASSERT(!mAdjacencies);
			mAdjacencies = PX_ALLOCATE(PxU32, mNbTriangles * 3, "mAdjacencies");
			mFlags |= PxTriangleMeshFlag::eADJACENCY_INFO;
			return mAdjacencies;
		}

		PX_NOINLINE PxU32* allocateFaceRemap()
		{
			PX_ASSERT(mNbTriangles);
			PX_ASSERT(!mFaceRemap);
			mFaceRemap = PX_ALLOCATE(PxU32, mNbTriangles, "mFaceRemap");
			return mFaceRemap;
		}

		PX_NOINLINE void* allocateTriangles(PxU32 nbTriangles, bool force32Bit, PxU32 allocateGPUData = 0)
		{
			PX_ASSERT(mNbVertices);
			PX_ASSERT(!mTriangles);

			bool index16 = mNbVertices <= 0xffff && !force32Bit;
			if(index16)
				mFlags |= PxTriangleMeshFlag::e16_BIT_INDICES;

			mTriangles = PX_ALLOC(nbTriangles * (index16 ? sizeof(PxU16) : sizeof(PxU32)) * 3, "mTriangles");
			if (allocateGPUData)
				mGRB_primIndices = PX_ALLOC(nbTriangles * (index16 ? sizeof(PxU16) : sizeof(PxU32)) * 3, "mGRB_triIndices");
			mNbTriangles = nbTriangles;
			return mTriangles;
		}

		PX_NOINLINE PxU16* allocateMaterials()
		{
			PX_ASSERT(mNbTriangles);
			PX_ASSERT(!mMaterialIndices);
			mMaterialIndices = PX_ALLOCATE(PxU16, mNbTriangles, "mMaterialIndices");
			return mMaterialIndices;
		}

		PX_NOINLINE PxU8* allocateExtraTrigData()
		{
			PX_ASSERT(mNbTriangles);
			PX_ASSERT(!mExtraTrigData);
			mExtraTrigData = PX_ALLOCATE(PxU8, mNbTriangles, "mExtraTrigData");
			return mExtraTrigData;
		}

		PX_FORCE_INLINE void	setTriangleAdjacency(PxU32 triangleIndex, PxU32 adjacency, PxU32 offset)
		{
			PX_ASSERT(mAdjacencies); 
			mAdjacencies[triangleIndex*3 + offset] = adjacency; 
		}
	};

	class RTreeTriangleData : public TriangleMeshData
	{
		public:
								RTreeTriangleData()		{ mType = PxMeshMidPhase::eBVH33; }
		virtual					~RTreeTriangleData()	{}

				Gu::RTree		mRTree;
	};

	class BV4TriangleData : public TriangleMeshData
	{
		public:
								BV4TriangleData()	{ mType = PxMeshMidPhase::eBVH34;	}
		virtual					~BV4TriangleData()	{}

				Gu::SourceMesh	mMeshInterface;
				Gu::BV4Tree		mBV4Tree;
	};

#if PX_VC
#pragma warning(pop)
#endif


} // namespace Gu

}

#endif // #ifdef GU_MESH_DATA_H
