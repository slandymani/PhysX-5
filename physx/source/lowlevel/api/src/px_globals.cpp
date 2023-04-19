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

#include "PxvGlobals.h"
#include "PxsContext.h"
#include "PxcContactMethodImpl.h"
#include "GuContactMethodImpl.h"

namespace physx
{

PxvOffsetTable gPxvOffsetTable;

void PxvInit(const PxvOffsetTable& offsetTable)
{
	gPxvOffsetTable = offsetTable;
}

void PxvTerm()
{
}

}

#include "common/PxMetaData.h"
#include "PxsMaterialCore.h"

namespace physx
{

template<> void PxsMaterialCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxCombineMode::Enum, PxU32)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxMaterialFlags, PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,	PxsMaterialCore)

	// MaterialData
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxReal,			dynamicFriction,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxReal,			staticFriction,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxReal,			restitution,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxReal,			damping,			0)

	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxMaterialFlags,	flags,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxU8,				fricCombineMode,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxU8,				restCombineMode,	0)

	// MaterialCore
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxMaterial,		mMaterial,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxU16,				mMaterialIndex,		PxMetaDataFlag::eHANDLE)
}

}

