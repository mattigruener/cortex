//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2018, Image Engine Design Inc. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are
//  met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of Image Engine Design nor the names of any
//       other contributors to this software may be used to endorse or
//       promote products derived from this software without specific prior
//       written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
//  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////
#ifndef IE_COREMAYA_SCENESHAPESUBSCENEOVERRIDE_H
#define IE_COREMAYA_SCENESHAPESUBSCENEOVERRIDE_H

#include <bitset>
#include <map>

#include "IECore/MurmurHash.h"
#include "IECore/InternedString.h"
#include "IECore/RefCounted.h"

#include "IECoreScene/SceneInterface.h"
#include "IECoreScene/MeshPrimitive.h"

#include "IECoreMaya/Export.h"
#include "IECoreMaya/SceneShape.h"

#include "maya/MPxSubSceneOverride.h"
#include "maya/MSelectionContext.h"

namespace
{
}

namespace IECoreMaya
{

enum class RenderStyle { BoundingBox, Wireframe, Solid, Textured, Last };

using RenderItemMap = std::map<const std::string, std::pair<MHWRender::MRenderItem*, MMatrixArray> >;

class IECOREMAYA_API SceneShapeSubSceneOverride : public MHWRender::MPxSubSceneOverride
{
	public :

		static MString& drawDbClassification();
		static MString& drawDbId();
		static MHWRender::MPxSubSceneOverride* Creator( const MObject& obj );

		~SceneShapeSubSceneOverride() override;

		// Maya calls this to determine if `update` needs to be called at all for this refresh. Gets called a lot.
		bool requiresUpdate(const MHWRender::MSubSceneContainer& container, const MHWRender::MFrameContext& frameContext) const override;

		// Performing the actual updating. Needs to fill given container with MRenderItem objects for drawing.
		void update(MHWRender::MSubSceneContainer&  container, const MHWRender::MFrameContext& frameContext) override;

		// We are responsible for drawing all instances. Maya therefore refers to
		// us for figuring out which instance was selected when the user clicks
		// on one of our MRenderItems.
		bool getInstancedSelectionPath( const MHWRender::MRenderItem &renderItem, const MHWRender::MIntersection &intersection, MDagPath &dagPath ) const override;

		// Maya allows us to switch between object and component selection by changing the MSelectionContext.
		void updateSelectionGranularity( const MDagPath &path, MHWRender::MSelectionContext &selectionContext ) override;

		MHWRender::DrawAPI supportedDrawAPIs() const override;

	protected :

		SceneShapeSubSceneOverride( const MObject& obj );

	private :

		struct Instance
		{
			Instance( Imath::M44d transformation, bool selected, bool componentMode, MDagPath path )
				: transformation( transformation ), selected( selected ), componentMode( componentMode ), path( path )
			{
			}

			bool operator==( const Instance &rhs ) const
			{
				return transformation == rhs.transformation && selected == rhs.selected && path == rhs.path && componentMode == rhs.componentMode;
			}

			Imath::M44d transformation;
			bool selected;
			bool componentMode;
			MDagPath path;
		};

		using Instances = std::vector<Instance>;

		// Traverse the scene and create MRenderItems as necessary while collecting all matrices to be associated with them.
		void visitSceneLocations( IECoreScene::ConstSceneInterfacePtr sceneInterface, RenderItemMap &renderItems, MHWRender::MSubSceneContainer &container, const Imath::M44d &matrix, bool isRoot = false );

		// Provide information about the instances that need drawing as
		// SubSceneOverrides are responsible for drawing all instances of the
		// shape, which is different to how things were handled in Maya's VP1.
		void gatherInstanceInformation( Instances &instances, bool globalWireframeDrawing ) const;

		// Retrieve global display settings (can be locally overridden by instances)
		bool renderAllBounds( unsigned int displayStyle ) const;
		bool renderAllWireframes( unsigned int displayStyle ) const;
		bool renderAllShaded( unsigned int displayStyle ) const;

		// Get all component indices that are currently selected. Useful for
		// determining both selection changes and rendering states of these
		// components.
		void selectedComponentIndices( std::set<int> &selectedComponents ) const;

		SceneShape *m_sceneShape;

		std::string m_drawTagsFilter;
		double m_time;

		using StyleMask = std::bitset<3>;
		StyleMask m_styleMask; // \todo: now that things are simpler, consider replacing this with three bools.
		Instances m_instances;

		bool m_drawRootBounds;
		MPlug m_shaderOutPlug;
		bool m_materialIsDirty;
		bool m_instancedRendering;
		IECoreScene::ConstSceneInterfacePtr m_sceneInterface;

		std::map<const std::string, MDagPath> m_renderItemNameToDagPath;
		std::set<int> m_selectedComponents;
};

} // namespace IECoreMaya

#endif // IE_COREMAYA_SCENESHAPESUBSCENEOVERRIDE_H
