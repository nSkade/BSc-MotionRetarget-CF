/*****************************************************************************\
*                                                                           *
* File(s): SkeletalAnimationController.h and SkeletalAnimationController.cpp*
*                                                                           *
* Content:    *
*          .                                         *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_SKELETALANIMATIONCONTROLLER_H__
#define __CFORGE_SKELETALANIMATIONCONTROLLER_H__

#include "../../AssetIO/T3DMesh.hpp"
#include "../UniformBufferObjects/UBOBoneData.h"
#include "../Shader/ShaderCode.h"
#include "../Shader/GLShader.h"

namespace CForge {
	class CFORGE_API SkeletalAnimationController: public CForgeObject {
	public:
		struct Animation {
			int32_t AnimationID;   // m_SkeletalAnimations index
			float Speed;           // playback speed
			float Duration;
			float t;               // current timestep
			float SamplesPerSecond;
			int64_t LastTimestamp;
			bool Finished;
		};

		struct SkeletalJoint : public CForgeObject {
			int32_t ID;
			std::string Name;
			Eigen::Matrix4f OffsetMatrix;
			Eigen::Vector3f LocalPosition;
			Eigen::Quaternionf LocalRotation;
			Eigen::Vector3f LocalScale;
			Eigen::Matrix4f SkinningMatrix;

			int32_t Parent;
			std::vector<int32_t> Children;

			SkeletalJoint(void) : CForgeObject("SkeletalAnimationController::SkeletalJoint") {
				ID = -1;
				Parent = -1;
			}
		};

		SkeletalAnimationController(void);
		~SkeletalAnimationController(void);

		// pMesh has to hold skeletal definition
		void init(T3DMesh<float>* pMesh, bool CopyAnimationData = true);
		void update();
		void update(float FPSScale);
		void clear(void);

		void addAnimationData(T3DMesh<float>::SkeletalAnimation* pAnimation);

		Animation* createAnimation(int32_t AnimationID, float Speed, float Offset);
		void destroyAnimation(Animation* pAnim);
		void applyAnimation(Animation* pAnim, bool UpdateUBO = true);

		UBOBoneData* ubo(void);

		T3DMesh<float>::SkeletalAnimation* animation(uint32_t ID);
		uint32_t animationCount(void)const;

		GLShader* shadowPassShader(void);

		UBOBoneData* boneUBO(void);
		void retrieveSkinningMatrices(std::vector<Eigen::Matrix4f>* pSkinningMats);

		std::vector<SkeletalJoint*> retrieveSkeleton(void)const;
		void retrieveSkeleton(std::vector<SkeletalJoint*>* pSkeleton);
		void setSkeletonValues(std::vector<SkeletalJoint*>* pSkeleton, bool UpdateUBO = true);

		Eigen::Vector3f transformVertex(Eigen::Vector3f V, Eigen::Vector4i BoneInfluences, Eigen::Vector4f BoneWeights);

	protected:

		void transformSkeleton(SkeletalJoint* pJoint, Eigen::Matrix4f ParentTransform);
		int32_t jointIDFromName(std::string JointName);

		SkeletalJoint* m_pRoot;
		std::vector<SkeletalJoint*> m_Joints;
		
		std::vector<T3DMesh<float>::SkeletalAnimation*> m_SkeletalAnimations; // available animations for this skeleton
		std::vector<Animation*> m_ActiveAnimations;

		UBOBoneData m_UBO;
		GLShader *m_pShadowPassShader;
		ShaderCode* m_pShadowPassVSCode;
		ShaderCode* m_pShadowPassFSCode;

		std::string m_GLSLVersionTag;
		std::string m_GLSLPrecisionTag;

	};//SkeletalAnimationController

}//name space

#endif 