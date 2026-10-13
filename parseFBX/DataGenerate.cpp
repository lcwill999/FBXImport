#include "DataGenerate.h"
#include "Utility.h"
#include "FBXImporterDef.h"
#include "AnimationKeyFrameReducer.h"
#include <iostream>
#include <fstream>
#include <limits>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <Windows.h>

// 全局变量定义
std::map<const FBXImportNode*, std::string> gNodePtr2Path;
std::map<std::string, MeshBoneRef> gMeshBoneRefs;
std::map<std::string, std::string> gBlendShapeMesh2Bone;
std::string gFBXFileName = "";
std::string gOutPutDir;
std::string gOutPutDirSantinized;
bool gOutPutTangent = false;
bool gAnimUseFBXName = false;

bool ShouldAbsorbSceneWrapper(const FBXImportScene& scene)
{
	return scene.nodes.size() == 1 && !scene.sceneInfo.hasSkeleton;
}

void RegisterNodePath(FBXImportNode& node, const std::string& nodePath)
{
	gNodePtr2Path[&node] = nodePath;
}

void BuildBoneNameMap(FBXImportNode& node, std::string parentpath)
{
	std::string NodePath = node.name;
	if (parentpath != "")
		NodePath = parentpath + "/" + node.name;
	RegisterNodePath(node, NodePath);
	for (auto i = 0; i < node.children.size(); i++)
	{
		BuildBoneNameMap(node.children[i], NodePath);
	}
}

void BuildAllBoneNameMap(FBXImportScene& scene)
{
	gNodePtr2Path.clear();
	if (!ShouldAbsorbSceneWrapper(scene) || scene.nodes.empty())
	{
		for (auto i = 0; i < scene.nodes.size(); i++)
			BuildBoneNameMap(scene.nodes[i]);
		return;
	}

	RegisterNodePath(scene.nodes[0], "");
	for (auto i = 0; i < scene.nodes[0].children.size(); i++)
		BuildBoneNameMap(scene.nodes[0].children[i], "");
}

std::string ResolveAnimNodePath(const FBXImportNode* node)
{
	if (node == nullptr)
		return "";
	auto found = gNodePtr2Path.find(node);
	if (found != gNodePtr2Path.end())
		return found->second;
	return node->name;
}

bool ShouldWriteAnimNode(const FBXImportNode* node)
{
	return !ResolveAnimNodePath(node).empty();
}

static Vector3f RotateVectorByQuat(const Quaternionf& q, const Vector3f& v)
{
	Quaternionf p(v.x, v.y, v.z, 0.0f);
	Quaternionf inv(-q.x, -q.y, -q.z, q.w);
	Quaternionf r = q * p * inv;
	return Vector3f(r.x, r.y, r.z);
}

static Quaternionf CanonicalQuat(const Quaternionf& q)
{
	return q.w < 0.0f ? -q : q;
}

static void ComposeWrapperRestIntoChild(const FBXImportNode& wrapper, const Vector3f& childPos, const Quaternionf& childRot, const Vector3f& childScale, Vector3f& outPos, Quaternionf& outRot, Vector3f& outScale)
{
	const Quaternionf wrapperRot = CanonicalQuat(wrapper.rotation);
	Vector3f scaled(childPos.x * wrapper.scale.x, childPos.y * wrapper.scale.y, childPos.z * wrapper.scale.z);
	Vector3f rotated = RotateVectorByQuat(wrapperRot, scaled);
	outPos = Vector3f(rotated.x + wrapper.position.x, rotated.y + wrapper.position.y, rotated.z + wrapper.position.z);
	outRot = wrapperRot * childRot;
	outScale = Vector3f(wrapper.scale.x * childScale.x, wrapper.scale.y * childScale.y, wrapper.scale.z * childScale.z);
}

static void ComposeWrapperRestIntoChild(const FBXImportNode& wrapper, const FBXImportNode& child, Vector3f& outPos, Quaternionf& outRot, Vector3f& outScale)
{
	ComposeWrapperRestIntoChild(wrapper, child.position, child.rotation, child.scale, outPos, outRot, outScale);
}

void AttachSceneNodesToSkeleton(FBXImportScene& scene, message::UGCResBoneNodeData* root)
{
	if (ShouldAbsorbSceneWrapper(scene))
	{
		RegisterNodePath(scene.nodes[0], "");
		for (auto i = 0; i < scene.nodes[0].children.size(); i++)
		{
			Vector3f pos;
			Quaternionf rot;
			Vector3f scl;
			ComposeWrapperRestIntoChild(scene.nodes[0], scene.nodes[0].children[i], pos, rot, scl);
			BuildBoneNodeData(scene, scene.nodes[0].children[i], root, "", pos, rot, scl);
		}
		return;
	}

	for (auto i = 0; i < scene.nodes.size(); i++)
		BuildBoneNodeData(scene, scene.nodes[i], root, "");
}

static bool IsWrapperRestIdentity(const FBXImportNode& wrapper)
{
	const Quaternionf wrapperRot = CanonicalQuat(wrapper.rotation);
	const bool posIdentity =
		CompareApproximately(wrapper.position.x, 0.0f) &&
		CompareApproximately(wrapper.position.y, 0.0f) &&
		CompareApproximately(wrapper.position.z, 0.0f);
	const bool scaleIdentity =
		CompareApproximately(wrapper.scale.x, 1.0f) &&
		CompareApproximately(wrapper.scale.y, 1.0f) &&
		CompareApproximately(wrapper.scale.z, 1.0f);
	const bool rotIdentity =
		CompareApproximately(wrapperRot.x, 0.0f) &&
		CompareApproximately(wrapperRot.y, 0.0f) &&
		CompareApproximately(wrapperRot.z, 0.0f) &&
		CompareApproximately(wrapperRot.w, 1.0f);
	return posIdentity && scaleIdentity && rotIdentity;
}

static bool CurvesShareKeyTimes(const FBXAnimationCurve* curves, int count)
{
	if (count <= 0 || curves[0].GetKeyCount() <= 0)
		return false;
	const int n = curves[0].GetKeyCount();
	for (int i = 1; i < count; ++i)
	{
		if (curves[i].GetKeyCount() != n)
			return false;
		for (int k = 0; k < n; ++k)
		{
			if (curves[i].GetKey(k).time != curves[0].GetKey(k).time)
				return false;
		}
	}
	return true;
}

static float EvalCurveOr(const FBXAnimationCurve& curve, float time, float fallback)
{
	if (curve.GetKeyCount() <= 0)
		return fallback;
	return curve.Evaluate(time);
}

static void CollectCurveTimes(const FBXAnimationCurve* curves, int count, std::set<float>& times)
{
	for (int i = 0; i < count; ++i)
	{
		for (int k = 0; k < curves[i].GetKeyCount(); ++k)
			times.insert(curves[i].GetKey(k).time);
	}
}

static void RebuildCurveAtTimes(FBXAnimationCurve& curve, const std::vector<float>& times, const std::vector<float>& values)
{
	InternalWrapMode pre = curve.preInfinity;
	InternalWrapMode post = curve.postInfinity;
	RotationOrder rotOrder = curve.GetRotationOrder();
	curve.m_Curve.clear();
	curve.InvalidateCache();
	curve.preInfinity = pre;
	curve.postInfinity = post;
	curve.SetRotationOrder(rotOrder);

	const int n = (int)times.size();
	for (int i = 0; i < n; ++i)
	{
		FBXKeyFrameTpl<float> key;
		key.time = times[i];
		key.value = values[i];
		key.weightedMode = kNotWeighted;
		key.inWeight = 1.0f / 3.0f;
		key.outWeight = 1.0f / 3.0f;
		key.inSlope = 0.0f;
		key.outSlope = 0.0f;
		curve.AddKeyBackFast(key);
	}

	if (n < 2)
		return;
	for (int i = 0; i < n; ++i)
	{
		float m = 0.0f;
		if (i == 0)
			m = (values[1] - values[0]) / (times[1] - times[0]);
		else if (i == n - 1)
			m = (values[n - 1] - values[n - 2]) / (times[n - 1] - times[n - 2]);
		else
		{
			const float dx1 = times[i] - times[i - 1];
			const float dx2 = times[i + 1] - times[i];
			const float m1 = Abs(dx1) > 0.00001f ? (values[i] - values[i - 1]) / dx1 : 0.0f;
			const float m2 = Abs(dx2) > 0.00001f ? (values[i + 1] - values[i]) / dx2 : 0.0f;
			m = 0.5f * m1 + 0.5f * m2;
		}
		curve.GetKey(i).inSlope = m;
		curve.GetKey(i).outSlope = m;
	}
	curve.InvalidateCache();
}

static void BakeSharedTimeRotation(FBXAnimationCurve rot[4], const Quaternionf& wrapperRot)
{
	const int n = rot[0].GetKeyCount();
	Quaternionf prev = Quaternionf::identity();
	for (int k = 0; k < n; ++k)
	{
		Quaternionf q(rot[0].GetKey(k).value, rot[1].GetKey(k).value, rot[2].GetKey(k).value, rot[3].GetKey(k).value);
		Quaternionf qIn(rot[0].GetKey(k).inSlope, rot[1].GetKey(k).inSlope, rot[2].GetKey(k).inSlope, rot[3].GetKey(k).inSlope);
		Quaternionf qOut(rot[0].GetKey(k).outSlope, rot[1].GetKey(k).outSlope, rot[2].GetKey(k).outSlope, rot[3].GetKey(k).outSlope);

		Quaternionf baked = NormalizeSafe(wrapperRot * q);
		Quaternionf bakedIn = wrapperRot * qIn;
		Quaternionf bakedOut = wrapperRot * qOut;
		if (k > 0 && Dot(baked, prev) < 0.0f)
		{
			baked = -baked;
			bakedIn = -bakedIn;
			bakedOut = -bakedOut;
		}
		prev = baked;

		const float bakedComp[4] = { baked.x, baked.y, baked.z, baked.w };
		const float bakedInComp[4] = { bakedIn.x, bakedIn.y, bakedIn.z, bakedIn.w };
		const float bakedOutComp[4] = { bakedOut.x, bakedOut.y, bakedOut.z, bakedOut.w };
		for (int i = 0; i < 4; ++i)
		{
			rot[i].GetKey(k).value = bakedComp[i];
			rot[i].GetKey(k).inSlope = bakedInComp[i];
			rot[i].GetKey(k).outSlope = bakedOutComp[i];
		}
	}
	for (int i = 0; i < 4; ++i)
		rot[i].InvalidateCache();
}

static void BakeSharedTimeTranslation(FBXAnimationCurve trans[3], const FBXImportNode& wrapper)
{
	const Quaternionf wrapperRot = CanonicalQuat(wrapper.rotation);
	const int n = trans[0].GetKeyCount();
	for (int k = 0; k < n; ++k)
	{
		Vector3f local(trans[0].GetKey(k).value, trans[1].GetKey(k).value, trans[2].GetKey(k).value);
		Vector3f inS(trans[0].GetKey(k).inSlope, trans[1].GetKey(k).inSlope, trans[2].GetKey(k).inSlope);
		Vector3f outS(trans[0].GetKey(k).outSlope, trans[1].GetKey(k).outSlope, trans[2].GetKey(k).outSlope);

		Quaternionf dummyQ;
		Vector3f dummyS;
		Vector3f bakedPos;
		ComposeWrapperRestIntoChild(wrapper, local, Quaternionf::identity(), Vector3f(1.0f, 1.0f, 1.0f), bakedPos, dummyQ, dummyS);
		Vector3f bakedIn = RotateVectorByQuat(wrapperRot, Vector3f(inS.x * wrapper.scale.x, inS.y * wrapper.scale.y, inS.z * wrapper.scale.z));
		Vector3f bakedOut = RotateVectorByQuat(wrapperRot, Vector3f(outS.x * wrapper.scale.x, outS.y * wrapper.scale.y, outS.z * wrapper.scale.z));

		for (int i = 0; i < 3; ++i)
		{
			trans[i].GetKey(k).value = bakedPos[i];
			trans[i].GetKey(k).inSlope = bakedIn[i];
			trans[i].GetKey(k).outSlope = bakedOut[i];
		}
	}
	for (int i = 0; i < 3; ++i)
		trans[i].InvalidateCache();
}

static void BakeSharedTimeScale(FBXAnimationCurve scale[3], const Vector3f& wrapperScale)
{
	const int n = scale[0].GetKeyCount();
	for (int k = 0; k < n; ++k)
	{
		for (int i = 0; i < 3; ++i)
		{
			scale[i].GetKey(k).value *= wrapperScale[i];
			scale[i].GetKey(k).inSlope *= wrapperScale[i];
			scale[i].GetKey(k).outSlope *= wrapperScale[i];
		}
	}
	for (int i = 0; i < 3; ++i)
		scale[i].InvalidateCache();
}

static void BakeRotationFallback(FBXAnimationCurve rot[4], const Quaternionf& wrapperRot, const Quaternionf& restRot)
{
	std::set<float> rotTimes;
	CollectCurveTimes(rot, 4, rotTimes);
	if (rotTimes.empty())
		return;

	std::vector<float> times(rotTimes.begin(), rotTimes.end());
	std::vector<float> baked[4];
	for (int i = 0; i < 4; ++i)
		baked[i].reserve(times.size());

	Quaternionf prev = Quaternionf::identity();
	for (size_t i = 0; i < times.size(); ++i)
	{
		Quaternionf local(
			EvalCurveOr(rot[0], times[i], restRot.x),
			EvalCurveOr(rot[1], times[i], restRot.y),
			EvalCurveOr(rot[2], times[i], restRot.z),
			EvalCurveOr(rot[3], times[i], restRot.w));
		Quaternionf composed = NormalizeSafe(wrapperRot * local);
		if (i > 0 && Dot(composed, prev) < 0.0f)
			composed = -composed;
		prev = composed;
		baked[0].push_back(composed.x);
		baked[1].push_back(composed.y);
		baked[2].push_back(composed.z);
		baked[3].push_back(composed.w);
	}
	for (int i = 0; i < 4; ++i)
		RebuildCurveAtTimes(rot[i], times, baked[i]);
}

static void BakeTranslationFallback(FBXAnimationCurve trans[3], const FBXImportNode& wrapper, const Vector3f& restPos)
{
	std::set<float> transTimes;
	CollectCurveTimes(trans, 3, transTimes);
	if (transTimes.empty())
		return;

	std::vector<float> times(transTimes.begin(), transTimes.end());
	std::vector<float> baked[3];
	for (int i = 0; i < 3; ++i)
		baked[i].reserve(times.size());
	for (size_t i = 0; i < times.size(); ++i)
	{
		Vector3f local(
			EvalCurveOr(trans[0], times[i], restPos.x),
			EvalCurveOr(trans[1], times[i], restPos.y),
			EvalCurveOr(trans[2], times[i], restPos.z));
		Vector3f composedPos;
		Quaternionf dummyQ;
		Vector3f dummyS;
		ComposeWrapperRestIntoChild(wrapper, local, Quaternionf::identity(), Vector3f(1.0f, 1.0f, 1.0f), composedPos, dummyQ, dummyS);
		baked[0].push_back(composedPos.x);
		baked[1].push_back(composedPos.y);
		baked[2].push_back(composedPos.z);
	}
	for (int i = 0; i < 3; ++i)
		RebuildCurveAtTimes(trans[i], times, baked[i]);
}

static void BakeScaleFallback(FBXAnimationCurve scale[3], const Vector3f& wrapperScale, const Vector3f& restScale)
{
	std::set<float> scaleTimes;
	CollectCurveTimes(scale, 3, scaleTimes);
	if (scaleTimes.empty())
		return;

	std::vector<float> times(scaleTimes.begin(), scaleTimes.end());
	std::vector<float> baked[3];
	for (int i = 0; i < 3; ++i)
		baked[i].reserve(times.size());
	for (size_t i = 0; i < times.size(); ++i)
	{
		baked[0].push_back(EvalCurveOr(scale[0], times[i], restScale.x) * wrapperScale.x);
		baked[1].push_back(EvalCurveOr(scale[1], times[i], restScale.y) * wrapperScale.y);
		baked[2].push_back(EvalCurveOr(scale[2], times[i], restScale.z) * wrapperScale.z);
	}
	for (int i = 0; i < 3; ++i)
		RebuildCurveAtTimes(scale[i], times, baked[i]);
}

static void BakeWrapperRestIntoNodeAnim(const FBXImportNode& wrapper, FBXImportNodeAnimation& anim)
{
	if (anim.node == nullptr)
		return;

	const Quaternionf wrapperRot = CanonicalQuat(wrapper.rotation);
	const bool rotIdentity =
		CompareApproximately(wrapperRot.x, 0.0f) &&
		CompareApproximately(wrapperRot.y, 0.0f) &&
		CompareApproximately(wrapperRot.z, 0.0f) &&
		CompareApproximately(wrapperRot.w, 1.0f);
	const bool scaleIdentity =
		CompareApproximately(wrapper.scale.x, 1.0f) &&
		CompareApproximately(wrapper.scale.y, 1.0f) &&
		CompareApproximately(wrapper.scale.z, 1.0f);
	const bool posIdentity =
		CompareApproximately(wrapper.position.x, 0.0f) &&
		CompareApproximately(wrapper.position.y, 0.0f) &&
		CompareApproximately(wrapper.position.z, 0.0f);

	if (!rotIdentity)
	{
		if (CurvesShareKeyTimes(anim.rotation, 4))
			BakeSharedTimeRotation(anim.rotation, wrapperRot);
		else
			BakeRotationFallback(anim.rotation, wrapperRot, anim.node->rotation);
	}

	if (!(posIdentity && rotIdentity && scaleIdentity))
	{
		if (CurvesShareKeyTimes(anim.translation, 3))
			BakeSharedTimeTranslation(anim.translation, wrapper);
		else
			BakeTranslationFallback(anim.translation, wrapper, anim.node->position);
	}

	if (!scaleIdentity)
	{
		if (CurvesShareKeyTimes(anim.scale, 3))
			BakeSharedTimeScale(anim.scale, wrapper.scale);
		else
			BakeScaleFallback(anim.scale, wrapper.scale, anim.node->scale);
	}
}

void BakeAbsorbedWrapperRestIntoDirectChildClips(FBXImportScene& scene)
{
	if (!ShouldAbsorbSceneWrapper(scene))
		return;
	if (IsWrapperRestIdentity(scene.nodes[0]))
		return;

	std::set<const FBXImportNode*> directChildren;
	for (auto i = 0; i < scene.nodes[0].children.size(); i++)
		directChildren.insert(&scene.nodes[0].children[i]);

	for (auto clip = scene.animationClips.begin(); clip != scene.animationClips.end(); ++clip)
	{
		for (auto it = clip->nodeAnimations.begin(); it != clip->nodeAnimations.end(); ++it)
		{
			if (it->node != nullptr && directChildren.find(it->node) != directChildren.end())
				BakeWrapperRestIntoNodeAnim(scene.nodes[0], *it);
		}
	}
}

void BuildMeshBoneRefMap(FBXImportScene& scene)
{
	gMeshBoneRefs.clear();
	auto meshes = scene.meshes;
	auto scale = scene.fileScaleFactor;
	for (auto i = 0; i < meshes.size(); i++)
	{
		auto singleMeshBones = meshes[i].bones;
		MeshBoneRef meshBones;
		for (auto j = 0; j < singleMeshBones.size(); j++)
		{
			Matrix4x4f newPose = singleMeshBones[j].bindpose;
			newPose[12] *= scale;
			newPose[13] *= scale;
			newPose[14] *= scale;
			meshBones.boneNames.push_back(singleMeshBones[j].node->name);
			meshBones.bindPoses.push_back(newPose);
		}
		if (!meshBones.boneNames.empty())
			gMeshBoneRefs[meshes[i].name] = meshBones;
	}
}

void BuildBlendShapeBoneMap(FBXImportScene& scene)
{
	gBlendShapeMesh2Bone.clear();
	for (const auto& mesh : scene.meshes)
	{
		if (mesh.shapes.empty() || mesh.vertices.empty())
			continue;
		for (const auto& pair : gNodePtr2Path)
		{
			if (pair.first != nullptr && pair.first->name == mesh.name)
			{
				gBlendShapeMesh2Bone[mesh.name] = pair.first->name;
				break;
			}
		}
	}
}

void PrebuildFBXMeshForBlendShape(FBXMesh& meshData)
{
	if (meshData.vertices.size() > 0)
	{
		BoneWeights4 boneWeights = { {1.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0} };
		meshData.boneWeights.resize(meshData.vertices.size(), boneWeights);
		Matrix4x4f ident;
		ident.SetIdentity();
		meshData.bindPoses.push_back(ident);
	}
}

message::UGCResSkinnedMeshExtData BuildMeshExtData(FBXMesh& meshData)
{
	std::string meshName = meshData.name;
	message::UGCResSkinnedMeshExtData ext;
	auto foundSkin = gMeshBoneRefs.find(meshName);
	if (foundSkin != gMeshBoneRefs.end())
	{
		for (auto i = 0; i < foundSkin->second.boneNames.size(); i++)
		{
			ext.add_bonenames(foundSkin->second.boneNames[i]);
		}
	}
	auto foundBlend = gBlendShapeMesh2Bone.find(meshName);
	if (foundBlend != gBlendShapeMesh2Bone.end())
	{
		ext.add_bonenames(foundBlend->second);
		PrebuildFBXMeshForBlendShape(meshData);
	}
	return ext;
}

void BuildMeshTxt(FBXMesh& meshData, FBXImportScene& importScene, const char* outdir)
{
	std::string meshfilename(meshData.name);
	std::string directory(outdir);

	char tempPath[MAX_PATH];
	GetTempPathA(MAX_PATH, tempPath);
	
	char tempFileName[MAX_PATH];
	GetTempFileNameA(tempPath, "MTX", 0, tempFileName);
	std::string tempMeshFilename(tempFileName);
	
	std::ofstream osData(tempMeshFilename);
	osData.precision(8);
	osData << "MeshName: " << meshData.name << std::endl;

	osData << "*********************************************************" << std::endl;
	auto& bw = meshData.boneWeights;
	osData << "Mesh boneweight Count: " << bw.size() << std::endl;
	for (auto i = 0; i < bw.size(); i++)
	{
		auto currentbw = bw[i];
		osData << "BoneWeight No. " << i << " weight [ " << currentbw.weight[0] << ", " << currentbw.weight[1] << ", " << currentbw.weight[2] << ", " << currentbw.weight[3] << "]" << std::endl;
		osData << "BoneWeight No. " << i << " boneIndex [ " << currentbw.boneIndex[0] << ", " << currentbw.boneIndex[1] << ", " << currentbw.boneIndex[2] << ", " << currentbw.boneIndex[3] << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "Mesh bindPoses Count: " << meshData.bindPoses.size() << std::endl;
	for (auto i = 0; i < meshData.bindPoses.size(); i++)
	{
		auto mat = meshData.bindPoses[i];
		osData << "*********************************************************" << std::endl;
		osData << "[ " << mat[0] << " , " << mat[1] << " , " << mat[2]<< " , " << mat[3] <<" ]" << std::endl;
		osData << "[ " << mat[4] << " , " << mat[5] << " , " << mat[6] << " , " << mat[7] << " ]" << std::endl;
		osData << "[ " << mat[8] << " , " << mat[9] << " , " << mat[10] << " , " << mat[11] << " ]" << std::endl;
		osData << "[ " << mat[12] << " , " << mat[13] << " , " << mat[14] << " , " << mat[15] << " ]" << std::endl;
		osData << "*********************************************************" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "Vert Count : " << meshData.vertices.size() << std::endl;
	for (auto i = 0; i < meshData.vertices.size(); i++)
	{
		osData << "Vertex No. " << i << "[ " << meshData.vertices[i].x << ", " << meshData.vertices[i].y << ", " << meshData.vertices[i].z << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "Norl Count : " << meshData.normals.size() << std::endl;
	for (auto i = 0; i < meshData.normals.size(); i++)
	{
		osData << "Norl No. " << i << "[ " << meshData.normals[i].x << ", " << meshData.normals[i].y << ", " << meshData.normals[i].z << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "uv Count : " << meshData.uv1.size() << std::endl;
	for (auto i = 0; i < meshData.uv1.size(); i++)
	{
		osData << "UV1 No. " << i << "[ " << meshData.uv1[i].x << ", " << meshData.uv1[i].y << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData << "index Count : " << meshData.indices.size() << std::endl;
	for (auto i = 0; i < meshData.indices.size(); i++)
	{
		osData << "Index No. " << i << "[ " << meshData.indices[i] << "]" << std::endl;
	}
	osData << "*********************************************************" << std::endl;
	osData.close();

	std::string finalMeshFilename = directory + "\\" + meshfilename + ".txt";
	std::wstring meshfilenameW = ConvertUTF8ToWide(finalMeshFilename);
	RenameFileToWide(tempMeshFilename, meshfilenameW);
}

void BuildSingleMesh(FBXMesh& meshData, FBXImportScene& importScene, std::string& filename, const char* outdir, std::string& realPath)
{
	if (meshData.vertices.empty() || meshData.indices.empty())
	{
		std::cerr << "Error : Vertices or Indexes may be missing, skip mesh: " << (meshData.name ? meshData.name : "unknown") << std::endl;
		realPath.clear();
		return;
	}

	EnsureDirectoryExists(outdir);
	bool isSkinnedMesh = false;
	message::UGCResSkinnedMeshExtData extData = BuildMeshExtData(meshData);
	if (extData.bonenames_size() > 0)
	{
		isSkinnedMesh = true;
	}
	std::string meshfilename(meshData.name);
	std::string directory(outdir);
	
	char tempPath[MAX_PATH];
	GetTempPathA(MAX_PATH, tempPath);
	
	char tempFileName[MAX_PATH];
	GetTempFileNameA(tempPath, "FBX", 0, tempFileName);
	std::string tempFilename(tempFileName);
	
	std::string targetFilename;
	if (isSkinnedMesh) {
		targetFilename = directory + "\\" + meshfilename + ".~@FFSKIN";
	} else {
		targetFilename = directory + "\\" + meshfilename + ".~@FFFUB";
	}
	
	filename = targetFilename;

	std::ofstream osData(tempFilename, std::ios_base::out | std::ios_base::binary);

	if (!osData.is_open()) {
		char errorMessage[256] = { 0 };
		strerror_s(errorMessage, sizeof(errorMessage), errno);
		std::cerr << "Error: Failed to open temporary file at " << tempFilename << std::endl;
		std::cerr << "Reason: " << errorMessage << std::endl;
		return;
	}
	osData.precision(8);
	MeshBody body;
	BuildMeshHead(meshData, extData, body, osData);
	BuildMeshBody(meshData, body, osData);
	if (isSkinnedMesh)
		extData.SerializePartialToOstream(&osData);
	osData.close();
	
	std::string dstDirectory(gOutPutDir);
	realPath = SanitizeName(meshData.name);
	if (isSkinnedMesh)
		realPath = dstDirectory + "\\" + realPath + ".~@FFSKIN";
	else
		realPath = dstDirectory + "\\" + realPath + ".~@FFFUB";
	auto dstMeshfilenameW = ConvertUTF8ToWide(realPath);
	RenameFileToWide(tempFilename, dstMeshfilenameW);
#if DebugMeshInfoOutput
	BuildMeshTxt(meshData, importScene, outdir);
	ParseSingleMesh(realPath);
#endif
}

void BuildMeshHead(FBXMesh& meshData, message::UGCResSkinnedMeshExtData& extData, MeshBody& body, std::ofstream& osData)
{
	MeshHead head;
	body.NamePos = sizeof(MeshHead);
	body.NameHeadLength = 8;
	body.NameLength = strlen(meshData.name) + 1;

	body.VerticesPos += body.NamePos + body.NameHeadLength + body.NameLength;
	body.VerticesHeadLength = 8;
	body.VerticesLength = meshData.vertices.size();

	body.ColorPos += body.VerticesPos + body.VerticesHeadLength + body.VerticesLength * sizeof(Vector3f);
	body.ColorHeadLength = 8;
	body.ColorLength = meshData.colors.size();

	body.NormalPos += body.ColorPos + body.ColorHeadLength + body.ColorLength * sizeof(ColorRGBA32);
	body.NormalHeadLength = 8;
	body.NormalLength = meshData.normals.size();

	body.TangentPos += body.NormalPos + body.NormalHeadLength + body.NormalLength * sizeof(Vector3f);
	body.TangentHeadLength = 8;
	if (gOutPutTangent)
		body.TangentLength = meshData.tangents.size();
	else
		body.TangentLength = 0;

	body.UV1Pos += body.TangentPos + body.TangentHeadLength + body.TangentLength * sizeof(Vector4f);
	body.UV1HeadLength = 8;
	body.UV1Length = meshData.uv1.size();

	body.UV2Pos += body.UV1Pos + body.UV1HeadLength + body.UV1Length * sizeof(Vector2f);
	body.UV2HeadLength = 8;
	body.UV2Length = meshData.uv2.size();

	body.IndexSizePos += body.UV2Pos + body.UV2HeadLength + body.UV2Length * sizeof(Vector2f);
	body.IndexSizeHeadLength = 8;
	body.IndexSizeLength = meshData.indicesize.size();

	body.IndexPos += body.IndexSizePos + body.IndexSizeHeadLength + body.IndexSizeLength * sizeof(uint32_t);
	body.IndexHeadLength = 8;
	body.IndexLength = meshData.indices.size();

	body.MatPos += body.IndexPos + body.IndexHeadLength + body.IndexLength * sizeof(uint32_t);
	body.MatHeadLength = 8;
	body.MatLength = meshData.materialindex.size();

	body.BindPosesPos += body.MatPos + body.MatHeadLength + body.MatLength * sizeof(uint32_t);
	body.BindPosesHeadLength = 8;
	body.BindPosesLength = meshData.bindPoses.size();

	body.BoneWeightPos += body.BindPosesPos + body.BindPosesHeadLength + body.BindPosesLength * sizeof(Matrix4x4f);
	body.BoneWeightHeadLength = 8;
	body.BoneWeightLength = meshData.boneWeights.size();

	head.MeshDataStartPos = body.NamePos;
	head.MeshDataExtPos = body.BoneWeightPos + body.BoneWeightHeadLength + body.BoneWeightLength * sizeof(BoneWeights4);
	head.MeshDataSize = head.MeshDataExtPos - head.MeshDataStartPos;

	if (extData.bonenames_size() > 0)
	{
		head.MeshDataExtSize = extData.ByteSizeLong();
		head.MeshType = 1;
	}

	head.MagicNumber1 = 100;
	head.MagicNumber2 = 97;
	head.MagicNumber3 = 157;
	head.MagicNumber4 = 136;
	head.Version = 1009715;
	unsigned char* p = (unsigned char*)&head.Version;
	p[0] = 1;
	p[1] = 1;
	p[2] = 1;
	p[3] = 1;
	
	osData.write(reinterpret_cast<char*>(&head.MagicNumber1), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&head.MagicNumber2), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&head.MagicNumber3), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&head.MagicNumber4), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&head.Version), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshType), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshDataStartPos), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshDataSize), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshDataExtPos), sizeof(int));
	osData.write(reinterpret_cast<char*>(&head.MeshDataExtSize), sizeof(int));
}

void BuildMeshBody(FBXMesh& meshData, MeshBody& bodyinfo, std::ofstream& osData)
{
	osData.write(reinterpret_cast<char*>(&bodyinfo.NameLength), 8);
	osData.write(meshData.name, bodyinfo.NameLength);

	osData.write(reinterpret_cast<char*>(&bodyinfo.VerticesLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.vertices.data()), bodyinfo.VerticesLength * sizeof(Vector3f));

	osData.write(reinterpret_cast<char*>(&bodyinfo.ColorLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.colors.data()), bodyinfo.ColorLength * sizeof(ColorRGBA32));

	osData.write(reinterpret_cast<char*>(&bodyinfo.NormalLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.normals.data()), bodyinfo.NormalLength * sizeof(Vector3f));

	osData.write(reinterpret_cast<char*>(&bodyinfo.TangentLength), 8);
	if(gOutPutTangent)
		osData.write(reinterpret_cast<char*>(meshData.tangents.data()), bodyinfo.TangentLength * sizeof(Vector4f));

	osData.write(reinterpret_cast<char*>(&bodyinfo.UV1Length), 8);
	osData.write(reinterpret_cast<char*>(meshData.uv1.data()), bodyinfo.UV1Length * sizeof(Vector2f));

	osData.write(reinterpret_cast<char*>(&bodyinfo.UV2Length), 8);
	osData.write(reinterpret_cast<char*>(meshData.uv2.data()), bodyinfo.UV2Length * sizeof(Vector2f));

	osData.write(reinterpret_cast<char*>(&bodyinfo.IndexSizeLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.indicesize.data()), bodyinfo.IndexSizeLength * sizeof(uint32_t));

	osData.write(reinterpret_cast<char*>(&bodyinfo.IndexLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.indices.data()), bodyinfo.IndexLength * sizeof(uint32_t));

	osData.write(reinterpret_cast<char*>(&bodyinfo.MatLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.materialindex.data()), bodyinfo.MatLength * sizeof(uint32_t));

	osData.write(reinterpret_cast<char*>(&bodyinfo.BindPosesLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.bindPoses.data()), bodyinfo.BindPosesLength * sizeof(Matrix4x4f));

	osData.write(reinterpret_cast<char*>(&bodyinfo.BoneWeightLength), 8);
	osData.write(reinterpret_cast<char*>(meshData.boneWeights.data()), bodyinfo.BoneWeightLength * sizeof(BoneWeights4));
}

std::wstring ConvertToWide(const char* utf8Str) {
	if (utf8Str == nullptr) {
		return L"";
	}

	int sizeNeeded = MultiByteToWideChar(CP_UTF8, 0, utf8Str, -1, NULL, 0);
	if (sizeNeeded <= 0) {
		return L"";
	}

	std::wstring wideStr(sizeNeeded - 1, 0);
	MultiByteToWideChar(CP_UTF8, 0, utf8Str, -1, &wideStr[0], sizeNeeded);
	return wideStr;
}

std::wstring ConvertUTF8ToWide(const std::string& utf8Str) {
	return ConvertToWide(utf8Str.c_str());
}

std::wstring MultiByteToWide(const std::string& multiByteStr, UINT codePage) {
	int sizeNeeded = MultiByteToWideChar(codePage, 0, multiByteStr.c_str(), -1, NULL, 0);
	if (sizeNeeded <= 0) {
		return L"";
	}
	std::wstring wideStr(sizeNeeded - 1, 0);
	MultiByteToWideChar(codePage, 0, multiByteStr.c_str(), -1, &wideStr[0], sizeNeeded);
	return wideStr;
}

void EnsureDirectoryExists(const std::wstring& directoryPath) {
	DWORD fileAttributes = GetFileAttributesW(directoryPath.c_str());

	if (fileAttributes != INVALID_FILE_ATTRIBUTES) {
		if (fileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			return;
		}
		else {
			return;
		}
	}

	std::size_t pos = directoryPath.find_last_of(L"\\/");
	if (pos != std::wstring::npos) {
		EnsureDirectoryExists(directoryPath.substr(0, pos));
	}

	if (CreateDirectoryW(directoryPath.c_str(), NULL)) {
		// 成功创建目录
	}
	else {
		DWORD error = GetLastError();
		if (error == ERROR_ALREADY_EXISTS) {
			return;
		}
	}
}

void EnsureDirectoryExists(const std::string& directoryPath) {
	DWORD fileAttributes = GetFileAttributesA(directoryPath.c_str());

	if (fileAttributes != INVALID_FILE_ATTRIBUTES) {
		if (fileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			return;
		}
		else {
			return;
		}
	}

	std::size_t pos = directoryPath.find_last_of("\\/");
	if (pos != std::string::npos) {
		EnsureDirectoryExists(directoryPath.substr(0, pos));
	}

	if (CreateDirectoryA(directoryPath.c_str(), NULL)) {
		// 成功创建目录
	}
	else {
		DWORD error = GetLastError();
		if (error == ERROR_ALREADY_EXISTS) {
			return;
		}
	}
}

void RenameFileToWide(const std::string& originalName, const std::wstring& newName) {
	try {
		std::wstring wideOriginalName = ConvertUTF8ToWide(originalName);

		std::size_t pos = newName.find_last_of(L"\\/");
		if (pos != std::wstring::npos) {
			std::wstring targetDirectory = newName.substr(0, pos);
			EnsureDirectoryExists(targetDirectory);
		}

		if (MoveFileExW(wideOriginalName.c_str(), newName.c_str(), MOVEFILE_REPLACE_EXISTING)) {
			wprintf(L"File renamed successfully to: %ls\n", newName.c_str());
		}
		else {
			DWORD error = GetLastError();
			if (error == ERROR_NOT_SAME_DEVICE) {
				if (CopyFileW(wideOriginalName.c_str(), newName.c_str(), FALSE)) {
					if (DeleteFileW(wideOriginalName.c_str())) {
						wprintf(L"File copied and original deleted successfully to: %ls\n", newName.c_str());
					}
					else {
						wprintf(L"File copied but failed to delete original, error code: %d\n", GetLastError());
					}
				}
				else {
					DWORD copyError = GetLastError();
					wprintf(L"File copy failed, error code: %d\n", copyError);
					wprintf(L"  Source: %ls\n", wideOriginalName.c_str());
					wprintf(L"  Destination: %ls\n", newName.c_str());
				}
			}
			else {
				wprintf(L"File rename failed, error code: %d\n", error);
				wprintf(L"  Source: %ls\n", wideOriginalName.c_str());
				wprintf(L"  Destination: %ls\n", newName.c_str());
			}
		}
	}
	catch (const std::exception& e) {
		printf("Error: %s\n", e.what());
	}
}

bool IsSubPath(const std::wstring& sourcePath, const std::wstring& targetPath) {
	if (targetPath.find(sourcePath) == 0 && (targetPath[sourcePath.length()] == L'\\' || targetPath[sourcePath.length()] == L'/')) {
		return true;
	}
	return false;
}

bool IsDirectoryEmpty(const std::string& directoryPath) {
	WIN32_FIND_DATAA findFileData;
	std::string searchPath = directoryPath + "\\*";
	HANDLE hFind = FindFirstFileA(searchPath.c_str(), &findFileData);

	if (hFind == INVALID_HANDLE_VALUE) {
		return false;
	}

	bool isEmpty = true;
	do {
		if (strcmp(findFileData.cFileName, ".") != 0 && strcmp(findFileData.cFileName, "..") != 0) {
			isEmpty = false;
			break;
		}
	} while (FindNextFileA(hFind, &findFileData) != 0);

	FindClose(hFind);
	return isEmpty;
}

void DeleteEmptyFolders(const std::string& sourcePath) {
	std::string currentPath = sourcePath;
	while (!currentPath.empty()) {
		if (IsDirectoryEmpty(currentPath)) {
			if (RemoveDirectoryA(currentPath.c_str())) {
				// 删除空文件夹成功
			}
			else {
				break;
			}
		}
		else {
			break;
		}

		std::size_t pos = currentPath.find_last_of("\\/");
		if (pos != std::string::npos) {
			currentPath = currentPath.substr(0, pos);
		}
		else {
			currentPath.clear();
		}
	}
}

// UTF-8解码函数实现
ucs4 DecodeUTF8_Secure(utf8*& readPtr)
{
	ucs4 result;

#define FIRST_BYTE(mask, shift)  result = (c & (mask)) << (shift);
#define NEXT_BYTE(shift) c=*readPtr; if (c==0) return 0; if ((c&0xC0) != 0x80) return IllegalSequence; readPtr++; result |= (c&0x3F) << shift;

	char c = *readPtr;
	if (c == 0)
		return '\0';
	readPtr++;

	if ((c & 0x80) == 0)
	{
		return (ucs4)c;
	}
	else if ((c & 0xE0) == 0xC0)
	{
		FIRST_BYTE(0x1F, 6);
		NEXT_BYTE(0);
		if (result < 0x80)
			return IllegalSequence;
	}
	else if ((c & 0xF0) == 0xE0)
	{
		FIRST_BYTE(0x0F, 12);
		NEXT_BYTE(6);
		NEXT_BYTE(0);
		if (result < 0x800)
			return IllegalSequence;
		if (result >= 0x0D800 && result <= 0x0DFFF)
			return IllegalSequence;
		if (result == 0x0FFFE || result == 0x0FFFF)
			return IllegalSequence;
	}
	else if ((c & 0xF8) == 0xF0)
	{
		FIRST_BYTE(0x07, 18);
		NEXT_BYTE(12);
		NEXT_BYTE(6);
		NEXT_BYTE(0);
		if (result < 0x010000)
			return IllegalSequence;
	}
	else if ((c & 0xFC) == 0xF8)
	{
		FIRST_BYTE(0x03, 24);
		NEXT_BYTE(18);
		NEXT_BYTE(12);
		NEXT_BYTE(6);
		NEXT_BYTE(0);
		if (result < 0x0200000)
			return IllegalSequence;
	}
	else if ((c & 0xFE) == 0xFC)
	{
		FIRST_BYTE(0x01, 30);
		NEXT_BYTE(24);
		NEXT_BYTE(18);
		NEXT_BYTE(12);
		NEXT_BYTE(6);
		NEXT_BYTE(0);
		if (result < 0x04000000)
			return IllegalSequence;
	}
	else
	{
		return IllegalSequence;
	}
	return result;
}

std::string SanitizeName(const char* name) {
	if (name == nullptr) {
		return std::string();
	}
	
	std::string input(name);
	std::string result;

	size_t lastSlash = input.find_last_of("\\/");
	std::string path = "";
	std::string filename = input;
	
	if (lastSlash != std::string::npos) {
		path = input.substr(0, lastSlash + 1);
		filename = input.substr(lastSlash + 1);
	}
	std::string sanitizedFilename;
	utf8* ptr = (utf8*)(filename.c_str());
	size_t len = filename.length();
	utf8* end = ptr + len;

	while (ptr < end) {
		utf8* oldPtr = ptr;
		ucs4 ch = DecodeUTF8_Secure(ptr);

		if (ch == IllegalSequence) {
			while (oldPtr != ptr) {
				sanitizedFilename += '_';
				oldPtr++;
			}
		}
		else if (ch < 0x20 || ch == 0x7F) {
			sanitizedFilename += '_';
		}
		else if (ch == ':' || ch == '*' || ch == '?' || ch == '"' || 
				 ch == '<' || ch == '>' || ch == '|') {
			sanitizedFilename += '_';
		}
		else if (ch < 0x80) {
			sanitizedFilename += (char)ch;
		}
		else {
			while (oldPtr != ptr) {
				sanitizedFilename += *oldPtr;
				oldPtr++;
			}
		}
	}

	return path + sanitizedFilename;
}

std::string CodeTUTF8(const char* str, int t)
{
	std::string result;
	WCHAR* strSrc;
	LPSTR szRes;

	int i = MultiByteToWideChar(t, 0, str, -1, NULL, 0);
	strSrc = new WCHAR[i + 1];
	MultiByteToWideChar(t, 0, str, -1, strSrc, i);

	i = WideCharToMultiByte(CP_UTF8, 0, strSrc, -1, NULL, 0, NULL, NULL);
	szRes = new CHAR[i + 1];
	WideCharToMultiByte(CP_UTF8, 0, strSrc, -1, szRes, i, NULL, NULL);

	result = szRes;
	delete[]strSrc;
	delete[]szRes;

	return result;
}

// Unity 的 AnimationClip.length 取曲线最后一帧，不用 AnimStack 时间轴。
// 关键帧压缩会保留首尾键，最后一帧时间不变。
static void AccumulateCurveStop(const FBXAnimationCurve& curve, float& stopTime, bool& hasKey)
{
	if (curve.GetKeyCount() <= 0)
		return;
	hasKey = true;
	const float curveStop = curve.GetRange().second;
	if (curveStop > stopTime)
		stopTime = curveStop;
}

static void ApplyClipLengthFromKeys(FBXImportAnimationClip& clip)
{
	float stopTime = -std::numeric_limits<float>::infinity();
	bool hasKey = false;

	for (auto it = clip.nodeAnimations.begin(); it != clip.nodeAnimations.end(); ++it)
	{
		for (int i = 0; i < 4; ++i)
			AccumulateCurveStop(it->rotation[i], stopTime, hasKey);
		for (int i = 0; i < 3; ++i)
		{
			AccumulateCurveStop(it->translation[i], stopTime, hasKey);
			AccumulateCurveStop(it->scale[i], stopTime, hasKey);
		}
	}
	for (auto it = clip.floatAnimations.begin(); it != clip.floatAnimations.end(); ++it)
		AccumulateCurveStop(it->curve, stopTime, hasKey);

	if (hasKey)
		clip.bakeStop = stopTime;
}

// 动画相关函数
void BuildSingleAnimProtoFile(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir)
{
	auto& floatCurves = clip.floatAnimations;
	auto& nodeCurves = clip.nodeAnimations;
	message::UGCResAnimClipData AnimClipProto;

	auto sampleRate = scene.sceneInfo.sampleRate;
	auto fileScale = scene.fileScaleFactor;
	ReduceKeyframes(clip, sampleRate, 0.5f, 0.5f, 0.5f, 0.5f);
	ApplyClipLengthFromKeys(clip);
	AnimClipProto.set_name(clip.name);
	AnimClipProto.set_bakestart(clip.bakeStart);
	AnimClipProto.set_bakestop(clip.bakeStop);
	AnimClipProto.set_samplerate(sampleRate);

	//Add Node Animation
	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); it++)
	{
		if (!ShouldWriteAnimNode(it->node))
			continue;
		message::UGCResAnimNodeCurves* curNodeProto = AnimClipProto.add_nodeanim();
		auto nodeName = ResolveAnimNodePath(it->node);
		curNodeProto->set_name(nodeName);

		auto& rotCurves = it->rotation;
		for (auto i = 0; i < 4; i++)
		{
			auto& curve = rotCurves[i];
			auto& keyFrames = curve.m_Curve;
			message::FBXAnimationCurve* floatAnimCurveProto = curNodeProto->add_rotation();
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				message::UGCResAnimKeyFrameFloat* keyframeProto = floatAnimCurveProto->add_keyframes();
				keyframeProto->set_time(curKeyFrame.time);
				keyframeProto->set_weightedmode(curKeyFrame.weightedMode);
				keyframeProto->set_value(curKeyFrame.value);
				keyframeProto->set_inslope(curKeyFrame.inSlope);
				keyframeProto->set_outslope(curKeyFrame.outSlope);
				keyframeProto->set_inweight(curKeyFrame.inWeight);
				keyframeProto->set_outweight(curKeyFrame.outWeight);
			}
		}
		auto& transCurves = it->translation;
		for (auto i = 0; i < 3; i++)
		{
			auto& curve = transCurves[i];
			auto& keyFrames = curve.m_Curve;
			message::FBXAnimationCurve* floatAnimCurveProto = curNodeProto->add_translation();
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				message::UGCResAnimKeyFrameFloat* keyframeProto = floatAnimCurveProto->add_keyframes();
				keyframeProto->set_time(curKeyFrame.time);
				keyframeProto->set_weightedmode(curKeyFrame.weightedMode);
				keyframeProto->set_value(curKeyFrame.value * fileScale);
				keyframeProto->set_inslope(curKeyFrame.inSlope);
				keyframeProto->set_outslope(curKeyFrame.outSlope);
				keyframeProto->set_inweight(curKeyFrame.inWeight);
				keyframeProto->set_outweight(curKeyFrame.outWeight);
			}
		}

		auto& scaleCurves = it->scale;
		for (auto i = 0; i < 3; i++)
		{
			auto& curve = scaleCurves[i];
			auto& keyFrames = curve.m_Curve;
			message::FBXAnimationCurve* floatAnimCurveProto = curNodeProto->add_scale();
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				message::UGCResAnimKeyFrameFloat* keyframeProto = floatAnimCurveProto->add_keyframes();
				keyframeProto->set_time(curKeyFrame.time);
				keyframeProto->set_weightedmode(curKeyFrame.weightedMode);
				keyframeProto->set_value(curKeyFrame.value);
				keyframeProto->set_inslope(curKeyFrame.inSlope);
				keyframeProto->set_outslope(curKeyFrame.outSlope);
				keyframeProto->set_inweight(curKeyFrame.inWeight);
				keyframeProto->set_outweight(curKeyFrame.outWeight);
			}
		}
	}
	std::string directory(outdir);
	std::string filename(clip.name);

	char tempPath[MAX_PATH];
	GetTempPathA(MAX_PATH, tempPath);
	
	char tempFileName[MAX_PATH];
	GetTempFileNameA(tempPath, "APT", 0, tempFileName);
	std::string tempAnimFilename(tempFileName);
	
	std::fstream output(tempAnimFilename, std::ios::out | std::ios::trunc | std::ios::binary);
	bool flag = AnimClipProto.SerializePartialToOstream(&output);
	if (!flag)
	{
		std::cout << "Error when Serializing Anim File" << std::endl;
	}
	output.close();

	//Rename To Support Chinese		
	std::string dstDirectory(gOutPutDir);
	std::string dstAnimfilename = SanitizeName(clip.name.c_str());
	dstAnimfilename = dstDirectory + "\\" + dstAnimfilename + ".Anim";
	std::wstring dstMeshfilenameW = ConvertUTF8ToWide(dstAnimfilename);
	RenameFileToWide(tempAnimFilename, dstMeshfilenameW);

#if DebugMeshInfoOutput
	ParseAnimProto(AnimClipProto);
#endif
}

void BuildSingleAnimBinaryFile(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir)
{
	std::string Animfilename(clip.name);
	std::string directory(outdir);
	
	char tempPath[MAX_PATH];
	GetTempPathA(MAX_PATH, tempPath);
	
	char tempFileName[MAX_PATH];
	GetTempFileNameA(tempPath, "ANM", 0, tempFileName);
	std::string tempAnimFilename(tempFileName);
	
	std::ofstream osData(tempAnimFilename, std::ios_base::out | std::ios_base::binary);
	osData.precision(8);

	//build head
	byte MagicNumber1 = 100;
	byte MagicNumber2 = 98;
	byte MagicNumber3 = 158;
	byte MagicNumber4 = 134;
	int Version = 1009715;

	ApplyClipLengthFromKeys(clip);
	float BakeStart = clip.bakeStart;
	float BakeStop = clip.bakeStop;
	float SampleRate = scene.sceneInfo.sampleRate;
	auto fileScale = scene.fileScaleFactor;
	int NameLength = clip.name.size();
	std::string Name = clip.name;

	//WriteHead
	osData.write(reinterpret_cast<char*>(&MagicNumber1), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&MagicNumber2), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&MagicNumber3), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&MagicNumber4), sizeof(byte));
	osData.write(reinterpret_cast<char*>(&Version), sizeof(int));
	osData.write(reinterpret_cast<char*>(&BakeStart), sizeof(float));
	osData.write(reinterpret_cast<char*>(&BakeStop), sizeof(float));
	osData.write(reinterpret_cast<char*>(&SampleRate), sizeof(float));
	osData.write(reinterpret_cast<char*>(&NameLength), sizeof(int));
	osData.write(Name.data(),NameLength);
	
	ReduceKeyframes(clip, SampleRate, 0.5f, 0.5f, 0.5f, 0.5f);

	//Node Anim
	auto& nodeCurves = clip.nodeAnimations;
	int NodeAnimCurveCount = 0;
	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); ++it)
	{
		if (ShouldWriteAnimNode(it->node))
			++NodeAnimCurveCount;
	}
	osData.write(reinterpret_cast<char*>(&NodeAnimCurveCount), sizeof(int));

	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); it++)
	{
		if (!ShouldWriteAnimNode(it->node))
			continue;
		auto nodeName = ResolveAnimNodePath(it->node);
		int NodeNameLength = (int)nodeName.size();
		osData.write(reinterpret_cast<char*>(&NodeNameLength), sizeof(int));
		osData.write(nodeName.data(), NodeNameLength);

		auto& rotCurves = it->rotation;
		for (auto i = 0; i < 4; i++)
		{
			auto& curve = rotCurves[i];
			auto& keyFrames = curve.m_Curve;
			int RotKeyFrameCount = keyFrames.size();
			osData.write(reinterpret_cast<char*>(&RotKeyFrameCount), sizeof(int));
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				osData.write(reinterpret_cast<char*>(&curKeyFrame.weightedMode), sizeof(int));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.time), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.value), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inWeight), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outWeight), sizeof(float));
			}
		}
		auto& scaleCurves = it->scale;
		for (auto i = 0; i < 3; i++)
		{
			auto& curve = scaleCurves[i];
			auto& keyFrames = curve.m_Curve;
			int ScaleKeyFrameCount = keyFrames.size();
			osData.write(reinterpret_cast<char*>(&ScaleKeyFrameCount), sizeof(int));
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				osData.write(reinterpret_cast<char*>(&curKeyFrame.weightedMode), sizeof(int));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.time), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.value), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inWeight), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outWeight), sizeof(float));
			}
		}
		auto& transCurves = it->translation;
		for (auto i = 0; i < 3; i++)
		{
			auto& curve = transCurves[i];
			auto& keyFrames = curve.m_Curve;
			int TransKeyFrameCount = keyFrames.size();
			osData.write(reinterpret_cast<char*>(&TransKeyFrameCount), sizeof(int));
			for (auto i = 0; i < keyFrames.size(); i++)
			{
				auto& curKeyFrame = keyFrames[i];
				//apply scale
				curKeyFrame.value = curKeyFrame.value * fileScale;
				osData.write(reinterpret_cast<char*>(&curKeyFrame.weightedMode), sizeof(int));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.time), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.value), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outSlope), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.inWeight), sizeof(float));
				osData.write(reinterpret_cast<char*>(&curKeyFrame.outWeight), sizeof(float));
			}
		}
	}

	osData.close();
	//Rename To Support Chinese	
	std::string dstDirectory(gOutPutDir);
	std::string dstAnimfilename = SanitizeName(clip.name.c_str());
	dstAnimfilename = dstDirectory + "\\" + dstAnimfilename + ".Anim";
	std::wstring AnimfilenameW = ConvertUTF8ToWide(dstAnimfilename);
	RenameFileToWide(tempAnimFilename, AnimfilenameW);
}

void WriteNodeAnimationsToText(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir)
{
	std::string Animfilename(clip.name);
	std::string directory(outdir);
	
	char tempPath[MAX_PATH];
	GetTempPathA(MAX_PATH, tempPath);
	
	char tempFileName[MAX_PATH];
	GetTempFileNameA(tempPath, "ATX", 0, tempFileName);
	std::string tempAnimFilename(tempFileName);
	
	std::ofstream osData(tempAnimFilename);

	float fileScale = scene.fileScaleFactor;
	auto& nodeCurves = clip.nodeAnimations;
	int NodeAnimCurveCount = 0;
	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); ++it)
	{
		if (ShouldWriteAnimNode(it->node))
			++NodeAnimCurveCount;
	}

	//build head
	byte MagicNumber1 = 100;
	byte MagicNumber2 = 98;
	byte MagicNumber3 = 158;
	byte MagicNumber4 = 134;
	int Version = 1009715;

	ApplyClipLengthFromKeys(clip);
	float BakeStart = clip.bakeStart;
	float BakeStop = clip.bakeStop;
	float SampleRate = scene.sceneInfo.sampleRate;
	int NameLength = clip.name.size();
	std::string Name = clip.name;

	osData << MagicNumber1 << std::endl;
	osData << MagicNumber2 << std::endl;
	osData << MagicNumber3 << std::endl;
	osData << MagicNumber4 << std::endl;
	osData << Version << std::endl;
	osData << BakeStart << std::endl;
	osData << BakeStop << std::endl;
	osData << SampleRate << std::endl;
	osData << NameLength << std::endl;
	osData << Name.data() << std::endl;
	osData << "**********************************************" << std::endl;
	osData << "Node Anim Count: " << NodeAnimCurveCount << std::endl;

	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); it++) {
		if (!ShouldWriteAnimNode(it->node))
			continue;
		auto nodeName = ResolveAnimNodePath(it->node);
		int NodeNameLength = (int)nodeName.size();
		osData << "Node Anim NodeNameLength: " << NodeNameLength << std::endl;
		osData << "Node Anim NodeName: " << nodeName.data() << std::endl;

		auto& rotCurves = it->rotation;
		for (auto i = 0; i < 4; i++) {
			auto& curve = rotCurves[i];
			auto& keyFrames = curve.m_Curve;
			int RotKeyFrameCount = keyFrames.size();
			osData << "Node Anim RotKeyFrameCount: " << RotKeyFrameCount << std::endl;
			for (auto j = 0; j < keyFrames.size(); j++) {
				auto& curKeyFrame = keyFrames[j];

				osData << "    Channel [" << i << " ] Num [ " << j << "] weightMode " << curKeyFrame.weightedMode << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] time " << curKeyFrame.time << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] value " << curKeyFrame.value << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inSlope " << curKeyFrame.inSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outSlope " << curKeyFrame.outSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inWeight " << curKeyFrame.inWeight << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outWeight " << curKeyFrame.outWeight << std::endl;
			}
		}

		auto& scaleCurves = it->scale;
		for (auto i = 0; i < 3; i++) {
			auto& curve = scaleCurves[i];
			auto& keyFrames = curve.m_Curve;
			int ScaleKeyFrameCount = keyFrames.size();
			osData << "Node Anim ScaleKeyFrameCount: " << ScaleKeyFrameCount << std::endl;
			for (auto j = 0; j < keyFrames.size(); j++) {
				auto& curKeyFrame = keyFrames[j];
				osData << "    Channel [" << i << " ] Num [ " << j << "] weightMode " << curKeyFrame.weightedMode << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] time " << curKeyFrame.time << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] value " << curKeyFrame.value << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inSlope " << curKeyFrame.inSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outSlope " << curKeyFrame.outSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inWeight " << curKeyFrame.inWeight << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outWeight " << curKeyFrame.outWeight << std::endl;
			}
		}

		auto& transCurves = it->translation;
		for (auto i = 0; i < 3; i++) {
			auto& curve = transCurves[i];
			auto& keyFrames = curve.m_Curve;
			int TransKeyFrameCount = keyFrames.size();
			osData << "Node Anim TransKeyFrameCount: " << TransKeyFrameCount << std::endl;
			for (auto j = 0; j < keyFrames.size(); j++) {
				auto& curKeyFrame = keyFrames[j];
				// apply scale
				curKeyFrame.value = curKeyFrame.value * fileScale;
				osData << "    Channel [" << i << " ] Num [ " << j << "] weightMode " << curKeyFrame.weightedMode << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] time " << curKeyFrame.time << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] value " << curKeyFrame.value << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inSlope " << curKeyFrame.inSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outSlope " << curKeyFrame.outSlope << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] inWeight " << curKeyFrame.inWeight << std::endl;
				osData << "    Channel [" << i << " ] Num [ " << j << "] outWeight " << curKeyFrame.outWeight << std::endl;
			}
		}

		osData << "***************************************" << std::endl;
	}

	osData.close();
	//Rename To Support Chinese		
	std::string dstDirectory(gOutPutDir);
	std::string dstAnimfilename = SanitizeName(clip.name.c_str());
	dstAnimfilename = dstDirectory + "\\" + dstAnimfilename + "_anim.txt";
	std::wstring AnimfilenameW = ConvertUTF8ToWide(dstAnimfilename);
	RenameFileToWide(tempAnimFilename, AnimfilenameW);
}

void BuildBoneNodeData(FBXImportScene& scene, FBXImportNode& node, message::UGCResBoneNodeData* parent, std::string parentpath)
{
	BuildBoneNodeData(scene, node, parent, parentpath, node.position, node.rotation, node.scale);
}

void BuildBoneNodeData(FBXImportScene& scene, FBXImportNode& node, message::UGCResBoneNodeData* parent, const std::string& parentpath, const Vector3f& position, const Quaternionf& rotation, const Vector3f& scale)
{
	message::UGCResBoneNodeData* msg_node = parent->add_childbones();
	message::UGCResBoneNodeCapsuleData root_capsule;

	message::ProtoBuffVector3* msg_pos = new message::ProtoBuffVector3();
	message::ProtoBuffVector3* msg_scale = new message::ProtoBuffVector3();
	message::ProtoBuffQuaternion* msg_quat = new message::ProtoBuffQuaternion();
	auto fileScale = scene.fileScaleFactor;
	msg_pos->set_x(position.x * fileScale); msg_pos->set_y(position.y * fileScale); msg_pos->set_z(position.z * fileScale);
	msg_scale->set_x(scale.x); msg_scale->set_y(scale.y); msg_scale->set_z(scale.z);
	msg_quat->set_x(rotation.x); msg_quat->set_y(rotation.y); msg_quat->set_z(rotation.z); msg_quat->set_w(rotation.w);

	msg_node->set_bonename(node.name);
	msg_node->set_allocated_localposition(msg_pos);
	msg_node->set_allocated_localrotation(msg_quat);
	msg_node->set_allocated_localscale(msg_scale);
	std::string nodeName = node.name;
	std::string nodePath = nodeName;
	if (parentpath != "")
		nodePath = parentpath + "/" + nodeName;
	RegisterNodePath(node, nodePath);
	
	std::string childParentPath = nodePath;
	
	for (auto i = 0; i < node.children.size(); i++)
	{
		BuildBoneNodeData(scene, node.children[i], msg_node, childParentPath);
	}
}

void ParseSingleMesh(std::string meshfile)
{
	std::cout << "*******************************Begin Parse Single Mesh**************************" << std::endl;
	std::ifstream istream(meshfile, std::ios_base::in | std::ios_base::binary);

	MeshHead head;
	istream.read(reinterpret_cast<char*>(&head.MagicNumber1), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&head.MagicNumber2), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&head.MagicNumber3), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&head.MagicNumber4), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&head.Version), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshType), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshDataStartPos), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshDataSize), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshDataExtPos), sizeof(int));
	istream.read(reinterpret_cast<char*>(&head.MeshDataExtSize), sizeof(int));

	std::cout << "filename: " << meshfile << std::endl;
	std::cout << "MagicNumber: " << (int)head.MagicNumber1 << "," << (int)head.MagicNumber2 << "," << (int)head.MagicNumber3 << "," << (int)head.MagicNumber4 << std::endl;
	std::cout << "Version: " << head.Version << std::endl;
	std::cout << "MeshType: " << head.MeshType << std::endl;
	std::cout << "Version: " << head.Version << std::endl;
	std::cout << "MeshDataStartPos: " << head.MeshDataStartPos << std::endl;
	std::cout << "MeshDataSize: " << head.MeshDataSize << std::endl;
	std::cout << "MeshDataExtPos: " << head.MeshDataExtPos << std::endl;
	std::cout << "MeshDataExtSize: " << head.MeshDataExtSize << std::endl;

	MeshBody body;

	//Name
	{
		istream.read(reinterpret_cast<char*>(&body.NameLength), 8);
		std::string name;
		istream.read(reinterpret_cast<char*>(&name), body.NameLength);
		std::cout << "[BODY]Name: " << name.c_str() << std::endl;
	}
	//vertices
	{
		istream.read(reinterpret_cast<char*>(&body.VerticesLength), 8);
		std::vector<Vector3f> verts;
		verts.clear();
		for (auto i = 0; i < body.VerticesLength; i++)
		{
			Vector3f vert;
			istream.read(reinterpret_cast<char*>(&vert.x), sizeof(float));
			istream.read(reinterpret_cast<char*>(&vert.y), sizeof(float));
			istream.read(reinterpret_cast<char*>(&vert.z), sizeof(float));
			verts.push_back(vert);
		}
		std::cout << "[BODY]Vertices: " << verts.size() << std::endl;
	}

	//normals
	{
		istream.read(reinterpret_cast<char*>(&body.NormalLength), 8);
		std::vector<Vector3f> normals;
		normals.clear();
		for (auto i = 0; i < body.NormalLength; i++)
		{
			Vector3f norm;
			istream.read(reinterpret_cast<char*>(&norm.x), sizeof(float));
			istream.read(reinterpret_cast<char*>(&norm.y), sizeof(float));
			istream.read(reinterpret_cast<char*>(&norm.z), sizeof(float));
			normals.push_back(norm);
		}
		std::cout << "[BODY]Normal: " << normals.size() << std::endl;
	}

	//uv1
	{
		istream.read(reinterpret_cast<char*>(&body.UV1Length), 8);
		std::vector<Vector2f> uv1s;
		uv1s.clear();
		for (auto i = 0; i < body.UV1Length; i++)
		{
			Vector2f uv;
			istream.read(reinterpret_cast<char*>(&uv.x), sizeof(float));
			istream.read(reinterpret_cast<char*>(&uv.y), sizeof(float));
			uv1s.push_back(uv);
		}
		std::cout << "[BODY]UV1: " << uv1s.size() << std::endl;
	}

	//uv2
	{
		istream.read(reinterpret_cast<char*>(&body.UV2Length), 8);
		std::vector<Vector2f> uv2s;
		uv2s.clear();
		for (auto i = 0; i < body.UV2Length; i++)
		{
			Vector2f uv;
			istream.read(reinterpret_cast<char*>(&uv.x), sizeof(float));
			istream.read(reinterpret_cast<char*>(&uv.y), sizeof(float));
			uv2s.push_back(uv);
		}
		std::cout << "[BODY]UV2: " << uv2s.size() << std::endl;
	}

	//IndexSize
	{
		istream.read(reinterpret_cast<char*>(&body.IndexSizeLength), 8);
		std::vector<uint32_t> indexSizes;
		indexSizes.clear();
		for (auto i = 0; i < body.IndexSizeLength; i++)
		{
			uint32_t idxsize;
			istream.read(reinterpret_cast<char*>(&idxsize), sizeof(uint32_t));
			indexSizes.push_back(idxsize);
		}
		std::cout << "[BODY]IndexSize: " << indexSizes.size() << std::endl;
	}

	//Index
	{
		istream.read(reinterpret_cast<char*>(&body.IndexLength), 8);
		std::vector<uint32_t> indexes;
		indexes.clear();
		for (auto i = 0; i < body.IndexLength; i++)
		{
			uint32_t idxsize;
			istream.read(reinterpret_cast<char*>(&idxsize), sizeof(uint32_t));
			indexes.push_back(idxsize);
		}
		std::cout << "[BODY]Index: " << indexes.size() << std::endl;
	}

	//Material
	{
		istream.read(reinterpret_cast<char*>(&body.MatLength), 8);
		std::vector<uint32_t> mats;
		mats.clear();
		for (auto i = 0; i < body.MatLength; i++)
		{
			uint32_t mat;
			istream.read(reinterpret_cast<char*>(&mat), sizeof(uint32_t));
			mats.push_back(mat);
		}
		std::cout << "[BODY]Index: " << mats.size() << std::endl;
	}

	message::UGCResSkinnedMeshExtData ext1;
	bool flag = ext1.ParseFromIstream(&istream);
	if (flag)
	{
		std::cout << "Byte Size of ext new " << ext1.ByteSizeLong() << std::endl;
		if (ext1.ByteSizeLong() > 0)
		{
			for (auto i = 0; i < ext1.bonenames_size(); i++)
			{
				std::cout << "the [" << i << "] Bone is " << ext1.bonenames(i) << std::endl;
			}
		}
	}
	std::cout << "*******************************End Parse Single Mesh**************************" << std::endl;
}

void DebugMeshInfo(FBXMesh& meshData, MeshBody& bodyinfo)
{
	uint64_t namecount = strlen(meshData.name) + 1;
	uint64_t vertexcount = meshData.vertices.size();
	uint64_t normalcount = meshData.normals.size();
	uint64_t uv1count = meshData.uv1.size();
	uint64_t uv2count = meshData.uv2.size();
	uint64_t indicesizecount = meshData.indicesize.size();
	uint64_t indicecount = meshData.indices.size();
	uint64_t matcount = meshData.materialindex.size();
	std::cout << "////////////////////////////////////////////////////////////////////////" << std::endl;
	std::cout << "name: " << meshData.name << std::endl;
	std::cout << "namecount: " << namecount << std::endl;
	std::cout << "vertexcount: " << vertexcount << std::endl;
	std::cout << "normalcount: " << normalcount << std::endl;
	std::cout << "uv1count: " << uv1count << std::endl;
	std::cout << "uv2count: " << uv2count << std::endl;
	std::cout << "indicesizecount: " << indicesizecount << std::endl;
	std::cout << "indicecount: " << indicecount << std::endl;

	std::cout << "************************************************************************" << std::endl;
	std::cout << "bodyinfo.NamePos :" << bodyinfo.NamePos << std::endl;
	std::cout << "bodyinfo.NameHeadLength :" << bodyinfo.NameHeadLength << std::endl;
	std::cout << "bodyinfo.NameLength :" << bodyinfo.NameLength << std::endl;
	std::cout << "bodyinfo.VerticesPos :" << bodyinfo.VerticesPos << std::endl;
	std::cout << "bodyinfo.VerticesHeadLength :" << bodyinfo.VerticesHeadLength << std::endl;
	std::cout << "bodyinfo.VerticesLength :" << bodyinfo.VerticesLength << std::endl;
	std::cout << "bodyinfo.NormalPos :" << bodyinfo.NormalPos << std::endl;
	std::cout << "bodyinfo.NormalHeadLength :" << bodyinfo.NormalHeadLength << std::endl;
	std::cout << "bodyinfo.NormalLength :" << bodyinfo.NormalLength << std::endl;
	std::cout << "bodyinfo.UV1Pos :" << bodyinfo.UV1Pos << std::endl;
	std::cout << "bodyinfo.UV1HeadLength :" << bodyinfo.UV1HeadLength << std::endl;
	std::cout << "bodyinfo.UV1Length :" << bodyinfo.UV1Length << std::endl;
	std::cout << "bodyinfo.UV2Pos :" << bodyinfo.UV2Pos << std::endl;
	std::cout << "bodyinfo.UV2HeadLength :" << bodyinfo.UV2HeadLength << std::endl;
	std::cout << "bodyinfo.UV2Length :" << bodyinfo.UV2Length << std::endl;
	std::cout << "bodyinfo.IndexSizePos :" << bodyinfo.IndexSizePos << std::endl;
	std::cout << "bodyinfo.IndexSizeHeadLength :" << bodyinfo.IndexSizeHeadLength << std::endl;
	std::cout << "bodyinfo.IndexSizeLength :" << bodyinfo.IndexSizeLength << std::endl;
	std::cout << "bodyinfo.IndexPos :" << bodyinfo.IndexPos << std::endl;
	std::cout << "bodyinfo.IndexHeadLength :" << bodyinfo.IndexHeadLength << std::endl;
	std::cout << "bodyinfo.IndexLength :" << bodyinfo.IndexLength << std::endl;
	std::cout << "bodyinfo.MatPos :" << bodyinfo.MatPos << std::endl;
	std::cout << "bodyinfo.MatHeadLength :" << bodyinfo.MatHeadLength << std::endl;
	std::cout << "bodyinfo.MatLength :" << bodyinfo.MatLength << std::endl;
	std::cout << "bodyinfo.BindPosesPos :" << bodyinfo.BindPosesPos << std::endl;
	std::cout << "bodyinfo.BindPosesHeadLength :" << bodyinfo.BindPosesHeadLength << std::endl;
	std::cout << "bodyinfo.BindPosesLength :" << bodyinfo.BindPosesLength << std::endl;
	std::cout << "bodyinfo.BoneWeightPos :" << bodyinfo.BoneWeightPos << std::endl;
	std::cout << "bodyinfo.BoneWeightHeadLength :" << bodyinfo.BoneWeightHeadLength << std::endl;
	std::cout << "bodyinfo.BoneWeightLength :" << bodyinfo.BoneWeightLength << std::endl;
}

void ParseAnimProto(message::UGCResAnimClipData& msg)
{
	std::cout << "*****************************Begin Parse Anim Proto!*********************************" << std::endl;
	//name
	auto name = msg.name();
	std::cout << "The Name is :" << name.c_str() << std::endl;
	//BakeStart,End
	auto start = msg.bakestart();
	auto stop = msg.bakestop();
	std::cout << "The bake start stop are : " << start << " , " << stop << std::endl;
	auto samplerate = msg.samplerate();
	std::cout << "The Sample Rate is : " << samplerate << std::endl;
	//Float Anim
	auto floatAnimSize = msg.floatanim_size();
	for (auto i = 0; i < floatAnimSize; i++)
	{
		auto floatAnim = msg.floatanim(i);
		std::cout << "The [" << i << "] th Float Anim are list below: " << std::endl;
		std::cout << "		The ClassName is : " << floatAnim.classname().c_str() << std::endl;
		std::cout << "		The PropertyName is : " << floatAnim.propertyname().c_str() << std::endl;
		std::cout << "		The CurveCount is : " << floatAnim.curve_size() << std::endl;
		for (int j = 0; j < floatAnim.curve_size(); j++)
		{
			auto curve = floatAnim.curve(j);
			std::cout << "		The [" << j << "] channel curve count is  : " << curve.keyframes_size() << std::endl;
		}
	}

	//Node Anim
	auto nodeAnimSize = msg.nodeanim_size();
	for (auto i = 0; i < nodeAnimSize; i++)
	{
		auto nodeAnim = msg.nodeanim(i);
		auto name = nodeAnim.name();
		std::cout << "The [" << i << "] th Node Anim:" << name << " are list below: " << std::endl;
		std::cout << "		The Rotation CurveChannel is : " << nodeAnim.rotation_size() << std::endl;
		for (int j = 0; j < nodeAnim.rotation_size(); j++)
		{
			auto rot = nodeAnim.rotation(j);
			std::cout << "		The [" << j << "] channel curve count is  : " << rot.keyframes_size() << std::endl;
		}

		std::cout << "		The Scale CurveChannel is : " << nodeAnim.scale_size() << std::endl;
		for (int j = 0; j < nodeAnim.scale_size(); j++)
		{
			auto scale = nodeAnim.scale(j);
			std::cout << "		The [" << j << "] channel curve count is  : " << scale.keyframes_size() << std::endl;
		}
		std::cout << "		The Transition CurveChannel is : " << nodeAnim.translation_size() << std::endl;
		for (int j = 0; j < nodeAnim.translation_size(); j++)
		{
			auto trans = nodeAnim.translation(j);
			std::cout << "		The [" << j << "] channel curve count is  : " << trans.keyframes_size() << std::endl;
		}
	}

	std::cout << "*****************************End Parse Anim Proto!*********************************" << std::endl;
}

void ParseSingleAnim(std::string animfile)
{
	std::cout << "*******************************Begin Parse Single Anim**************************" << std::endl;
	std::ifstream istream(animfile, std::ios_base::in | std::ios_base::binary);

	byte MagicNum[4] = { 0,0,0,0 };
	int Version = -1;
	float BakeStart = 0;
	float BakeStop = 0;
	float SampleRate = 0;
	int NameLength = 0;
	std::string Name;

	istream.read(reinterpret_cast<char*>(&MagicNum[0]), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&MagicNum[1]), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&MagicNum[2]), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&MagicNum[3]), sizeof(byte));
	istream.read(reinterpret_cast<char*>(&Version), sizeof(int));
	istream.read(reinterpret_cast<char*>(&BakeStart), sizeof(float));
	istream.read(reinterpret_cast<char*>(&BakeStop), sizeof(float));
	istream.read(reinterpret_cast<char*>(&SampleRate), sizeof(float));
	istream.read(reinterpret_cast<char*>(&NameLength), sizeof(int));
	Name.resize(NameLength);
	istream.read(&Name[0], NameLength);

	std::cout << "MagicNumber: " << (int)MagicNum[0] << "," << (int)MagicNum[1] << "," << (int)MagicNum[2] << "," << (int)MagicNum[3] << std::endl;
	std::cout << "Version: " << Version << std::endl;
	std::cout << "BakeStart: " << BakeStart << std::endl;
	std::cout << "BakeStop: " << BakeStop << std::endl;
	std::cout << "SampleRate: " << SampleRate << std::endl;
	std::cout << "NameLength: " << NameLength << std::endl;
	std::cout << "filename: " << Name << std::endl;
	int NodeCount = 0;
	istream.read(reinterpret_cast<char*>(&NodeCount), sizeof(int));
	std::cout << "NodeCount: " << NodeCount << std::endl;
	for (auto i = 0; i < NodeCount; i++)
	{
		int NodeNameLength = -1;
		std::string NodeName;
		istream.read(reinterpret_cast<char*>(&NodeNameLength), sizeof(int));
		NodeName.resize(NodeNameLength);
		istream.read(&NodeName[0], NodeNameLength);

		std::cout << "Node Name: " << NodeName << std::endl;

		int weightedMode = -1;
		float time, value, inSlope, outSlope, inWeight, outWeight;

		//Rot
		for (auto channel = 0; channel < 4; channel++)
		{
			int RotKeyFrameCount = 0;
			istream.read(reinterpret_cast<char*>(&RotKeyFrameCount), sizeof(int));
			if (RotKeyFrameCount != 0)
			{
				std::cout << "Node RotKeyFrameCount channel : " << (int)channel << "Node RotKeyFrameCount: " << RotKeyFrameCount<<std::endl;
			}
			for (auto id = 0; id < RotKeyFrameCount; id++)
			{
				istream.read(reinterpret_cast<char*>(&weightedMode), sizeof(int));
				istream.read(reinterpret_cast<char*>(&time), sizeof(float));
				istream.read(reinterpret_cast<char*>(&value), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inWeight), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outWeight), sizeof(float));
				std::cout << "		time: " << time << std::endl;
				std::cout << "		value: " << value << std::endl;
			}
		}
		//Scale
		for (auto channel = 0; channel < 3; channel++)
		{
			int ScaleKeyFrameCount = 0;
			istream.read(reinterpret_cast<char*>(&ScaleKeyFrameCount), sizeof(int));
			if (ScaleKeyFrameCount != 0)
			{
				std::cout << "Node ScaleKeyFrameCount channel : " << (int)channel << "Node ScaleKeyFrameCount: " << ScaleKeyFrameCount << std::endl;
			}

			for (auto id = 0; id < ScaleKeyFrameCount; id++)
			{
				istream.read(reinterpret_cast<char*>(&weightedMode), sizeof(int));
				istream.read(reinterpret_cast<char*>(&time), sizeof(float));
				istream.read(reinterpret_cast<char*>(&value), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inWeight), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outWeight), sizeof(float));
				std::cout << "		time: " << time << std::endl;
				std::cout << "		value: " << value << std::endl;
			}
		}
		//Trans
		for (auto channel = 0; channel < 3; channel++)
		{
			int TransKeyFrameCount = 0;
			istream.read(reinterpret_cast<char*>(&TransKeyFrameCount), sizeof(int));
			if (TransKeyFrameCount != 0)
			{
				std::cout << "Node TransKeyFrameCount channel : " << (int)channel << "Node TransKeyFrameCount: " << TransKeyFrameCount << std::endl;
			}
			for (auto id = 0; id < TransKeyFrameCount; id++)
			{
				istream.read(reinterpret_cast<char*>(&weightedMode), sizeof(int));
				istream.read(reinterpret_cast<char*>(&time), sizeof(float));
				istream.read(reinterpret_cast<char*>(&value), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outSlope), sizeof(float));
				istream.read(reinterpret_cast<char*>(&inWeight), sizeof(float));
				istream.read(reinterpret_cast<char*>(&outWeight), sizeof(float));
				std::cout << "		time: " << time << std::endl;
				std::cout << "		value: " << value << std::endl;
			}
		}
	}

	std::cout << "*******************************End Parse Single Mesh**************************" << std::endl;
} 