#include "DataGenerate.h"
#include "Utility.h"
#include "FBXImporterDef.h"
#include "AnimationKeyFrameReducer.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <Windows.h>

// 全局变量定义
std::map<std::string, std::string> gNodePath2Name;
std::map<std::string, std::vector<std::string>> gNodeName2BoneName;
std::map<std::string, std::vector<Matrix4x4f>> gNodeName2BoneBindePose;
std::map<std::string, std::string> gBlendShapeMesh2Bone;
std::string gOutPutDir;
std::string gOutPutDirSantinized;

void BuildBoneNameMap(FBXImportNode& node, std::map<std::string, std::string>& path2name, std::string parentpath)
{
	std::string NodeName = node.name;
	std::string NodePath = NodeName;
	if(parentpath != "")
		NodePath = parentpath + "/" + NodeName;
	path2name.insert(std::pair<std::string, std::string>(NodePath, NodeName));
	for (auto i = 0; i < node.children.size(); i++)
	{
		if(node.isBone)
			BuildBoneNameMap(node.children[i], path2name, NodePath);
		else
			BuildBoneNameMap(node.children[i], path2name, "");
	}
}

void BuildAllBoneNameMap(FBXImportScene& scene)
{
	gNodePath2Name.clear();
	auto allnodes = scene.nodes;
	if (allnodes.size() > 1)
	{
		for (auto i = 0; i < allnodes.size(); i++)
		{
			BuildBoneNameMap(allnodes[i], gNodePath2Name);
		}
	}
	else if (allnodes.size() == 1)
	{
		std::vector<FBXImportNode> root = allnodes[0].children;
		if (allnodes[0].name == UNITY_BONE_ROOT)
		{
			root = allnodes;
		}

		for (auto i = 0; i < root.size(); i++)
		{
			BuildBoneNameMap(root[i], gNodePath2Name);
		}
	}
}

void BuildMeshBoneRefMap(FBXImportScene& scene)
{
	gNodeName2BoneName.clear();
	gNodeName2BoneBindePose.clear();
	auto meshes = scene.meshes;
	auto scale = scene.fileScaleFactor;
	for (auto i = 0; i < meshes.size(); i++)
	{
		auto singleMeshBones = meshes[i].bones;
		std::vector<std::string> boneNames;
		std::vector<Matrix4x4f> boneBindPoses;

		boneNames.clear();
		boneBindPoses.clear();
		for (auto j = 0; j < singleMeshBones.size(); j++)
		{
			std::string name = singleMeshBones[j].node->name;
			Matrix4x4f& bindpos = singleMeshBones[j].bindpose;
			Matrix4x4f newPose = bindpos;
			newPose[12] *= scale;
			newPose[13] *= scale;
			newPose[14] *= scale;
			boneNames.push_back(name);
			boneBindPoses.push_back(newPose);
		}
		if (boneNames.size() > 0)
			gNodeName2BoneName.insert(std::pair<std::string, std::vector<std::string>>(meshes[i].name, boneNames));

		if (boneBindPoses.size() > 0)
			gNodeName2BoneBindePose.insert(std::pair<std::string, std::vector<Matrix4x4f>>(meshes[i].name, boneBindPoses));
	}
}

void BuildBlendShapeBoneMap(FBXImportScene& scene)
{
	auto allMesh = scene.meshes;
	for (auto it = allMesh.begin(); it != allMesh.end(); it++)
	{
		if (it->shapes.size() > 0 && it->vertices.size() > 0)
		{
			auto blendShapeMeshName = it->name;
			for (const auto& pair : gNodePath2Name) {
				if (pair.second == blendShapeMeshName) {
					auto bonename = pair.first;
					size_t pos = bonename.rfind('/');
					if (pos != std::string::npos) 
					{
						bonename = bonename.substr(pos + 1);
					}
					gBlendShapeMesh2Bone.insert(std::pair<std::string, std::string>(pair.second, bonename));
					break; 
				}
			}
		}
	}
}

void PrebuildFBXMeshForBlendShape(FBXMesh& meshData)
{
	if (meshData.vertices.size() > 0)
	{
		auto allBoneWeights = meshData.boneWeights;
		BoneWeights4 boneWeights = { {1.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0} };
		allBoneWeights.resize(meshData.vertices.size(), boneWeights);
		Matrix4x4f ident;
		ident.SetIdentity();
		meshData.bindPoses.push_back(ident);
	}
}

message::UGCResSkinnedMeshExtData BuildMeshExtData(FBXMesh& meshData)
{
	std::string meshName = meshData.name;
	message::UGCResSkinnedMeshExtData ext;
	if (gNodeName2BoneName.find(meshName) != gNodeName2BoneName.end())
	{
		auto bones = gNodeName2BoneName[meshName];
		for (auto i = 0; i < bones.size(); i++)
		{
			ext.add_bonenames(bones[i]);
		}
	}
	if (gBlendShapeMesh2Bone.find(meshName) != gBlendShapeMesh2Bone.end())
	{
		auto bone = gBlendShapeMesh2Bone[meshName];
		ext.add_bonenames(bone);
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
	realPath = std::string(meshData.name);
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

	body.UV1Pos += body.NormalPos + body.NormalHeadLength + body.NormalLength * sizeof(Vector3f);
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

		// 首先尝试直接移动文件
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
					wprintf(L"File copy failed, error code: %d\n", GetLastError());
				}
			}
			else {
				wprintf(L"File rename failed, error code: %d\n", error);
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

// 动画相关函数
void BuildSingleAnimProtoFile(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir)
{
	auto& floatCurves = clip.floatAnimations;
	auto& nodeCurves = clip.nodeAnimations;
	message::UGCResAnimClipData AnimClipProto;

	auto sampleRate = scene.sceneInfo.sampleRate;
	auto fileScale = scene.fileScaleFactor;
	ReduceKeyframes(clip, sampleRate, 0.5f, 0.5f, 0.5f, 0.5f);
	AnimClipProto.set_name(clip.name);
	AnimClipProto.set_bakestart(clip.bakeStart);
	AnimClipProto.set_bakestop(clip.bakeStop);
	AnimClipProto.set_samplerate(sampleRate);

	//Add Node Animation
	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); it++)
	{
		message::UGCResAnimNodeCurves* curNodeProto = AnimClipProto.add_nodeanim();
		auto nodeName = it->node->name;

		for (auto it = gNodePath2Name.begin(); it != gNodePath2Name.end(); it++)
		{
			if (it->second == nodeName)
			{
				nodeName = it->first;
				break;
			}
		}
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
	std::string dstAnimfilename(clip.name);
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
	int NodeAnimCurveCount = clip.nodeAnimations.size();
	osData.write(reinterpret_cast<char*>(&NodeAnimCurveCount), sizeof(int));

	for (auto it = nodeCurves.begin(); it != nodeCurves.end(); it++)
	{
		auto nodeName = it->node->name;

		for (auto it = gNodePath2Name.begin(); it != gNodePath2Name.end(); it++)
		{
			if (it->second == nodeName)
			{
				nodeName = it->first;
				break;
			}
		}
		int NodeNameLength = nodeName.size();
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
	std::string dstAnimfilename(clip.name);
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
	int NodeAnimCurveCount = nodeCurves.size();

	//build head
	byte MagicNumber1 = 100;
	byte MagicNumber2 = 98;
	byte MagicNumber3 = 158;
	byte MagicNumber4 = 134;
	int Version = 1009715;

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
		auto nodeName = it->node->name;
		for (auto it = gNodePath2Name.begin(); it != gNodePath2Name.end(); it++) {
			if (it->second == nodeName) {
				nodeName = it->first;
				break;
			}
		}
		int NodeNameLength = nodeName.size();
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
	std::string dstAnimfilename(clip.name);
	dstAnimfilename = dstDirectory + "\\" + dstAnimfilename + "_anim.txt";
	std::wstring AnimfilenameW = ConvertUTF8ToWide(dstAnimfilename);
	RenameFileToWide(tempAnimFilename, AnimfilenameW);
}

void BuildBoneNodeData(FBXImportScene& scene, FBXImportNode& node, message::UGCResBoneNodeData* parent)
{
	message::UGCResBoneNodeData* msg_node = parent->add_childbones();
	message::UGCResBoneNodeCapsuleData root_capsule;

	message::ProtoBuffVector3* msg_pos = new message::ProtoBuffVector3();
	message::ProtoBuffVector3* msg_scale = new message::ProtoBuffVector3();
	message::ProtoBuffQuaternion* msg_quat = new message::ProtoBuffQuaternion();
	auto scale = scene.fileScaleFactor;
	msg_pos->set_x(node.position.x * scale); msg_pos->set_y(node.position.y * scale); msg_pos->set_z(node.position.z * scale);
	msg_scale->set_x(node.scale.x); msg_scale->set_y(node.scale.y); msg_scale->set_z(node.scale.z);
	msg_quat->set_x(node.rotation.x); msg_quat->set_y(node.rotation.y); msg_quat->set_z(node.rotation.z); msg_quat->set_w(node.rotation.w);

	msg_node->set_bonename(node.name);
	msg_node->set_allocated_localposition(msg_pos);
	msg_node->set_allocated_localrotation(msg_quat);
	msg_node->set_allocated_localscale(msg_scale);
	for (auto i = 0; i < node.children.size(); i++)
	{
		auto nextNode = node.children[i];
		BuildBoneNodeData(scene, nextNode, msg_node);
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