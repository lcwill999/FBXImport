#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <Windows.h>
#include "proto/ProtoBuffUGCResource.pb.h"
#include "FBXImporterDef.h"
#include "Matrix.h"
#include "Vector.h"
#include "Color.h"

// 调试输出宏定义
#define DebugMeshInfoOutput 0

// 外部全局变量声明
extern std::map<std::string, std::string> gNodePath2Name;
extern std::map<std::string, std::vector<std::string>> gNodeName2BoneName;
extern std::map<std::string, std::vector<Matrix4x4f>> gNodeName2BoneBindePose;
extern std::map<std::string, std::string> gBlendShapeMesh2Bone;
extern std::string gOutPutDir;
extern std::string gOutPutDirSantinized;

// 字符串转换函数声明
std::string CodeTUTF8(const char* str, int t);
std::wstring ConvertUTF8ToWide(const std::string& utf8Str);
std::wstring ConvertToWide(const char* utf8Str);
std::wstring MultiByteToWide(const std::string& multiByteStr, UINT codePage);

// 目录和文件操作函数声明
void EnsureDirectoryExists(const std::wstring& directoryPath);
void EnsureDirectoryExists(const std::string& directoryPath);
void RenameFileToWide(const std::string& originalName, const std::wstring& newName);
bool IsSubPath(const std::wstring& sourcePath, const std::wstring& targetPath);
bool IsDirectoryEmpty(const std::string& directoryPath);
void DeleteEmptyFolders(const std::string& sourcePath);

// UTF-8 解码相关
typedef uint32_t ucs4;
typedef uint8_t utf8;
enum { IllegalSequence = 0x0FFFD };
ucs4 DecodeUTF8_Secure(utf8*& readPtr);
std::string SanitizeName(const char* name);

// 骨骼名称映射函数声明
void BuildBoneNameMap(FBXImportNode& node, std::map<std::string, std::string>& path2name, std::string parentpath = "");
void BuildAllBoneNameMap(FBXImportScene& scene);
void BuildMeshBoneRefMap(FBXImportScene& scene);
void BuildBlendShapeBoneMap(FBXImportScene& scene);

// 网格头部结构体
struct MeshHead
{
	byte MagicNumber1;
	byte MagicNumber2;
	byte MagicNumber3;
	byte MagicNumber4;
	int Version;
	int MeshType;
	int MeshDataStartPos;
	int MeshDataSize;
	int MeshDataExtPos;
	int MeshDataExtSize;

	MeshHead() : MagicNumber1(0), MagicNumber2(0), MagicNumber3(0), MagicNumber4(0),
		Version(0), MeshType(0), MeshDataStartPos(0), MeshDataSize(0), MeshDataExtPos(0), MeshDataExtSize(0)
	{}
};

// 网格体结构体
struct MeshBody
{
	int NamePos;
	int NameHeadLength;//8
	uint64_t NameLength;
	int VerticesPos;
	int VerticesHeadLength;//8
	uint64_t VerticesLength;
	int ColorPos;
	int ColorHeadLength;//8
	uint64_t ColorLength;
	int NormalPos;
	int NormalHeadLength;//8
	uint64_t NormalLength;
	int UV1Pos;
	int UV1HeadLength;//8
	uint64_t UV1Length;
	int UV2Pos;
	int UV2HeadLength;//8
	uint64_t UV2Length;
	int IndexSizePos;
	int IndexSizeHeadLength;//8
	uint64_t IndexSizeLength;
	int IndexPos;
	int IndexHeadLength;//8
	uint64_t IndexLength;
	int MatPos;
	int MatHeadLength;//8
	uint64_t MatLength;
	int BindPosesPos;
	int BindPosesHeadLength;//8
	uint64_t BindPosesLength;
	int BoneWeightPos;
	int BoneWeightHeadLength;//8
	uint64_t BoneWeightLength;

	MeshBody() : NamePos(0), NameHeadLength(0), NameLength(0), VerticesPos(0), VerticesHeadLength(0), VerticesLength(0),
		ColorPos(0), ColorHeadLength(0), ColorLength(0), NormalPos(0), NormalHeadLength(0), NormalLength(0),
		UV1Pos(0), UV1HeadLength(0), UV1Length(0), UV2Pos(0), UV2HeadLength(0), UV2Length(0),
		IndexSizePos(0), IndexSizeHeadLength(0), IndexSizeLength(0), IndexPos(0), IndexHeadLength(0), IndexLength(0),
		MatPos(0), MatHeadLength(0), MatLength(0), BindPosesPos(0), BindPosesHeadLength(0), BindPosesLength(0),
		BoneWeightPos(0), BoneWeightHeadLength(0), BoneWeightLength(0)
	{}
};

// 网格处理函数声明
void BuildMeshHead(FBXMesh& meshData, message::UGCResSkinnedMeshExtData& extData, MeshBody& body, std::ofstream& osData);
message::UGCResSkinnedMeshExtData BuildMeshExtData(FBXMesh& meshData);
void BuildMeshBody(FBXMesh& meshData, MeshBody& bodyinfo, std::ofstream& osData);
void BuildSingleMesh(FBXMesh& meshData, FBXImportScene& importScene, std::string& filename, const char* outdir, std::string& realPath);
void BuildMeshTxt(FBXMesh& meshData, FBXImportScene& importScene, const char* outdir);
void PrebuildFBXMeshForBlendShape(FBXMesh& meshData);

// 动画处理函数声明
void BuildSingleAnimProtoFile(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir);
void BuildSingleAnimBinaryFile(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir);
void WriteNodeAnimationsToText(FBXImportScene& scene, FBXImportAnimationClip& clip, const char* outdir);

// 骨骼数据处理函数声明
void BuildBoneNodeData(FBXImportScene& scene, FBXImportNode& node, message::UGCResBoneNodeData* parent);

// 调试和解析函数声明
void ParseSingleMesh(std::string meshfile);
void DebugMeshInfo(FBXMesh& meshData, MeshBody& bodyinfo);
void ParseAnimProto(message::UGCResAnimClipData& msg);
void ParseSingleAnim(std::string animfile);
