#pragma once
#include "Utility.h"
#include "FBXImporterDef.h"
#include <wingdi.h>
#include <GL/glu.h>
#define TESS_FUNCTION_CALLCONV CALLBACK

#define EXPORT_DLL extern "C" __declspec(dllexport) 


enum GfxPrimitiveType
{
    kPrimitiveInvalid = -1,

    kPrimitiveTriangles = 0, kPrimitiveTypeFirst = kPrimitiveTriangles,
    kPrimitiveTriangleStrip,
    kPrimitiveQuads,
    kPrimitiveLines,
    kPrimitiveLineStrip,
    kPrimitivePoints, kPrimitiveTypeLast = kPrimitivePoints,

    kPrimitiveForce32BitInt = 0x7fffffff // force 32 bit enum size
};

typedef std::pair<int, GfxPrimitiveType> SubsetKey;
typedef std::map<SubsetKey, int> SubsetLookup;

EXPORT_DLL void ParseFBX(char* fbxpath, char* outdir, char* parameter);

// 新的宽字符支持接口 - 支持特殊字符和宽字符路径
EXPORT_DLL void ParseFBXW(const wchar_t* fbxpath, const wchar_t* outdir, const wchar_t* parameter);

// 字符串转换工具函数
std::string WideToUTF8(const std::wstring& wideStr);
std::wstring UTF8ToWide(const std::string& utf8Str);


