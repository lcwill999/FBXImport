#pragma once
#include <fbxsdk.h>
#include <algorithm>
#include <set>
#include <list>
#include <direct.h>
#include <io.h>
#include "DLLInterface.h"
#include "ProcessFBX.h"
#include "Triangulate.h"
#include "Common.h"
#include "FBXFileIO.h"
#include "MikkTSpace.h"
#include "dynamic_bitset.h"
#include "Components.h"
#include "Constraints.h"


static bool gIsBrokenLightwaveFile = false;
static std::map<FbxNode*, FBXImportNode*> gFBXNodeMap;
static float gGlobalScale = 1.0f;
static int gMeshLimitation = 30000;  // 默认值：90000/3 = 30000


class ConnectedMesh
{
public:
    struct Vertex
    {
        UInt32* faces;
        int  faceCount;
    };

    struct Edge
    {
        UInt32 faces[2];
        SInt8 edgeNos[2];
    };

    enum { kInvalidEdgeIndex = 0xFFFFFFFF };

    /// The list of faces per vertex
    std::vector<Vertex> vertices;
    /// The list of vertices per face
    std::vector<std::vector<UInt32> > faces;

    std::vector<Edge> edges;
    std::vector<UInt32> edgeIndices;
    std::vector<UInt32> faceOffsets;

    ConnectedMesh(int vertexCount, const std::vector<UInt32>& indices, const std::vector<UInt32>& faceSizes);

private:
    void BuildVertexConnections(int vertexCount, const std::vector<UInt32>& indices, const std::vector<UInt32>& faceSizes);
    void BuildEdgeInfo(const std::vector<UInt32>& indices, const std::vector<UInt32>& faceSizes);
    void SortVertexFaceLists();
    bool CheckWinding(const UInt32* faceIndices, UInt32 faceSize, UInt32 edgeVertIndex, UInt32 nextVertIndex, SInt8& outEdgeNo);


    std::vector<UInt32> m_FaceBuffer;
};

ConnectedMesh::ConnectedMesh(int vertexCount, const std::vector<UInt32>& indices, const std::vector<UInt32>& faceSizes)
{
    BuildVertexConnections(vertexCount, indices, faceSizes);
    BuildEdgeInfo(indices, faceSizes);
}

void ConnectedMesh::BuildVertexConnections(int vertexCount, const std::vector<UInt32>& indices, const std::vector<UInt32>& faceSizes)
{
    const int indexCount = indices.size();
    const int faceCount = faceSizes.size();
    Vertex temp; temp.faces = NULL; temp.faceCount = 0;
    vertices.resize(vertexCount, temp);
    m_FaceBuffer.resize(indexCount, 0);
    faces.resize(faceCount);

    // Count faces that use each vertex
    for (int i = 0, idx = 0; i < faceCount; ++i)
    {
        const int fs = faceSizes[i];
        for (int e = 0; e < fs; ++e)
        {
            vertices[indices[idx + e]].faceCount++;
            faces[i].push_back(indices[idx + e]);
        }
        idx += fs;
    }

    // Assign memory to faces (reset faceCount - so we reuse it as a counter)
    int bufferIdx = 0;

    for (int i = 0; i < vertexCount; ++i)
    {
        vertices[i].faces = &m_FaceBuffer[bufferIdx];
        bufferIdx += vertices[i].faceCount;
        vertices[i].faceCount = 0;
    }

    // Assign vertex->face connections
    for (int i = 0, idx = 0; i < faceCount; ++i)
    {
        const int fs = faceSizes[i];
        for (int e = 0; e < fs; ++e)
        {
            int vertexIndex = indices[idx + e];
            vertices[vertexIndex].faces[vertices[vertexIndex].faceCount] = i;
            vertices[vertexIndex].faceCount++;
        }
        idx += fs;
    }
}

void ConnectedMesh::BuildEdgeInfo(const std::vector<UInt32>& indices, const std::vector<UInt32>& faceSizes)
{
    const int indexCount = indices.size();
    const int faceCount = faceSizes.size();

    SortVertexFaceLists();

    faceOffsets.resize(faceCount);
    UInt32 faceOffset = 0;
    for (int face = 0; face < faceCount; ++face)
    {
        faceOffsets[face] = faceOffset;
        faceOffset += faceSizes[face];
    }
    //AssertMsg(faceOffset == indexCount, "Face sizes did not add up correctly");

    // Expected number for a closed manifold
    edges.reserve(indexCount);

    // Edge index per input index
    edgeIndices.resize(indexCount);
    std::fill(edgeIndices.begin(), edgeIndices.end(), kInvalidEdgeIndex);

    UInt32 edgeOffset = 0;
    faceOffset = 0;
    for (int face = 0; face < faceCount; ++face)
    {
        const UInt32 faceSize = faceSizes[face];
        for (UInt32 e = 0; e < faceSize; ++e, ++edgeOffset)
        {
            if (edgeIndices[edgeOffset] != kInvalidEdgeIndex)
                continue;

            const UInt32 vertIndex1 = indices[faceOffset + e];
            const UInt32 vertIndex2 = indices[faceOffset + (e + 1) % faceSize];
            const Vertex& vert1 = vertices[vertIndex1];
            const Vertex& vert2 = vertices[vertIndex2];
            UInt32 vert1FaceIndex = 0;
            UInt32 vert2FaceIndex = 0;
            while (vert1FaceIndex < vert1.faceCount && vert2FaceIndex < vert2.faceCount)
            {
                if (vert1.faces[vert1FaceIndex] < vert2.faces[vert2FaceIndex])
                    ++vert1FaceIndex;
                else if (vert1.faces[vert1FaceIndex] > vert2.faces[vert2FaceIndex])
                    ++vert2FaceIndex;
                else
                {
                    const UInt32 otherFace = vert1.faces[vert1FaceIndex];
                    const UInt32 otherFaceOffset = faceOffsets[otherFace];
                    SInt8 otherEdgeNo;

                    // Add this edge if second face index is bigger (so we don't add edges twice)
                    // and both faces are facing the same direction (in case geometry is double-sided or has weird topology).
                    // Also, double check that the other side wasn't somehow paired already.
                    if (otherFace > face &&
                        CheckWinding(&indices[faceOffsets[otherFace]], faceSizes[otherFace], vertIndex2, vertIndex1, otherEdgeNo) &&
                        edgeIndices[otherFaceOffset + otherEdgeNo] == kInvalidEdgeIndex)
                    {
                        edgeIndices[edgeOffset] = edges.size();
                        edgeIndices[otherFaceOffset + otherEdgeNo] = edges.size();
                        Edge edge;
                        edge.faces[0] = face;
                        edge.faces[1] = otherFace;
                        edge.edgeNos[0] = e;
                        edge.edgeNos[1] = otherEdgeNo;
                        edges.push_back(edge);
                        break;
                    }
                    ++vert1FaceIndex;
                    ++vert2FaceIndex;
                }
            }
        }
        faceOffset += faceSizes[face];
    }
}

void ConnectedMesh::SortVertexFaceLists()
{
    int vertexCount = vertices.size();
    for (int v = 0; v < vertexCount; ++v)
    {
        Vertex& vertex = vertices[v];
        std::sort(vertex.faces, vertex.faces + vertex.faceCount);
    }
}

bool ConnectedMesh::CheckWinding(const UInt32* faceIndices, UInt32 faceSize, UInt32 edgeVertIndex, UInt32 nextVertIndex, SInt8& outEdgeNo)
{
    for (UInt32 e = 0; e < faceSize; ++e)
    {
        if (faceIndices[e] == edgeVertIndex)
        {
            UInt32 eNext = (e + 1) < faceSize ? (e + 1) : 0;
            if (faceIndices[eNext] == nextVertIndex)
            {
                outEdgeNo = SInt8(e);
                return true;
            }
            UInt32 ePrev = e > 0 ? (e - 1) : (faceSize - 1);
            if(faceIndices[ePrev] == nextVertIndex)
                std::cout << "Next vertex not found in CheckWinding()" << std::endl;
            //AssertMsg(faceIndices[ePrev] == nextVertIndex, "Next vertex not found in CheckWinding()");
            return false;
        }
    }
    std::cout << "Vertex not found in CheckWinding()" << std::endl;
    //ErrorString("Vertex not found in CheckWinding()");
    return false;
}


void ZeroPivotsForSkinsRecursive(FbxScene* scene, FbxNode* node)
{
    if (node)
    {
        FbxMesh* mesh = node->GetMesh();
        if (mesh)
        {
            if (mesh->GetDeformerCount(FbxDeformer::eSkin) != 0)
            {
                FbxSkin* skin = (FbxSkin*)mesh->GetDeformer(0, FbxDeformer::eSkin);
                int boneCount = skin->GetClusterCount();

                if (boneCount > 0)
                {
                    FbxVector4 zero(0, 0, 0);
                    node->SetRotationPivot(FbxNode::eSourcePivot, zero);
                    node->SetScalingPivot(FbxNode::eSourcePivot, zero);
                }
            }
        }

        for (int i = 0; i < node->GetChildCount(); i++)
            ZeroPivotsForSkinsRecursive(scene, node->GetChild(i));
    }
}

void ImportVertices(const FbxGeometryBase& fbx, const std::vector<Vector3f>* referenceVertices, std::vector<Vector3f>& vertices, std::vector<int>& invalidVertices)
{
    int vertexCount = fbx.GetControlPointsCount();
    vertices.resize(vertexCount);
    FbxVector4* controlPoints = fbx.GetControlPoints();
    for (int i = 0; i < vertexCount; i++)
    {
        vertices[i] = FBXPointToVector3Remap(controlPoints[i]);
    }
}

void ImportUVs(FbxMesh& fbx, FBXImportMesh& mesh, const int* indices, int polygonIndexCount, const std::vector<uint32_t>& polygonSizes, int polygonCount, int vertexCount, const std::string& meshName)
{
    // Import UV sets (maximum defined by ImportMesh::kMaxUVs)
    int uvsetIndex = 0;

    // First just try importing diffuse UVs from separate layers
    // (Maya exports that way)
    FbxLayerElementUV* firstUVset = NULL;
    FbxLayer* firstUVLayer = NULL;
    for (int i = 0; i < fbx.GetLayerCount(); i++)
    {
        FbxLayer* lay = fbx.GetLayer(i);
        if (!lay)
            continue;
        FbxLayerElementUV* uvs = lay->GetUVs();
        if (!uvs)
            continue;

        if (!firstUVset)
        {
            firstUVset = uvs;
            firstUVLayer = lay;
        }

        ExtractWedgeLayerData(uvs, mesh.uvs[uvsetIndex], indices, polygonIndexCount, polygonSizes, polygonCount, vertexCount, "UV coordinates", meshName, Vector2f(0.0f, 0.0f));
        uvsetIndex++;
        if (uvsetIndex == 2)
            break;
    }
    // If we have received one UV set, check whether the same layer contains an emissive UV set
    // that is different from diffuse UV set.
    // 3dsmax FBX exporters don't export UV sets as different layers, instead for lightmapping usually
    // a material is set up to have lightmap (2nd UV set) as self-illumination slot, and main texture
    // (1st UV set) as diffuse slot.
    if (uvsetIndex == 1 && firstUVset)
    {
        FbxLayerElementUV* secondaryUVs = NULL;
        for (int i = FbxLayerElement::eTextureEmissive; i < FbxLayerElement::eTypeCount; i++)
        {
            secondaryUVs = firstUVLayer->GetUVs((FbxLayerElement::EType)i);
            if (secondaryUVs != NULL)
                break;
        }

        if (secondaryUVs)
        {
            ExtractWedgeLayerData(secondaryUVs, mesh.uvs[uvsetIndex], indices, polygonIndexCount, polygonSizes, polygonCount, vertexCount, "UV coordinates", meshName, Vector2f(0.0f, 0.0f));
            uvsetIndex++;
        }
    }
}

int ConvertFBXToImportMaterial(FbxSurfaceMaterial* fbxMaterial, FBXImportScene& scene, FBXMaterialLookup& fbxMaterialLookup)
{
    FBXMaterialLookup::iterator found = fbxMaterialLookup.find(fbxMaterial);
    if (found != fbxMaterialLookup.end())
        return found->second;

    fbxMaterialLookup[fbxMaterial] = static_cast<int>(scene.materials.size());

    std::string matname = fbxMaterial!= NULL? fbxMaterial->GetName():"";
    scene.materials.push_back(matname);
    return static_cast<int>(scene.materials.size()) - 1;
}

void ConvertFBXMeshMaterials(FbxManager* sdkManager, FbxScene& fbxScene, FbxNode* node, FbxMesh& fbxMesh, const std::string& meshName, FBXImportMesh& mesh, FBXImportScene& scene, int polygonCount, const bool isBrokenLightwaveFile, FBXMaterialLookup& fbxMaterialLookup)
{
    // setup material import
    std::vector<FbxSurfaceMaterial*> materials;
    materials.resize(polygonCount);

    bool hasMaterialClashes = false;

    for (int layerIndex = 0, size = fbxMesh.GetLayerCount(); layerIndex < size; ++layerIndex)
    {
        FbxLayer* layer = fbxMesh.GetLayer(layerIndex);

        // Assign fbx materials to the per face material list
        FbxLayerElementMaterial* materialLayer = layer ? layer->GetMaterials() : NULL;

        if (materialLayer)
        {
            if (materialLayer->GetMappingMode() == FbxLayerElement::eByPolygon)
            {
                // Per polygon -> global index lookups
                if (materialLayer->GetReferenceMode() == FbxLayerElement::eIndex)
                {
                    for (int i = 0; i < polygonCount; i++)
                    {
                        materials[i] = fbxScene.GetMaterial(materialLayer->GetIndexArray().GetAt(i));
                    }
                }
                // Per polygon index -> material ptrs lookup
                else if (materialLayer->GetReferenceMode() == FbxLayerElement::eIndexToDirect)
                {
                    int lNbMaterial = node->GetSrcObjectCount<FbxSurfaceMaterial>();
                    int matIndexArraySize = materialLayer->GetIndexArray().GetCount();
                    int clampSize = polygonCount > matIndexArraySize ? matIndexArraySize : polygonCount;
                    for (int i = 0; i < clampSize; ++i)
                    {
                        int index = materialLayer->GetIndexArray().GetAt(i);
                        if (index >= 0 && index < lNbMaterial)
                        {
                            materials[i] = node->GetMaterial(index);
                        }
                    }
                }
                // Per polygon direct material lookup
                else if (materialLayer->GetReferenceMode() == FbxLayerElement::eDirect)
                {
                    mesh.materials.resize(polygonCount);
                    int clampCount = polygonCount > node->GetMaterialCount() ? node->GetMaterialCount() : polygonCount;
                    for (int i = 0; i < clampCount; i++)
                    {
                        materials[i] = node->GetMaterial(i);
                    }
                }
            }
            else if (materialLayer->GetMappingMode() == FbxLayerElement::eAllSame)
            {
                // Per object -> global material index lookup
                if (materialLayer->GetReferenceMode() == FbxLayerElement::eIndex)
                {
                    if (materialLayer->GetIndexArray().GetCount())
                    {
                        FbxSurfaceMaterial* mat = fbxScene.GetMaterial(materialLayer->GetIndexArray().GetFirst());
                        for (int i = 0; i < polygonCount; i++)
                        {
                            materials[i] = mat;
                        }
                    }
                }
                // Per object -> index to texture ptrs lookup
                else if (materialLayer->GetReferenceMode() == FbxLayerElement::eIndexToDirect)
                {
                    if (materialLayer->GetIndexArray().GetCount() && node->GetMaterialCount())
                    {
                        FbxSurfaceMaterial* mat = node->GetMaterial(materialLayer->GetIndexArray().GetFirst());
                        for (int i = 0; i < polygonCount; i++)
                        {
                            materials[i] = mat;
                        }
                    }
                }
                // Per object -> direct texture ptrs
                else if (materialLayer->GetReferenceMode() == FbxLayerElement::eDirect)
                {
                    if (node->GetMaterialCount())
                    {
                        FbxSurfaceMaterial* mat = node->GetMaterial(0);
                        for (int i = 0; i < polygonCount; i++)
                        {
                            materials[i] = mat;
                        }
                    }
                }
            }
        }
    }

    mesh.materials.resize(polygonCount, -1);
    for (uint32_t i = 0; i < materials.size(); i++)
        mesh.materials[i] = ConvertFBXToImportMaterial(materials[i], scene, fbxMaterialLookup);

}

void ConvertFBXMesh(FbxMesh& fbx, FbxManager* sdkManager, FbxScene& fbxScene, FbxNode* node, FBXImportMesh& mesh, FBXImportScene& scene, FBXMaterialLookup& fbxMaterialLookup)
{
    //FbxMesh& fbx = *node->GetMesh();
    const std::string meshName = static_cast<const char*>(node->GetNameWithoutNameSpacePrefix());
    mesh.name = meshName;
    const int vertexCount = fbx.GetControlPointsCount();
    {
        std::vector<int> invalidVertices;
        ImportVertices(fbx, NULL, mesh.vertices, invalidVertices);
    }
    int polygonCount = fbx.GetPolygonCount();
    const int* indices = fbx.GetPolygonVertices();

    if (polygonCount <= 0)
        return;

    mesh.polygonSizes.resize(polygonCount);
    int polygonIndexCount = 0;
    for (int i = 0; i < mesh.polygonSizes.size(); i++)
    {
        mesh.polygonSizes[i] = fbx.GetPolygonSize(i);
        polygonIndexCount += mesh.polygonSizes[i];
    }
    mesh.polygons.assign(indices, indices + polygonIndexCount);
    
    ImportUVs(fbx, mesh, indices, polygonIndexCount, mesh.polygonSizes, polygonCount, vertexCount, meshName);

    const int kMainLayer = 0;
    FbxLayer* const layer = fbx.GetLayer(kMainLayer);

    if (layer && layer->GetVertexColors())
    {
        ExtractWedgeLayerData(layer->GetVertexColors(), mesh.colors, indices, polygonIndexCount, mesh.polygonSizes, polygonCount, vertexCount, "vertex Colors", meshName);
    }
    if (layer && layer->GetNormals())
    {
        ExtractWedgeLayerData(layer->GetNormals(), mesh.normals, indices, polygonIndexCount, mesh.polygonSizes,
            polygonCount, vertexCount, "normals", meshName);
    }

    ConvertFBXMeshMaterials(sdkManager, fbxScene, node, fbx, meshName, mesh, scene, polygonCount, false, fbxMaterialLookup);

}

void RecursiveImportNodes(FbxManager* fbxManager, FbxScene& fbxScene, FbxNode* node, FBXImportNode& outNode, FBXImportScene& scene, const FBXImportSettings& settings, const FBXImportMeshSetting& meshSettings, FBXMeshToInfoMap& fbxMeshToInfoMap, FBXMaterialLookup& fbxMaterialLookup)
{
    FbxVector4 translation, eulerRotation, scale;
    outNode.eulerRotation = ExtractFBXEulerOld(node->LclRotation.EvaluateValue(0));
    outNode.rotation = ExtractQuaternionFromFBXEulerOld(node->LclRotation.EvaluateValue(0));
    outNode.position = FBXPointToVector3Remap(node->LclTranslation.EvaluateValue(0));
    outNode.scale = FBXPointToVector3(node->LclScaling.EvaluateValue(0));
    outNode.visibility = !CompareApproximately(node->Visibility.Get(), 0.0);

    FbxNode* current = node;
    FbxNode* parent = node->GetParent();
    while (outNode.visibility && parent != NULL && !CompareApproximately(current->VisibilityInheritance.Get(), 0.0))
    {
        outNode.visibility = outNode.visibility && !CompareApproximately(parent->Visibility.Get(), 0.0);
        current = parent;
        parent = current->GetParent();
    }

    //todo: parse user data
    //ExtractCustomProperties(outNode, node, settings);
    outNode.name = node->GetNameWithoutNameSpacePrefix();   
    gFBXNodeMap[node] = &outNode;

    if (node->GetMesh())
    {
        FbxMesh* mesh = node->GetMesh();

        FBXSharedMeshInfo* smInfo = 0;
        FBXMeshToInfoMap::iterator it = fbxMeshToInfoMap.find(mesh);

        if (it != fbxMeshToInfoMap.end())
        {
            smInfo = &it->second;
            // this mesh is an instance but might have different materials than its prototype.
            //if (settings.importMaterials)
            //{
            //	const std::string meshName = static_cast<const char*>(node->GetNameWithoutNameSpacePrefix());
            //	FBXImportMesh importMesh;
            //	ConvertFBXMeshMaterials(fbxManager, fbxScene, *node, *mesh, meshName, importMesh, scene, mesh->GetPolygonCount(), gIsBrokenLightwaveFile, fbxMaterialLookup);
            //	scene.meshInstanceMaterialInfos.insert(std::make_pair(&outNode, importMesh.materials));
            //}
        }
        else
        {
            const int meshIndex = (int)scene.meshes.size();

            std::pair<FBXMeshToInfoMap::iterator, bool> itInsert = fbxMeshToInfoMap.insert(std::make_pair(mesh, FBXSharedMeshInfo()));

            smInfo = &itInsert.first->second;
            smInfo->index = meshIndex;

            scene.meshes.push_back(FBXImportMesh());
            ConvertFBXMesh(*mesh, fbxManager, fbxScene, node, scene.meshes.back(), scene, fbxMaterialLookup);
        }

        smInfo->usedByNodes.push_back(node);
        outNode.meshIndex = smInfo->index;
    }
    else
        outNode.meshIndex = -1;

	if (settings.importCameras || settings.importLights)
		ConvertComponents(node, outNode, scene, settings);

    int count = node->GetChildCount();
    outNode.children.resize(count);
    for (int i = 0; i < count; ++i)
    {
        FbxNode* curChild = node->GetChild(i);
        RecursiveImportNodes(fbxManager, fbxScene, curChild, outNode.children[i], scene,settings,meshSettings,fbxMeshToInfoMap, fbxMaterialLookup);
    }
}

void InvertWinding(FBXImportMesh& mesh)
{
    InvertFaceWindings(mesh.polygons, mesh.polygonSizes);
    InvertFaceWindings(mesh.normals, mesh.polygonSizes);
    InvertFaceWindings(mesh.tangents, mesh.polygonSizes);
	InvertFaceWindings(mesh.colors, mesh.polygonSizes);
    for (int uvIndex = 0; uvIndex < 2; uvIndex++)
    {
        InvertFaceWindings(mesh.uvs[uvIndex], mesh.polygonSizes);
    }
}

void RemoveUnusedVertices(FBXImportMesh& mesh)
{
    dynamic_bitset usedVertices;
    usedVertices.resize(mesh.vertices.size());
    for (int i = 0; i < mesh.polygons.size(); i++)
        usedVertices[mesh.polygons[i]] = true;

    if (usedVertices.count() == mesh.vertices.size())
        return;

    std::vector<int> remap;
    remap.resize(mesh.vertices.size());

    int allocated = 0;
    for (int i = 0; i < mesh.vertices.size(); i++)
    {
        remap[i] = allocated;
        mesh.vertices[allocated] = mesh.vertices[i];

        if (!mesh.skin.empty())
            mesh.skin[allocated] = mesh.skin[i];
        allocated += usedVertices[i];
    }

    for (int i = 0; i < mesh.polygons.size(); i++)
        mesh.polygons[i] = remap[mesh.polygons[i]];
}

void RemoveDegenerateFaces(FBXImportMesh& mesh)
{
    // Check if we have any degenerate faces
    const int faceCount = mesh.polygonSizes.size();
    int i, idx;
    for (i = 0, idx = 0; i < faceCount; ++i)
    {
        const int fs = mesh.polygonSizes[i];
        if (IsDegenerateFace(&mesh.polygons[idx], fs) != kDegenNone)
            break;
        idx += fs;
    }
    // No degenerate faces, return
    if (i == faceCount)
        return;

    FBXImportMesh temp;
    const int indexCount = mesh.polygons.size();
    temp.polygons.reserve(indexCount);
    temp.materials.reserve(faceCount);
    temp.polygonSizes.reserve(faceCount);

    temp.tangents.reserve(indexCount);
    temp.normals.reserve(indexCount);
	temp.colors.reserve(indexCount);
    for (int uvIndex = 0; uvIndex < 2; uvIndex++)
        temp.uvs[uvIndex].reserve(indexCount);


    mesh.hasAnyQuads = false;

    // Build result and remove degenerates on the way.
    for (i = 0, idx = 0; i < faceCount; ++i)
    {
        const int fs = mesh.polygonSizes[i];
        const uint32_t* face = &mesh.polygons[idx];
        DegenFace degen = IsDegenerateFace(face, fs);

        int addFaceSize = fs;
        uint32_t addIndices[4] = { 0, 1, 2, 3 };
        if (degen == kDegenToTri)
        {
            if (face[0] == face[1])
                addIndices[0] = 1, addIndices[1] = 2, addIndices[2] = 3;
            else if (face[1] == face[2])
                addIndices[0] = 0, addIndices[1] = 2, addIndices[2] = 3;
            else if (face[2] == face[3])
                addIndices[0] = 0, addIndices[1] = 1, addIndices[2] = 3;
            else if (face[3] == face[0])
                addIndices[0] = 0, addIndices[1] = 1, addIndices[2] = 2;

            degen = kDegenNone;
            addFaceSize = 3;
        }

        if (degen == kDegenNone)
        {
            if (!mesh.materials.empty())
                temp.materials.push_back(mesh.materials[i]);
            temp.polygonSizes.push_back(addFaceSize);

            AddFace(mesh.polygons, temp.polygons, idx, addIndices, addFaceSize);
            AddFace(mesh.normals, temp.normals, idx, addIndices, addFaceSize);
            AddFace(mesh.tangents, temp.tangents, idx, addIndices, addFaceSize);
			AddFace(mesh.colors, temp.colors, idx, addIndices, addFaceSize);
            for (int uvIndex = 0; uvIndex < 2; uvIndex++)
                AddFace(mesh.uvs[uvIndex], temp.uvs[uvIndex], idx, addIndices, addFaceSize);
            if (addFaceSize == 4)
                mesh.hasAnyQuads = true;
        }
        idx += fs;
    }

    mesh.polygons.swap(temp.polygons);
    mesh.materials.swap(temp.materials);
    mesh.polygonSizes.swap(temp.polygonSizes);
    mesh.normals.swap(temp.normals);
    mesh.tangents.swap(temp.tangents);
	mesh.colors.swap(temp.colors);
    for (int uvIndex = 0; uvIndex < 2; uvIndex++)
        mesh.uvs[uvIndex].swap(temp.uvs[uvIndex]);

}

inline bool CompareBone(const BoneWeights4& lhs, const BoneWeights4& rhs)
{
    for (int i = 0; i < 4; i++)
    {
        if (!CompareApproximately(lhs.weight[i], rhs.weight[i]) || lhs.boneIndex[i] != rhs.boneIndex[i])
            return false;
    }
    return true;
}


int weld(std::vector<Vector3f>& vertices,std::vector<BoneWeights4>&skin, std::vector<int>& remap)
{
    const int NIL = -1;                               // linked list terminator symbol
    int     outputCount = 0;                                // # of output vertices
    int     hashSize = nextPowerOfTwo(vertices.size());                // size of the hash table
    int* hashTable = new int[hashSize + vertices.size()];            // hash table + linked list
    int* next = hashTable + hashSize;             // use bottom part as linked list

    remap.resize(vertices.size());

    memset(hashTable, NIL, (hashSize) * sizeof(int));       // init hash table (NIL = 0xFFFFFFFF so memset works)

    for (int i = 0; i < vertices.size(); i++)
    {
        const Vector3f& v = vertices[i];
        uint32_t          hashValue = GetVector3HashValue(v) & (hashSize - 1);
        int             offset = hashTable[hashValue];
        while (offset != NIL)
        {
            bool euqals = (vertices[offset] == v);

            if (euqals && !skin.empty() && !CompareBone(skin[i], skin[offset]))
                euqals = false;

            if (euqals)
                break;

            offset = next[offset];
        }

        if (offset == NIL)                                  // no match found - copy vertex & add to hash
        {
            remap[i] = outputCount;
            vertices[outputCount] = v;                    // copy vertex
            
            if (!skin.empty())
                skin[outputCount] = skin[i];

            next[outputCount] = hashTable[hashValue]; // link to hash table
            hashTable[hashValue] = outputCount++;        // update hash heads and increase output counter
        }
        else
        {
            remap[i] = offset;
        }
    }

    delete[] hashTable;                                     // cleanup
    if (outputCount < vertices.size())
    {
        vertices.resize(outputCount);
        if (!skin.empty())
            skin.resize(outputCount);
        return true;
    }
    else
        return false;
}

void WeldVertices(FBXImportMesh& mesh)
{
    std::vector<int> remap;
    if (weld(mesh.vertices,mesh.skin, remap))
    {
        uint32_t* indices = &mesh.polygons[0];
        for (int i = 0; i < mesh.polygons.size(); i++)
            indices[i] = remap[indices[i]];
    }
}

static void CleanUpMesh(FBXImportMesh& mesh)
{
    WeldVertices(mesh);
    RemoveUnusedVertices(mesh);
    RemoveDegenerateFaces(mesh);
}

void TransformMesh(FBXImportMesh& mesh, const Matrix4x4f& transform)
{
    // Transform vertices
    for (int i = 0; i < mesh.vertices.size(); i++)
        mesh.vertices[i] = transform.MultiplyPoint3(mesh.vertices[i]);

    // Transform normals
    Matrix3x3f invTranspose = Matrix3x3f(transform);
    invTranspose.InvertTranspose();
    for (int i = 0; i < mesh.normals.size(); i++)
        mesh.normals[i] = NormalizeRobust(invTranspose.MultiplyVector3(mesh.normals[i]));

    for (int i = 0; i < mesh.tangents.size(); i++)
    {
        // we transform and normalize tangent and keep sign of binormal
        const Vector4f& t4 = mesh.tangents[i];

        Vector3f t3(t4.x, t4.y, t4.z);
        //t3 = NormalizeRobust(invTranspose.MultiplyVector3 (t3));
        t3 = NormalizeRobust(transform.MultiplyVector3(t3));        // MM: tangents are not transformed using inverse transposed. This is only for coefficients of plane equations (normals)

        mesh.tangents[i].Set(t3.x, t3.y, t3.z, t4.w);
    }

}

struct SplitMeshImplementation
{
    float normalDotAngle;
    float uvEpsilon;
    const FBXImportMesh& srcMesh;
    FBXImportMesh& dstMesh;

    SplitMeshImplementation(const FBXImportMesh& src, FBXImportMesh& dst, float splitAngle)
        : srcMesh(src),
        dstMesh(dst)
    {
        uvEpsilon = 0.001F;
        normalDotAngle = cos(Deg2Rad(splitAngle)) - 0.001F; // NOTE: subtract is for more consistent results across platforms
        PerformSplit();
    }

    inline void CopyVertexAttributes(int wedgeIndex, int dstIndex)
    {
        if (!srcMesh.normals.empty())
            dstMesh.normals[dstIndex] = srcMesh.normals[wedgeIndex];
		if (!srcMesh.colors.empty())
            dstMesh.colors[dstIndex] = srcMesh.colors[wedgeIndex];
        for (int uvIndex = 0; uvIndex < 2; uvIndex++)
            if (!srcMesh.uvs[uvIndex].empty())
                dstMesh.uvs[uvIndex][dstIndex] = srcMesh.uvs[uvIndex][wedgeIndex];
        if (!srcMesh.tangents.empty())
            dstMesh.tangents[dstIndex] = srcMesh.tangents[wedgeIndex];

        for (size_t i = 0; i < srcMesh.shapes.size(); ++i)
        {
            const FBXImportBlendShape& srcShape = srcMesh.shapes[i];
            FBXImportBlendShape& dstShape = dstMesh.shapes[i];

            if (!srcShape.normals.empty())
                dstShape.normals[dstIndex] = srcShape.normals[wedgeIndex];
            if (!srcShape.tangents.empty())
                dstShape.tangents[dstIndex] = srcShape.tangents[wedgeIndex];
        }

    }

    void AddVertexByIndex(const FBXImportMesh& srcMesh, FBXImportMesh& dstMesh, int srcVertexIndex)
    {
        dstMesh.vertices.push_back(srcMesh.vertices[srcVertexIndex]);
        for (int i = 0; i < srcMesh.shapes.size(); ++i)
            dstMesh.shapes[i].vertices.push_back(srcMesh.shapes[i].vertices[srcVertexIndex]);

        if (!srcMesh.skin.empty())
            dstMesh.skin.push_back(srcMesh.skin[srcVertexIndex]);
    }
    void AddPolygonAttribute(const FBXImportMesh& srcMesh, FBXImportMesh& dstMesh, int srcAttributeIndex)
    {
        if (!srcMesh.normals.empty())
            dstMesh.normals.push_back(srcMesh.normals[srcAttributeIndex]);
        if (!srcMesh.colors.empty())
            dstMesh.colors.push_back(srcMesh.colors[srcAttributeIndex]);
        for (int uvIndex = 0; uvIndex < 2; uvIndex++)
            if (!srcMesh.uvs[uvIndex].empty())
                dstMesh.uvs[uvIndex].push_back(srcMesh.uvs[uvIndex][srcAttributeIndex]);
        if (!srcMesh.tangents.empty())
            dstMesh.tangents.push_back(srcMesh.tangents[srcAttributeIndex]);
        
        for (size_t i = 0; i < srcMesh.shapes.size(); ++i)
        {
            const FBXImportBlendShape& srcShape = srcMesh.shapes[i];
            FBXImportBlendShape& dstShape = dstMesh.shapes[i];

            if (!srcShape.normals.empty())
                dstShape.normals.push_back(srcShape.normals[srcAttributeIndex]);
            if (!srcShape.tangents.empty())
                dstShape.tangents.push_back(srcShape.tangents[srcAttributeIndex]);
        }
    }


    inline void AddVertex(int srcVertexIndex, int srcAttributeIndex)
    {
        AddVertexByIndex(srcMesh, dstMesh, srcVertexIndex);
        AddPolygonAttribute(srcMesh, dstMesh, srcAttributeIndex);
    }

    inline bool NeedTangentSplit(const Vector4f& lhs, const Vector4f& rhs)
    {
        if (Dot(Vector3f(lhs.x, lhs.y, lhs.z), Vector3f(rhs.x, rhs.y, rhs.z)) < normalDotAngle)
            return true;
        return !CompareApproximately(lhs.w, rhs.w);
    }

    inline bool NeedsSplitAttributes(int srcIndex, int dstIndex)
    {
        if (!srcMesh.normals.empty() && Dot(srcMesh.normals[srcIndex], dstMesh.normals[dstIndex]) < normalDotAngle)
            return true;
        if (!srcMesh.colors.empty() && srcMesh.colors[srcIndex] != dstMesh.colors[dstIndex])
            return true;
        for (int uvIndex = 0; uvIndex < 2; uvIndex++)
            if (!srcMesh.uvs[uvIndex].empty() && !CompareApproximately(srcMesh.uvs[uvIndex][srcIndex], dstMesh.uvs[uvIndex][dstIndex], uvEpsilon))
                return true;
        if (!srcMesh.tangents.empty() && NeedTangentSplit(srcMesh.tangents[srcIndex], dstMesh.tangents[dstIndex]))
            return true;
        for (size_t i = 0; i < srcMesh.shapes.size(); ++i)
        {
            const FBXImportBlendShape& srcShape = srcMesh.shapes[i];
            const FBXImportBlendShape& dstShape = dstMesh.shapes[i];

            if (!srcShape.normals.empty() && Dot(srcShape.normals[srcIndex], dstShape.normals[dstIndex]) < normalDotAngle)
                return true;

            if (!srcShape.tangents.empty() && Dot(srcShape.tangents[srcIndex], dstShape.tangents[dstIndex]) < normalDotAngle)
                return true;
        }
        return false;
    }

    // Wedge = a per face vertex. The source mesh has all uvs, normals etc. stored as wedges. Thus srcMesh.normals.size = srcMesh.polygons.size.

    void PerformSplit()
    {
        const int attributeCount = srcMesh.polygons.size();
        dstMesh.Reserve(attributeCount, srcMesh.polygonSizes.size(), &srcMesh);

        // Initialize faces to source faces
        dstMesh.polygons = srcMesh.polygons;
        dstMesh.polygonSizes = srcMesh.polygonSizes;
        dstMesh.hasAnyQuads = srcMesh.hasAnyQuads;

        // Initialize other data
        dstMesh.vertices = srcMesh.vertices;
        dstMesh.materials = srcMesh.materials;
        dstMesh.name = srcMesh.name;
        dstMesh.skin = srcMesh.skin;

        // Initialize attributes to some sane default values
        if (!srcMesh.normals.empty())
            dstMesh.normals.resize(srcMesh.vertices.size(), Vector3f(1.0F, 1.0F, 1.0F));
        if (!srcMesh.colors.empty())
            dstMesh.colors.resize(srcMesh.vertices.size(), ColorRGBA32(0xFFFFFFFF));
        for (int uvIndex = 0; uvIndex < 2; uvIndex++)
        {
            if (!srcMesh.uvs[uvIndex].empty())
                dstMesh.uvs[uvIndex].resize(srcMesh.vertices.size(), Vector2f(0.0F, 0.0F));
        }
        if (!srcMesh.tangents.empty())
            dstMesh.tangents.resize(srcMesh.vertices.size(), Vector4f(1.0f, 0.0f, 0.0f, 0.0F));


        dstMesh.shapes.resize(srcMesh.shapes.size());
        for (size_t i = 0; i < srcMesh.shapes.size(); ++i)
        {
            const FBXImportBlendShape& srcShape = srcMesh.shapes[i];
            FBXImportBlendShape& dstShape = dstMesh.shapes[i];

            dstShape.targetWeight = srcShape.targetWeight;
            dstShape.vertices = srcShape.vertices;
            if (!srcShape.normals.empty())
                dstShape.normals.resize(srcShape.vertices.size(), Vector3f::zAxis);
            if (!srcShape.tangents.empty())
                dstShape.tangents.resize(srcShape.vertices.size(), Vector3f::xAxis);
        }


        typedef std::list<int> AlreadySplitVertices;
        std::vector<AlreadySplitVertices> splitVertices;
        splitVertices.resize(srcMesh.vertices.size());

        int indexCount = dstMesh.polygons.size();
        for (int i = 0; i < indexCount; ++i)
        {
            int vertexIndex = dstMesh.polygons[i];

            // Go through the list of already assigned vertices and find the vertex with the same attributes
            int bestVertexSplitIndex = -1;
            AlreadySplitVertices& possibilities = splitVertices[vertexIndex];
            for (AlreadySplitVertices::iterator s = possibilities.begin(); s != possibilities.end(); s++)
            {
                if (!NeedsSplitAttributes(i, *s))
                {
                    bestVertexSplitIndex = *s;
                }
            }

            // No vertex was found that could be reused!
            if (bestVertexSplitIndex == -1)
            {
                // We haven't visited this vertex at all!
                // Use the original vertex and replace the vertex attribute's with the current wedge attributes
                if (possibilities.empty())
                {
                    bestVertexSplitIndex = vertexIndex;
                    CopyVertexAttributes(i, vertexIndex);
                }
                // We need to add a new vertex
                else
                {
                    bestVertexSplitIndex = dstMesh.vertices.size();
                    AddVertex(vertexIndex, i);
                }

                // Add the vertex to the possible vertices which other wedges can share!
                possibilities.push_back(bestVertexSplitIndex);
            }

            dstMesh.polygons[i] = bestVertexSplitIndex;
        }
    }
};

void SplitMesh(const FBXImportMesh& src, FBXImportMesh& dst, float splitAngle = 1.0F)
{
    SplitMeshImplementation split(src, dst, splitAngle);
}

SubsetLookup getSubsetsFromFaceMaterials(const FBXImportMesh& splitMesh, const std::vector<uint32_t>& faceMaterialIndexes, std::vector<int>& lodMeshMaterials, bool forceTriangles = false)
{
    SubsetLookup subsets;
    for (int i = 0; i < splitMesh.polygonSizes.size(); ++i)
    {
        const int materialIndex = faceMaterialIndexes[i];
        GfxPrimitiveType topo = (!forceTriangles && splitMesh.polygonSizes[i] == 4) ? kPrimitiveQuads : kPrimitiveTriangles;
        SubsetKey key(materialIndex, topo);
        if (subsets.insert(make_pair(key, subsets.size())).second)
            lodMeshMaterials.push_back(materialIndex);
    }
    return subsets;
}

static void FillLodMeshData(const FBXImportMesh& splitMesh, FBXMesh& lodMesh, std::vector<int>& lodMeshMaterials)
{
    int namelen = strlen(splitMesh.name.c_str()) + 1;
    lodMesh.name = new char[namelen];
    strcpy_s(lodMesh.name, namelen, splitMesh.name.c_str());

    lodMesh.vertices.clear();
    lodMesh.vertices.insert(lodMesh.vertices.end(), splitMesh.vertices.begin(), splitMesh.vertices.end());


    lodMesh.colors.clear();
    lodMesh.colors.insert(lodMesh.colors.end(), splitMesh.colors.begin(), splitMesh.colors.end());

    lodMesh.uv1.clear();
    lodMesh.uv1.insert(lodMesh.uv1.end(), splitMesh.uvs[0].begin(), splitMesh.uvs[0].end());
    if (splitMesh.uvs->size() > 1)
    {
        lodMesh.uv2.clear();
        lodMesh.uv2.insert(lodMesh.uv2.end(), splitMesh.uvs[1].begin(), splitMesh.uvs[1].end());
    }
    lodMesh.tangents.clear();
    lodMesh.tangents.insert(lodMesh.tangents.end(), splitMesh.tangents.begin(), splitMesh.tangents.end());
    lodMesh.normals.clear();
    lodMesh.normals.insert(lodMesh.normals.end(), splitMesh.normals.begin(), splitMesh.normals.end());

    lodMeshMaterials.clear();
    lodMesh.indices.clear();
    lodMesh.indicesize.clear();
    lodMesh.topologytype.clear();

    if (splitMesh.materials.empty())
    {
        lodMesh.indices.insert(lodMesh.indices.end(), splitMesh.polygons.begin(), splitMesh.polygons.end());
        lodMesh.indicesize.push_back(1);
        lodMesh.topologytype.push_back(3);
    }
    else
    {
        SubsetLookup subsets = getSubsetsFromFaceMaterials(splitMesh, splitMesh.materials, lodMeshMaterials);

        const int subsetCount = subsets.size();

        // Allocate split faces
        std::vector<std::vector<uint32_t> > splitFaces;
        splitFaces.resize(subsetCount);
        for (int i = 0; i < splitFaces.size(); i++)
            splitFaces[i].reserve(splitMesh.polygons.size());

        // Put faces into the separate per subset array
        std::vector<GfxPrimitiveType> subsetTopology(subsetCount);
        for (int i = 0, idx = 0; i < splitMesh.polygonSizes.size(); ++i)
        {
            const int fs = splitMesh.polygonSizes[i];

            GfxPrimitiveType topo = splitMesh.polygonSizes[i] == 4 ? kPrimitiveQuads : kPrimitiveTriangles;
            SubsetKey key(splitMesh.materials[i], topo);
            int subsetIndex = subsets.find(key)->second;
            subsetTopology[subsetIndex] = topo;
            std::vector<uint32_t>& buffer = splitFaces[subsetIndex];
            for (int j = 0; j < fs; ++j)
                buffer.push_back(splitMesh.polygons[idx + j]);
            idx += fs;
        }

        for (int i = 0; i < subsetCount; ++i)
        {
            lodMesh.indicesize.push_back(splitFaces[i].size());
            lodMesh.topologytype.push_back(subsetTopology[i] == kPrimitiveQuads ? 4 : 3);
            lodMesh.indices.insert(lodMesh.indices.end(), splitFaces[i].begin(), splitFaces[i].end());
        }
        //Add bone weights & bindpose
        lodMesh.boneWeights = splitMesh.skin;
        if (gNodeName2BoneBindePose.find(splitMesh.name) != gNodeName2BoneBindePose.end())
            lodMesh.bindPoses = gNodeName2BoneBindePose[splitMesh.name];
    }
}

int WrapIndex(int value, int modulo)
{
    if (value < 0)
        value += modulo;
    if (value >= modulo)
        value -= modulo;
    //AssertMsg(value >= 0 && value < modulo, "Value in WrapIndex() was outside expected range");
    return value;
}

inline Vector3f RobustNormalFromFace(const Vector3f* vertices, const UInt32* face, int faceSize)
{
    const int kMaxFaceSize = 4;

    if (faceSize < 3 || faceSize > kMaxFaceSize)
    {
        DebugAssertFormatMsg(false, "Invalid faceSize (%d) in RobustNormalFromFace", faceSize);
        return Vector3f::zero;
    }

    faceSize = (std::min)(faceSize, kMaxFaceSize);

    // Both the cross product and Newell's method are very sensitive to the input range,
    // so shift and scale input positions to [0-1] range for robustness.
    // Alternatively we would need to use doubles and smaller epsilons, which is harder
    // to get consistent across different platforms.
    Vector3f normVerts[kMaxFaceSize];
    for (int v = 0; v < faceSize; ++v)
        normVerts[v] = vertices[face[v]];
    
    Vector3f minPos = normVerts[0];
    Vector3f maxPos = normVerts[0];
    for (int v = 1; v < faceSize; ++v)
    {
        minPos.x = (std::min)(minPos.x, normVerts[v].x);
        minPos.y = (std::min)(minPos.y, normVerts[v].y);
        minPos.z = (std::min)(minPos.z, normVerts[v].z);
        maxPos.x = (std::max)(maxPos.x, normVerts[v].x);
        maxPos.y = (std::max)(maxPos.y, normVerts[v].y);
        maxPos.z = (std::max)(maxPos.z, normVerts[v].z);
    }
    
    const float kEpsilon = 0.000001f;
    Vector3f range = maxPos - minPos;
    float maxRange = (std::max)((std::max)(range.x, range.y), range.z);
    float scale = (maxRange > kEpsilon) ? (1.0f / maxRange) : 1.0f;
    
    for (int v = 0; v < faceSize; ++v)
        normVerts[v] = (normVerts[v] - minPos) * scale;

    Vector3f normal = Vector3f::zero;
    if (faceSize > 3)
    {
        // Use Newell's method for polygons
        for (int v = 0; v < faceSize; ++v)
        {
            const Vector3f& curVert = normVerts[v];
            const Vector3f& nextVert = normVerts[WrapIndex(v + 1, faceSize)];
            Vector3f curYZX(curVert.y, curVert.z, curVert.x);
            Vector3f nextYZX(nextVert.y, nextVert.z, nextVert.x);
            Vector3f curZXY(curVert.z, curVert.x, curVert.y);
            Vector3f nextZXY(nextVert.z, nextVert.x, nextVert.y);
            normal += (curYZX - nextYZX) * (curZXY + nextZXY);
        }
    }
    else
    {
        // Use cross product for triangles
        normal = Cross(normVerts[1] - normVerts[0], normVerts[2] - normVerts[0]);
    }

    float sqrLen = Dot(normal, normal);
    const float kSqrLenEpsilon = 0.00001f * 0.00001f;
    if (sqrLen > kSqrLenEpsilon)
    {
        float invSqrtLen = 1.0f / std::sqrt(sqrLen);
        normal = normal * invSqrtLen;
    }
    else
    {
        normal = Vector3f::zero;
    }
    
    return normal;
}
inline Vector3f RobustNormalFromFace_Legacy(const Vector3f* vertices, const UInt32* face, int faceSize)
{
    DebugAssert(faceSize == 3 || faceSize == 4);

    // For a triangle, cross of two edges. For a quad, cross of two diagonals.
    // So one edge is v1-(quad?v3:v0), another is v2-v0
    Vector3f ea = vertices[face[1]] - vertices[face[faceSize == 4 ? 3 : 0]];
    Vector3f eb = vertices[face[2]] - vertices[face[0]];
    Vector3f n = Cross(ea, eb);
    return NormalizeRobust(n);
}
inline float ComputeFaceArea(const std::vector<Vector3f>& vertices, const UInt32* face, int faceSize)
{
    DebugAssert(faceSize == 3 || faceSize == 4);

    // For a triangle, cross of two edges. For a quad, cross of two diagonals.
    // So one edge is v1-(quad?v3:v0), another is v2-v0
    Vector3f ea = vertices[face[1]] - vertices[face[faceSize == 4 ? 3 : 0]];
    Vector3f eb = vertices[face[2]] - vertices[face[0]];
    Vector3f n = Cross(ea, eb);

    // The weight is the surface area of the polygon (or more specifically, twice the surface area).
    // Conveniently, the surface area of a triangle is half the cross product magnitude of two sides,
    // while and the surface area of a quad is half the cross product magnitude of the two diagonals.
    // Since weights only have effect relative to other weights, we don't need to divide by two.
    return Magnitude(n);
}



void GenerateFaceNormals(const FBXImportMesh& mesh, const std::vector<Vector3f>& vertices, std::vector<Vector3f>& outFaceNormals)
{
    const int faceCount = mesh.polygonSizes.size();
    for (int i = 0, idx = 0; i < faceCount; ++i)
    {
        int fs = mesh.polygonSizes[i];
        const UInt32* face = &mesh.polygons[idx];
        Vector3f normal = RobustNormalFromFace(&vertices[0], face, fs);
        outFaceNormals[i] = normal;
        idx += fs;
    }
}

void ClassifySmoothEdgesFromSmoothingGroups(const ConnectedMesh& meshConnection, const std::vector<int>& smoothingGroups, dynamic_bitset& outSmoothEdges)
{
    for (size_t e = 0; e < meshConnection.edges.size(); ++e)
    {
        const ConnectedMesh::Edge& edge = meshConnection.edges[e];

        const UInt32 face1Index = meshConnection.faces[edge.faces[0]][edge.edgeNos[0]];
        const UInt32 face2Index = meshConnection.faces[edge.faces[1]][edge.edgeNos[1]];

        outSmoothEdges[e] = smoothingGroups[face1Index] & smoothingGroups[face2Index];
    }
}

void ClassifySmoothEdgesFromAngle(const ConnectedMesh& meshConnection, const std::vector<Vector3f>& faceNormals, const float hardAngle, dynamic_bitset& outSmoothEdges)
{
    const float hardDot = cos(Deg2Rad(hardAngle)) - 0.001F; // NOTE: subtract is for more consistent results across platforms

    for (size_t e = 0; e < meshConnection.edges.size(); ++e)
    {
        const ConnectedMesh::Edge& edge = meshConnection.edges[e];
        const Vector3f& faceNormal1 = faceNormals[edge.faces[0]];
        const Vector3f& faceNormal2 = faceNormals[edge.faces[1]];
        const float dp = Dot(faceNormal1, faceNormal2);
        outSmoothEdges[e] = (dp > hardDot);
    }
}

void ComputeAreaWeights(const FBXImportMesh& mesh, const std::vector<Vector3f>& vertices, std::vector<float>& outFaceAreaWeights)
{
    const int faceCount = mesh.polygonSizes.size();
    for (int i = 0, idx = 0; i < faceCount; ++i)
    {
        int faceSize = mesh.polygonSizes[i];
        const UInt32* face = &mesh.polygons[idx];
        float weight = ComputeFaceArea(vertices, face, faceSize);
        outFaceAreaWeights[i] = weight;
        idx += faceSize;
    }
}

void ComputeAngleWeights(const size_t indexCount, const size_t faceCount, const FBXImportMesh& mesh, std::vector<float>& angles)
{
    for (size_t face = 0, faceOffset = 0; face < faceCount; ++face)
    {
        const UInt32 faceSize = mesh.polygonSizes[face];
        UInt32 prevVertexIndex = mesh.polygons[faceOffset + faceSize - 1];
        for (UInt32 v = 0; v < faceSize; ++v)
        {
            UInt32 vertexIndex = mesh.polygons[faceOffset + v];
            UInt32 nextVertexIndex = mesh.polygons[faceOffset + WrapIndex(v + 1, faceSize)];
            const Vector3f vecToPrevVertex = mesh.vertices[prevVertexIndex] - mesh.vertices[vertexIndex];
            const Vector3f vecToNextVertex = mesh.vertices[nextVertexIndex] - mesh.vertices[vertexIndex];
            angles[faceOffset + v] = Angle(vecToPrevVertex, vecToNextVertex);
            prevVertexIndex = vertexIndex;
        }
        faceOffset += faceSize;
    }
}


void ComputeNormals(const size_t indexCount,
    const size_t faceCount,
    const FBXImportMesh& mesh,
    const std::vector<float>& angleWeights,
    const std::vector<Vector3f>& faceNormals,
    const std::vector<float>& faceAreaWeights,
    const ConnectedMesh& meshConnection,
    const dynamic_bitset& smoothEdges,
    std::vector<Vector3f>& outNormals)
{
    const UInt32 kNotComputed = ~UInt32(0);
    UInt32 indexOffset = 0;
    std::vector<UInt32> alreadyComputed(indexCount, kNotComputed);
    for (size_t face = 0, faceOffset = 0; face < faceCount; ++face)
    {
        const UInt32 faceSize = mesh.polygonSizes[face];
        for (UInt32 v = 0; v < faceSize; ++v, ++indexOffset)
        {
            if (alreadyComputed[indexOffset] != kNotComputed)
            {
                // Take already computed result as an optimization and to avoid numerical differences
                // for normals that should be identical (because they were added up in different order)
                outNormals[indexOffset] = outNormals[alreadyComputed[indexOffset]];
                continue;
            }

            // Contribution from first face normal
            const UInt32 vertexIndex = mesh.polygons[faceOffset + v];
            const float angleWeight = angleWeights[faceOffset + v];
            Vector3f normalSum = faceNormals[face] * angleWeight * faceAreaWeights[face];

            // Loop around vertex in both directions until hitting a hard edge or completing the loop
            bool vertexCompleted = false;
            for (int clockwise = 0; !vertexCompleted && clockwise < 2; ++clockwise)
            {
                const UInt32 startEdgeIndex = meshConnection.edgeIndices[faceOffset + WrapIndex(int(v) - clockwise, faceSize)];
                UInt32 curFace = face;
                UInt32 curEdgeIndex = startEdgeIndex;
                while (!vertexCompleted && curEdgeIndex != ConnectedMesh::kInvalidEdgeIndex)
                {
                    if (!smoothEdges[curEdgeIndex])
                        break;

                    const ConnectedMesh::Edge& edge = meshConnection.edges[curEdgeIndex];
                    for (size_t side = 0; side < 2; ++side)
                    {
                        if (edge.faces[side] != curFace)
                        {
                            curFace = edge.faces[side];
                            if (curFace == face)
                            {
                                // We're back to where we started, don't walk in opposite direction
                                vertexCompleted = true;
                                break;
                            }
                            const UInt32 curFaceOffset = meshConnection.faceOffsets[curFace];
                            const UInt32 curFaceSize = mesh.polygonSizes[curFace];
                            const int curVertNo = WrapIndex(edge.edgeNos[side] + (clockwise ? 0 : 1), curFaceSize);
                            //AssertMsg(mesh.polygons[curFaceOffset + curVertNo] == vertexIndex, "Unexpected vertex index while generating normals");

                            // Add contribution from face normal, mark current index as already computed
                            const UInt32 curIndexOffset = curFaceOffset + curVertNo;
                            const float curAngleWeight = angleWeights[curIndexOffset];
                            normalSum += faceNormals[curFace] * curAngleWeight * faceAreaWeights[curFace];
                            alreadyComputed[curIndexOffset] = indexOffset;

                            // Step to next triangle edge surrounding vertex
                            int edgeNo = WrapIndex(int(edge.edgeNos[side]) + (clockwise ? -1 : 1), curFaceSize);
                            curEdgeIndex = meshConnection.edgeIndices[curFaceOffset + edgeNo];
                            break;
                        }
                    }
                }
            }
            outNormals[indexOffset] = NormalizeRobust(normalSum);
        }
        faceOffset += faceSize;
    }

    for (size_t i = 0; i < indexCount; ++i)
    {
        outNormals[i] = NormalizeRobust(outNormals[i]);
    }
}

void ComputeNormals_Legacy(const size_t indexCount,
    const size_t faceCount,
    const FBXImportMesh& mesh,
    const std::vector<Vector3f>& vertices,
    const ConnectedMesh& meshConnection,
    const float hardAngle,
    std::vector<Vector3f>& outNormals)
{
    // generate the face normals using the legacy functions, since the SIMD function RobustNormalFromFace
    // produces slightly different results which can fail the asset import tests
    std::vector<Vector3f> faceNormals(faceCount);
    for (int i = 0, idx = 0; i < faceCount; ++i)
    {
        int fs = mesh.polygonSizes[i];
        const UInt32* face = &mesh.polygons[idx];
        const Vector3f& normal = RobustNormalFromFace_Legacy(&vertices[0], face, fs);
        faceNormals[i] = normal;
        idx += fs;
    }

    float hardDot = cos(Deg2Rad(hardAngle)) - 0.001F; // NOTE: subtract is for more consistent results across platforms

    for (int faceIdx = 0, idx = 0; faceIdx < faceCount; ++faceIdx)
    {
        const int faceSize = mesh.polygonSizes[faceIdx];
        const UInt32* face = &mesh.polygons[idx];
        const Vector3f& faceNormal = faceNormals[faceIdx];
        for (int vertexIdx = 0; vertexIdx < faceSize; ++vertexIdx)
        {
            Vector3f calculateNormal = Vector3f::zero;

            const ConnectedMesh::Vertex& connected = meshConnection.vertices[face[vertexIdx]];
            for (int connectedFaceIdx = 0; connectedFaceIdx < connected.faceCount; ++connectedFaceIdx)
            {
                const Vector3f& connectedFaceNormal = faceNormals[connected.faces[connectedFaceIdx]];
                float dp = Dot(faceNormal, connectedFaceNormal);
                if (dp > hardDot)
                {
                    calculateNormal += connectedFaceNormal;
                }
            }

            outNormals[idx + vertexIdx] = NormalizeRobust(calculateNormal);
        }
        idx += faceSize;
    }
}


static void GenerateNormals(const FBXImportMesh& mesh,
    const std::vector<Vector3f>& vertices,
    NormalSmoothingSourceOptions normalSmoothingSource,
    const float hardAngle,
    NormalCalculationOptions normalCalculationMode,
    std::vector<Vector3f>& normals)
{
    ConnectedMesh meshConnection(mesh.vertices.size(), mesh.polygons, mesh.polygonSizes);

    const size_t indexCount = mesh.polygons.size();
    const size_t faceCount = mesh.polygonSizes.size();
    const size_t edgeCount = meshConnection.edges.size();

    normals.resize(indexCount);
    std::fill(normals.begin(), normals.end(), Vector3f(0, 0, 0));

    if (normalCalculationMode != kNormalCalculationOptionsUnweighted_Legacy)
    {
        std::vector<Vector3f> faceNormals(faceCount);
        GenerateFaceNormals(mesh, vertices, faceNormals);

        dynamic_bitset smoothEdges(edgeCount, 0);
        if (normalSmoothingSource == kNormalSmoothingSourceOptionsFromSmoothingGroups || (normalSmoothingSource == kNormalSmoothingSourceOptionsPreferSmoothingGroups && !mesh.smoothingGroups.empty()))
        {
            if (!mesh.smoothingGroups.empty())
                ClassifySmoothEdgesFromSmoothingGroups(meshConnection, mesh.smoothingGroups, smoothEdges);
        }
        else if (normalSmoothingSource == kNormalSmoothingSourceOptionsFromAngle || (normalSmoothingSource == kNormalSmoothingSourceOptionsPreferSmoothingGroups && mesh.smoothingGroups.empty()))
        {
            ClassifySmoothEdgesFromAngle(meshConnection, faceNormals, hardAngle, smoothEdges);
        }

        // initialize the angles and area weights to 1.0f so we can use the same normal computation function for all types of weighted/unweighted normals
        std::vector<float> angleWeights(indexCount, 1.0f);
        std::vector<float> faceAreaWeights(faceCount, 1.0f);

        switch (normalCalculationMode)
        {
        case kNormalCalculationOptionsAreaWeighted:
            ComputeAreaWeights(mesh, vertices, faceAreaWeights);
            break;
        case kNormalCalculationOptionsAngleWeighted:
            ComputeAngleWeights(indexCount, faceCount, mesh, angleWeights);
            break;
        case kNormalCalculationOptionsAreaAndAngleWeighted:
            ComputeAreaWeights(mesh, vertices, faceAreaWeights);
            ComputeAngleWeights(indexCount, faceCount, mesh, angleWeights);
            break;
        case kNormalCalculationOptionsUnweighted:
        default:
            break;
        }

        ComputeNormals(indexCount, faceCount, mesh, angleWeights, faceNormals, faceAreaWeights, meshConnection, smoothEdges, normals);
    }
    else
    {
        ComputeNormals_Legacy(indexCount, faceCount, mesh, vertices, meshConnection, hardAngle, normals);
    }
}


static void GenerateNormals(FBXImportMesh& mesh, NormalSmoothingSourceOptions normalSmoothingSource, const float hardAngle, NormalCalculationOptions normalCalculationMode)
{
    GenerateNormals(mesh, mesh.vertices, normalSmoothingSource, hardAngle, normalCalculationMode, mesh.normals);

    // Generate normals for blendShapes
    for (int i = 0; i < mesh.shapes.size(); ++i)
    {
        FBXImportBlendShape& shape = mesh.shapes[i];
        if (!shape.vertices.empty())
            GenerateNormals(mesh, shape.vertices, normalSmoothingSource, hardAngle, normalCalculationMode, shape.normals);
    }
}

void CalulateNormals(FBXImportMesh& tmpMesh,FBXImportMeshSetting& settings)
{
    if (settings.normalImportMode == kNormalOptionsNone)
    {
        if (!tmpMesh.normals.empty())
        {
            //AssertString(Format("Mesh %s should have no normals", meshName.c_str()));
            tmpMesh.normals.clear();

            // Make sure shapes have also no normals.
            for (int i = 0; i < tmpMesh.shapes.size(); ++i)
            {
                FBXImportBlendShape& shape = tmpMesh.shapes[i];

                if (!shape.normals.empty())
                {
                    //AssertString(Format("Shape #%d of mesh %s shouldn't have normals", i, meshName.c_str()));
                    shape.normals.clear();
                }
            }
        }
    }
    else
    {
        if (settings.legacyComputeAllNormalsFromSmoothingGroupsWhenMeshHasBlendShapes)
        {
            const bool normalCountMismatch = tmpMesh.normals.size() != tmpMesh.polygons.size();

            if (settings.normalImportMode == kNormalOptionsCalculate || normalCountMismatch)
            {
                // only need to clean up early if normals are missing. Otherwise mikktspace
                // prefers the mesh has not been subject to changes before tangent space generation.
                CleanUpMesh(tmpMesh);

                // Legacy: recompute ALL normals, including blendshape normals
                GenerateNormals(tmpMesh, kNormalSmoothingSourceOptionsFromAngle, settings.normalSmoothAngle, settings.normalCalculationMode);
            }
        }
        else
        {
            const bool normalCountMismatch = tmpMesh.normals.size() != tmpMesh.polygons.size();

            if (settings.normalImportMode == kNormalOptionsCalculate || normalCountMismatch)
            {
                // only need to clean up early if normals are missing. Otherwise mikktspace
                // prefers the mesh has not been subject to changes before tangent space generation.
                CleanUpMesh(tmpMesh);

                GenerateNormals(tmpMesh, tmpMesh.vertices, settings.normalSmoothingSource, settings.normalSmoothAngle, settings.normalCalculationMode,
                    tmpMesh.normals);
            }

            for (int i = 0; i < tmpMesh.shapes.size(); ++i)
            {
                FBXImportBlendShape& shape = tmpMesh.shapes[i];

                const bool normalCountMismatch = tmpMesh.normals.size() != shape.normals.size();

                if (settings.blendShapeNormalImportMode == kNormalOptionsCalculate || (settings.blendShapeNormalImportMode == kNormalOptionsImport && normalCountMismatch))
                {
                    GenerateNormals(tmpMesh, shape.vertices, settings.normalSmoothingSource, settings.normalSmoothAngle,
                        settings.normalCalculationMode, shape.normals);
                }
            }
        }
    }
}

void GenerateMeshData(const FBXImportMesh& constantMesh, const Matrix4x4f& transform, const std::string& lodMeshName, FBXMesh& lodMesh, 
    std::vector<int>& lodMeshMaterials, char* outdir)
{
    const FBXImportMesh* currentMesh = &constantMesh;
    FBXImportMesh tmpMesh;

    FBXImportMeshSetting importmeshsetting1;
    importmeshsetting1.importNormal = true;
    importmeshsetting1.importTangent = false;

    Triangulate(*currentMesh, tmpMesh, importmeshsetting1, true);
    InvertWinding(tmpMesh);


    //ReGenerate Normals while missing
    CalulateNormals(tmpMesh, importmeshsetting1);
    //CreateTangent if need


    if(!tmpMesh.normals.empty())
        GenerateMikkTSpace(tmpMesh);  
    FBXImportMesh originalMesh;

    FBXImportMeshSetting importmeshsetting2;
    importmeshsetting2.importNormal = true;
    importmeshsetting2.normalImportMode = kNormalOptionsImport;
    importmeshsetting2.importTangent = true;
    importmeshsetting2.tangentImportMode = kTangentSpaceOptionsImport;
    Triangulate(tmpMesh, originalMesh, importmeshsetting2, false);

    CleanUpMesh(originalMesh);
    TransformMesh(originalMesh, transform);
    FBXImportMesh splitMesh;    
    SplitMesh(originalMesh, splitMesh);
    FillLodMeshData(splitMesh, lodMesh, lodMeshMaterials);
}

void InstantiateImportMesh(int index/*,const Matrix4x4f& transform Identity*/, FBXGameObject& gameobject,  FBXImportScene& importScene, char* outdir)
{
    Matrix4x4f scale;
    float globalScale = importScene.fileScaleFactor;
    scale.SetScale(Vector3f(globalScale, globalScale, globalScale));

    FBXImportMesh splitMesh;
    std::vector<int> lodMeshesMaterials;
    FBXMesh lodMesh;
    GenerateMeshData(importScene.meshes[index], scale, importScene.meshes[index].name, lodMesh, lodMeshesMaterials, outdir);
    lodMesh.materialindex.insert(lodMesh.materialindex.end(), lodMeshesMaterials.begin(), lodMeshesMaterials.end());
    for (int i = 0; i < lodMeshesMaterials.size(); i++)
    {
        lodMesh.materials.push_back(importScene.materials[lodMeshesMaterials[i]]);
        
    }
    gameobject.meshList.push_back(lodMesh);
}




//void GetNodePath(FBXImportNode& parent,std::string path = "")
//{
//	Bone bone;
//	bone.name = parent.name;
//	bone.localposition = parent.position;
//	bone.localscale = parent.scale;
//	bone.localRotation = parent.rotation;
//	gPath2Bone.insert(std::pair<std::string, Bone>(path, bone));
//    std::cout << bone.name << "contains mesh id = " << parent.meshIndex << std::endl;
//    for (auto i = 0; i < parent.children.size(); i++)
//    {
//        auto node = parent.children[i];
//        //std::string path = parent.name + "/" + node.name;
//        std::string newPath = path + "/" + node.name;
//        std::cout << "Get Node Path: " << newPath << std::endl;
//
//        GetNodePath(node, newPath);
//    }
//
//}

void DisplayBones(FBXImportScene& scene)
{
	auto mesh = scene.meshes;
	auto animclips = scene.animationClips;
	std::cout << "Mesh count is  " << mesh.size() << std::endl;
	std::cout << "animclips count is  " << animclips.size() << std::endl;
    std::set<std::string> nameSet;
	for (int i = 0; i < mesh.size(); i++)
	{
		auto bones = mesh[i].bones;
		std::cout << mesh[i].name << " Has " << bones.size() << " bones"<<std::endl;
        for (int j = 0;j< bones.size();j++)
        {
            std::cout << "   The [" << j << "] bone name:" << bones[j].node->name << std::endl;
            nameSet.insert(bones[j].node->name);
        }        
	}
    std::cout << "the count is " << nameSet.size() << std::endl;
    int index = 0;
    for (auto it = nameSet.begin(); it != nameSet.end(); it++)
    {
        std::cout << "Index [ " << index << " ] is " << it->c_str() << std::endl;
        index++;
    }

    //std::cout << "gFBXNodeMap count is  " << gFBXNodeMap.size() << std::endl;
    //for (auto it = gFBXNodeMap.begin(); it != gFBXNodeMap.end(); it++)
    //{
    //    std::cout << "gFBXNodeMap:" << it->second->name << std::endl;
    //}


    auto node = scene.nodes;
    std::cout << "Node count: " << node.size() << std::endl;
	for (int i = 0; i < node.size(); i++)
	{
		auto childrens = node[i].children;
		std::cout <<"Node: " << node[i].name << " Has " << childrens.size() << " childrens" << std::endl;
	}
 //   gPath2Bone.clear();
 //   for (auto i = 0; i < node.size(); i++)
	//{
	//	GetNodePath(node[i], node[i].name);
 //   }

    auto anim = scene.animationClips;
    for (auto i = 0;i<anim.size();i++)
    {
        auto clip = anim[i];
        auto floatcurves = clip.floatAnimations;
        auto nodecurves = clip.nodeAnimations;
    }
}


void BuildImportSetting(FBXImportSettings& settings)
{
    settings.importBlendShapes = true;//m_ImportBlendShapes;
    settings.importCameras = true;//m_ImportCameras;
    settings.importLights = true; //m_ImportLights;
    settings.importAnimations = true;//ShouldImportAnimations();
    settings.importMaterials = true;// m_ImportMaterials;
    settings.resampleCurves = true;// m_ResampleCurves;
    settings.importSkinMesh = true;// ShouldImportSkinnedMesh();
	// TODO@MECANIM validate new conditions to adjust clips by time range
    settings.adjustClipsByTimeRange = false;// ShouldAdjustClipsByDeprecatedTimeRange();
    settings.importVisibility = true;// m_ImportVisibility;
    settings.useFileScale = true;// m_UseFileScale;
    settings.importAnimatedCustomProperties = false;// m_ImportAnimatedCustomProperties;
	//settings.extraUserProperties = m_ExtraUserProperties;
    settings.importConstraints = false;// m_ImportConstraints;
}



void ParseFBXScene(FbxManager* fbxManager, FbxScene& fbxScene, char* outdir)
{
    gFBXNodeMap.clear();
    FBXImportScene outputScene;
    ProcessFBXImport(fbxManager, &fbxScene, outputScene, gGlobalScale);
	
    FBXMeshToInfoMap fbxMeshToInfoMap;
	FBXMaterialLookup fbxMaterialLookup;
	FBXImportSettings settings;
	FBXImportMeshSetting meshSettings;

    BuildImportSetting(settings);

    FbxNode* root = fbxScene.GetRootNode();
    outputScene.nodes.resize(root->GetChildCount());

    for (int i = 0, c = root->GetChildCount(); i < c; ++i)
    {
        RecursiveImportNodes(fbxManager, fbxScene, root->GetChild(i), outputScene.nodes[i], outputScene,settings,meshSettings,fbxMeshToInfoMap, fbxMaterialLookup);
        if (root->GetChild(i)->GetSkeleton())
        {
            auto ptr = root->GetChild(i)->GetSkeleton();
            outputScene.sceneInfo.hasSkeleton = true;
        }

    }

    //WriteSceneOutputFiles(outputScene, outdir);

    //WriteMeshFileNew(&gameObject, outputScene, outdir);

    //2023.7.26 Add anim parse
	// BlendShapes have to be imported after we import meshes, because we may need smoothing groups
	if (settings.importBlendShapes)
	    ImportAllBlendShapes(meshSettings, fbxMeshToInfoMap, outputScene);

	if (settings.importConstraints)
	{
		ImportConstraints(*fbxManager, fbxScene, root, outputScene);
	}

	if (settings.importCameras)
	{
		for (int i = 0, c = fbxScene.GetSrcObjectCount<FbxCamera>(); i < c; ++i)
		{
			FbxCamera* cameraComponent = fbxScene.GetSrcObject<FbxCamera>(i);
			FbxNode* fbxCameraNode = cameraComponent->GetNode();
			if (!fbxCameraNode)
				continue;  // when importing stereo camera not every camera has an fbxNode
			FBXImportNode* importCameraNode = gFBXNodeMap[fbxCameraNode];
			if (!importCameraNode || importCameraNode->cameraIndex == -1)
				continue; // if we ignore stereo camera the cameraIndex will be -1 see case 1063235
			FBXImportCameraComponent& importCameraComponent = outputScene.cameras[importCameraNode->cameraIndex];
			importCameraComponent.owner = importCameraNode;
			FbxNode* target = fbxCameraNode->GetTarget();
			if (target)
			{
				importCameraComponent.target = gFBXNodeMap[target];
				//Assert(importCameraComponent.target != NULL);
				FbxNode* up = fbxCameraNode->GetTargetUp();
				if (up)
				{
					importCameraComponent.up = gFBXNodeMap[up];
					//Assert(importCameraComponent.up != NULL);
				}
				else
				{
					importCameraComponent.roll = cameraComponent->Roll.EvaluateValue(0);
				}
			}
		}
	}

	if (settings.importAnimations)
	{
		AnimationSettings animationSettings;
		animationSettings.adjustClipsByTimeRange = (settings.adjustClipsByTimeRange != 0);
		animationSettings.animationOversampling = settings.resampleCurves ? 1 : 0;
		animationSettings.importBlendShapes = settings.importBlendShapes;
		animationSettings.importAnimatedCustomProperties = settings.importAnimatedCustomProperties;
		animationSettings.importVisibility = settings.importVisibility;
		animationSettings.importCameras = settings.importCameras;
		animationSettings.importLights = settings.importLights;
		animationSettings.importConstraints = settings.importConstraints;
		animationSettings.importMaterials = settings.importMaterials;
		animationSettings.sampleRate = 1.0 / outputScene.sceneInfo.sampleRate;
		ImportAllAnimations(*fbxManager, fbxScene, root, outputScene, gFBXNodeMap, fbxMeshToInfoMap, animationSettings);
	}

	// We import skin after all nodes are imported
	if (settings.importSkinMesh)
	{
		//Assert(scene.meshes.size() == fbxMeshToInfoMap.size());

		// We need to put meshes in stable order (because 3dAssetImport tests depend on that)
		std::vector<FbxMesh*> orderedFbxMeshToInfoMap(fbxMeshToInfoMap.size(), 0);
		for (FBXMeshToInfoMap::const_iterator it = fbxMeshToInfoMap.begin(), end = fbxMeshToInfoMap.end(); it != end; ++it)
		{
			//Assert(it->second.index < scene.meshes.size());
			orderedFbxMeshToInfoMap[it->second.index] = it->first;
		}

		for (unsigned int i = 0; i < orderedFbxMeshToInfoMap.size(); ++i)
		{
			FbxMesh* mesh = orderedFbxMeshToInfoMap[i];
			//Assert(mesh);
			ImportSkin(mesh, outputScene.meshes[i], gFBXNodeMap, fbxMeshToInfoMap);
		}
	}


    BuildAllBoneNameMap(outputScene);
    BuildMeshBoneRefMap(outputScene);
    BuildBlendShapeBoneMap(outputScene);

    //Build Mesh
    FBXGameObject gameObject;
    std::vector<std::string> outIndexMesh;
    outIndexMesh.clear();
    for (int i = 0; i < outputScene.meshes.size(); i++)
    {
        InstantiateImportMesh(i, gameObject, outputScene, outdir);

        if (outputScene.meshes[i].polygons.size() > gMeshLimitation * 3)
        {
            outIndexMesh.push_back(outputScene.meshes[i].name);
        }
    }

    if (outIndexMesh.size() > 0)
    {
        WriteManifestErrorInput(outdir, gFBXFileName,outIndexMesh);
        std::cout << "Error : Triangles Out of Limitation! " << outIndexMesh.size() << " mesh(es) exceeded the limit." << std::endl;
    }
    else
    {
        gameObject.meshCount = outputScene.meshes.size();

        //WriteMeshFileNew(&gameObject, outputScene, outdir);
        WriteMeshAllFile(&gameObject, outputScene, outdir, gFBXFileName);

        if (outputScene.sceneInfo.hasSkeleton || (gNodeName2BoneBindePose.size()!=0 && gNodeName2BoneName.size()!=0))
        {
            WriteSkeletonProtoBuf(outputScene, outdir, gFBXFileName.c_str());
        }
        else
        {
            if (outputScene.animationClips.size()>0)
            {
                BuildNodePath2NameForAnim(outputScene);
            }
        }
        WriteAnimClipProtoBuf(outputScene, outdir);
    }
    //Clear Path
    std::string outputPath(outdir);
    std::wstring outputfilenameW = ConvertUTF8ToWide(outputPath);
    DeleteEmptyFolders(outputPath);   
}



void Stringsplit(const std::string& str, const std::string& splits, std::vector<std::string>& res)
{
    if (str == "")	
        return;

    std::string strs = str + splits;
    size_t pos = strs.find(splits);
    int step = splits.size();

    while (pos != strs.npos)
    {
        std::string temp = strs.substr(0, pos);
        res.push_back(temp);
        strs = strs.substr(pos + step, strs.size());
        pos = strs.find(splits);
    }
}



void SplitParameter(char* parameter)
{
    if (parameter == nullptr)
        return;

    std::string pUTF8 = CodeTUTF8(parameter, CP_ACP);
    std::vector<std::string> plist;
    Stringsplit(pUTF8, ";", plist);
    for (int i = 0; i < plist.size(); i++)
    {
        std::vector<std::string> pResult;
        Stringsplit(plist[i], "=", pResult);
        if (pResult[0] == "globalscale")
        {
            gGlobalScale = atof(pResult[1].c_str());
        }
        if (pResult[0] == "enabletangent")
        {
            gOutPutTangent = true;
        }
        if (pResult[0] == "usefbxname")
        {
            gAnimUseFBXName = true;
        }
        if (pResult[0] == "meshlimitation")
        {
            gMeshLimitation = atoi(pResult[1].c_str());
        }
    }
}




void ParseFBX(char* fbxpath, char* outdir, char* paramater)
{
    FbxManager* lSdkManager = NULL;
    FbxScene* lScene = NULL;
    InitializeSdkObjects(lSdkManager, lScene);

    std::string fbxUTF8 = CodeTUTF8(fbxpath, CP_ACP);
    FbxString lFilePath(fbxUTF8.c_str());

	std::string::size_type iPos = (fbxUTF8.find_last_of('\\') + 1) == 0 ? fbxUTF8.find_last_of('/') + 1 : fbxUTF8.find_last_of('\\') + 1;
	std::string ImgName = fbxUTF8.substr(iPos, fbxUTF8.length() - iPos);
	gFBXFileName = ImgName.substr(0, ImgName.rfind("."));
	
    SplitParameter(paramater);

    if (!lFilePath.IsEmpty())
    {
        bool lResult = LoadScene(lSdkManager, lScene, lFilePath.Buffer());
        if (lResult == false)
        {
            std::cout << " File Type is Error " << std::endl;
        }
        else
        {
            if (outdir != nullptr) {                
                gOutPutDir = CodeTUTF8(outdir, CP_ACP);
                // 统一路径分隔符为反斜杠
                std::replace(gOutPutDir.begin(), gOutPutDir.end(), '/', '\\');
                gOutPutDirSantinized = gOutPutDir;  // 保持原始路径，不进行清理
            }
            else {
                gOutPutDir.clear();
                gOutPutDirSantinized.clear();
            }
            // gOutPutDirSantinized已经是UTF-8编码，直接使用
            std::vector<char> pathOutDir(gOutPutDirSantinized.begin(), gOutPutDirSantinized.end());
            pathOutDir.push_back('\0');
            ParseFBXScene(lSdkManager, *lScene, pathOutDir.data());
        }
    }
    else
    {
        std::cout << " File Type is Error " << std::endl;
    }
}

// 字符串转换工具函数实现
std::string WideToUTF8(const std::wstring& wideStr) {
    if (wideStr.empty()) return std::string();
    
    int sizeNeeded = WideCharToMultiByte(CP_UTF8, 0, wideStr.c_str(), -1, NULL, 0, NULL, NULL);
    if (sizeNeeded <= 0) return std::string();
    
    std::string utf8Str(sizeNeeded - 1, 0);
    WideCharToMultiByte(CP_UTF8, 0, wideStr.c_str(), -1, &utf8Str[0], sizeNeeded, NULL, NULL);
    return utf8Str;
}

std::wstring UTF8ToWide(const std::string& utf8Str) {
    if (utf8Str.empty()) return std::wstring();
    
    int sizeNeeded = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, NULL, 0);
    if (sizeNeeded <= 0) return std::wstring();
    
    std::wstring wideStr(sizeNeeded - 1, 0);
    MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, &wideStr[0], sizeNeeded);
    return wideStr;
}

// 宽字符参数分割函数
void SplitParameterW(const wchar_t* parameter)
{
    if (parameter == nullptr)
        return;

    std::string parameterUTF8 = WideToUTF8(parameter);
    std::vector<std::string> plist;
    Stringsplit(parameterUTF8, ";", plist);
    for (int i = 0; i < plist.size(); i++)
    {
        std::vector<std::string> pResult;
        Stringsplit(plist[i], "=", pResult);
        if (pResult[0] == "globalscale")
        {
            gGlobalScale = atof(pResult[1].c_str());
        }
        if (pResult[0] == "enabletangent")
        {
            gOutPutTangent = true;
        }
        if (pResult[0] == "usefbxname")
        {
            gAnimUseFBXName = true;
        }
        if (pResult[0] == "meshlimitation")
        {
            gMeshLimitation = atoi(pResult[1].c_str());
        }
    }
}

void ParseFBXW(const wchar_t* fbxpath, const wchar_t* outdir, const wchar_t* parameter)
{
    FbxManager* lSdkManager = NULL;
    FbxScene* lScene = NULL;
    InitializeSdkObjects(lSdkManager, lScene);

    std::string fbxUTF8 = WideToUTF8(fbxpath);
    FbxString lFilePath(fbxUTF8.c_str());

    std::string::size_type iPos = (fbxUTF8.find_last_of('\\') + 1) == 0 ? fbxUTF8.find_last_of('/') + 1 : fbxUTF8.find_last_of('\\') + 1;
    std::string ImgName = fbxUTF8.substr(iPos, fbxUTF8.length() - iPos);
    gFBXFileName = ImgName.substr(0, ImgName.rfind("."));
    
    SplitParameterW(parameter);

    if (!lFilePath.IsEmpty())
    {
        bool lResult = LoadScene(lSdkManager, lScene, lFilePath.Buffer());
        if (lResult == false)
        {
            std::cout << "File type error: " << fbxUTF8 << std::endl;
        }
        else
        {
            std::string outdirUTF8;
            if (outdir != nullptr) {
                // 转换宽字符输出路径为UTF-8
                outdirUTF8 = WideToUTF8(outdir);
                // 统一路径分隔符为反斜杠
                std::replace(outdirUTF8.begin(), outdirUTF8.end(), '/', '\\');
                gOutPutDir = outdirUTF8;
                gOutPutDirSantinized = outdirUTF8;  // 保持原始路径，不进行清理
            }
            else {
                gOutPutDir.clear();
                gOutPutDirSantinized.clear();
            }
            
            // 使用宽字符创建输出目录
            if (outdir != nullptr) {
                EnsureDirectoryExists(std::wstring(outdir));
            }
            
            // gOutPutDirSantinized已经是UTF-8编码，直接使用
            std::vector<char> pathOutDir(gOutPutDirSantinized.begin(), gOutPutDirSantinized.end());
            pathOutDir.push_back('\0');
            ParseFBXScene(lSdkManager, *lScene, pathOutDir.data());
        }
    }
    else
    {
        std::cout << "File type error: " << fbxUTF8 << std::endl;
    }
}   
