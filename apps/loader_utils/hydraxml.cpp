#include "hydraxml.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <codecvt>

#if defined(__ANDROID__)
#define LOGE(...) \
  ((void)__android_log_print(ANDROID_LOG_ERROR, "HydraXML", __VA_ARGS__))
#endif

namespace hydra_xml
{
  std::wstring s2ws(const std::string& str)
  {
    using convert_typeX = std::codecvt_utf8<wchar_t>;
    std::wstring_convert<convert_typeX, wchar_t> converterX;

    return converterX.from_bytes(str);
  }

  std::string ws2s(const std::wstring& wstr)
  {
    using convert_typeX = std::codecvt_utf8<wchar_t>;
    std::wstring_convert<convert_typeX, wchar_t> converterX;

    return converterX.to_bytes(wstr);
  }

  void HydraScene::LogError(const std::string &msg)
  {
    std::cout << "HydraScene ERROR: " << msg << std::endl;
  }

#if defined(__ANDROID__)
  int HydraScene::LoadState(AAssetManager* mgr, const std::string &path)
  {
    AAsset* asset = AAssetManager_open(mgr, path.c_str(), AASSET_MODE_STREAMING);
    if (!asset)
    {
      LOGE("Could not load scene from \"%s\"!", path.c_str());
    }
    assert(asset);

    size_t asset_size = AAsset_getLength(asset);

    assert(asset_size > 0);

    void* data = malloc(asset_size);
    AAsset_read(asset, data, asset_size);
    AAsset_close(asset);

    pugi::xml_document xmlDoc;

    auto loaded = xmlDoc.load_buffer(data, asset_size);

    if(!loaded)
    {
      std::string str(loaded.description());
      LOGE("pugixml error loading scene: %s", str.c_str());

      return -1;
    }

    auto pos = path.find_last_of(L'/');
    m_libraryRootDir = path.substr(0, pos);

    auto texturesLib  = xmlDoc.child(L"textures_lib");
    auto materialsLib = xmlDoc.child(L"materials_lib");
    auto geometryLib  = xmlDoc.child(L"geometry_lib");
    auto lightsLib    = xmlDoc.child(L"lights_lib");

    auto cameraLib    = xmlDoc.child(L"cam_lib");
    auto settingsNode = xmlDoc.child(L"render_lib");
    auto sceneNode    = xmlDoc.child(L"scenes");

    if (texturesLib == nullptr || materialsLib == nullptr || lightsLib == nullptr || cameraLib == nullptr ||
        geometryLib == nullptr || settingsNode == nullptr || sceneNode == nullptr)
    {
      std::string errMsg = "Loaded state (" +  path + ") doesn't have one of (textures_lib, materials_lib, lights_lib, cam_lib, geometry_lib, render_lib, scenes";
      LogError(errMsg);
      return -1;
    }

    parseInstancedMeshes(sceneNode, geometryLib);

    return 0;
  }
#else
  int HydraScene::LoadState(const std::string &path)
  {
    pugi::xml_document xmlDoc;

    auto loaded = xmlDoc.load_file(path.c_str());

    if(!loaded)
    {
      std::string  str(loaded.description());
      std::wstring errorMsg(str.begin(), str.end());

      LogError("Error loading scene from: " + path);
      LogError(ws2s(errorMsg));

      return -1;
    }

    auto pos = path.find_last_of(L'/');
    m_libraryRootDir = path.substr(0, pos);

    auto texturesLib  = xmlDoc.child(L"textures_lib");
    auto materialsLib = xmlDoc.child(L"materials_lib");
    auto geometryLib  = xmlDoc.child(L"geometry_lib");
    auto lightsLib    = xmlDoc.child(L"lights_lib");

    auto cameraLib    = xmlDoc.child(L"cam_lib");
    auto settingsNode = xmlDoc.child(L"render_lib");
    auto sceneNode    = xmlDoc.child(L"scenes");

    if (texturesLib == nullptr || materialsLib == nullptr || lightsLib == nullptr || cameraLib == nullptr ||
        geometryLib == nullptr || settingsNode == nullptr || sceneNode == nullptr)
    {
      std::string errMsg = "Loaded state (" +  path + ") doesn't have one of (textures_lib, materials_lib, lights_lib, cam_lib, geometry_lib, render_lib, scenes";
      LogError(errMsg);
      return -1;
    }

    parseInstancedMeshes(sceneNode, geometryLib);

    return 0;
  }
#endif

  void HydraScene::parseInstancedMeshes(pugi::xml_node a_scenelib, pugi::xml_node a_geomlib)
  {
    auto scene = a_scenelib.first_child();
    for (pugi::xml_node inst = scene.first_child(); inst != nullptr; inst = inst.next_sibling())
    {
      if (std::wstring(inst.name()) == L"instance_light")
        break;

      auto mesh_id = inst.attribute(L"mesh_id").as_string();
      auto matrix = std::wstring(inst.attribute(L"matrix").as_string());

      auto meshNode = a_geomlib.find_child_by_attribute(L"id", mesh_id);

      if(meshNode != nullptr)
      {
        auto meshLoc = ws2s(std::wstring(meshNode.attribute(L"loc").as_string()));
        meshLoc = m_libraryRootDir + "/" + meshLoc;

#if not defined(__ANDROID__)
        std::ifstream checkMesh(meshLoc);
        if(!checkMesh.good())
        {
          LogError("Mesh not found at: " + meshLoc + ". Loader will skip it.");
          continue;
        }
        else
        {
          checkMesh.close();
        }
#endif

        if(unique_meshes.find(meshLoc) == unique_meshes.end())
        {
          unique_meshes.emplace(meshLoc);
          m_meshloc.push_back(meshLoc);
        }


        if(m_instancesPerMeshLoc.find(meshLoc) != m_instancesPerMeshLoc.end())
        {
          m_instancesPerMeshLoc[meshLoc].push_back(float4x4FromString(matrix));
        }
        else
        {
          std::vector<LiteMath::float4x4> tmp = { float4x4FromString(matrix) };
          m_instancesPerMeshLoc[meshLoc] = tmp;
        }
      }
    }


  }

  LiteMath::float4x4 float4x4FromString(const std::wstring &matrix_str)
  {
    LiteMath::float4x4 result;
    std::wstringstream inputStream(matrix_str);
    
    float data[16];
    for(int i=0;i<16;i++)
      inputStream >> data[i];
    
    result.set_row(0, LiteMath::float4(data[0],data[1], data[2], data[3]));
    result.set_row(1, LiteMath::float4(data[4],data[5], data[6], data[7]));
    result.set_row(2, LiteMath::float4(data[8],data[9], data[10], data[11]));
    result.set_row(3, LiteMath::float4(data[12],data[13], data[14], data[15])); 

    return result;
  }

//  LiteMath::float4x4 float4x4FromString(const std::wstring &matrix_str)
//  {
//    LiteMath::float4x4 result;
//    std::wstringstream inputStream(matrix_str);
//
//    inputStream >> result.col(0).x  >> result.col(1).x  >> result.col(2).x  >> result.col(3).x
//                >> result.col(0).y  >> result.col(1).y  >> result.col(2).y  >> result.col(3).y
//                >> result.col(0).z  >> result.col(1).z  >> result.col(2).z  >> result.col(3).z
//                >> result.col(0).w  >> result.col(1).w  >> result.col(2).w  >> result.col(3).w;
//
//    return result;
//  }
}

