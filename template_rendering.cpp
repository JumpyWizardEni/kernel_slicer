#include "template_rendering.h"
#include <inja.hpp>

// Just for convenience
using namespace inja;
using json = nlohmann::json;

std::string GetFolderPath(const std::string& a_filePath)
{
  size_t lastindex = a_filePath.find_last_of("/"); 
  assert(lastindex != std::string::npos);   
  return a_filePath.substr(0, lastindex); 
}

void MakeAbsolutePathRelativeTo(std::string& a_filePath, const std::string& a_folderPath)
{
  if(a_filePath.find(a_folderPath) != std::string::npos)  // cut off folder path
    a_filePath = a_filePath.substr(a_folderPath.size() + 1);
}

void kslicer::PrintVulkanBasicsFile(const std::string& a_declTemplateFilePath, const MainClassInfo& a_classInfo)
{
  json data;
  inja::Environment env;
  inja::Template temp = env.parse_template(a_declTemplateFilePath.c_str());
  std::string result  = env.render(temp, data);
  
  std::string folderPath = GetFolderPath(a_classInfo.mainClassFileName);

  std::ofstream fout(folderPath + "/vulkan_basics.h");
  fout << result.c_str() << std::endl;
  fout.close();
}

std::string kslicer::PrintGeneratedClassDecl(const std::string& a_declTemplateFilePath, const MainClassInfo& a_classInfo)
{
  std::string rawname;
  {
    size_t lastindex = a_classInfo.mainClassFileName.find_last_of("."); 
    assert(lastindex != std::string::npos);
    rawname = a_classInfo.mainClassFileName.substr(0, lastindex); 
  }

  std::string folderPath = GetFolderPath(a_classInfo.mainClassFileName);
  std::string mainInclude = a_classInfo.mainClassFileInclude;
  
  MakeAbsolutePathRelativeTo(mainInclude, folderPath);

  std::stringstream strOut;
  strOut << "#include \"" << mainInclude.c_str() << "\"" << std::endl;

  json data;
  data["Includes"]      = strOut.str();
  data["MainClassName"] = a_classInfo.mainClassName;
  data["MainFuncName"]  = a_classInfo.mainFuncName;

  data["PlainMembersUpdateFunctions"]  = "";
  data["VectorMembersUpdateFunctions"] = "";
  data["KernelsDecl"]                  = "";
  
  inja::Environment env;
  inja::Template temp = env.parse_template(a_declTemplateFilePath.c_str());
  std::string result  = env.render(temp, data);
  
  std::string includeFileName = rawname + "_generated.h";
  std::ofstream fout(includeFileName);
  fout << result.c_str() << std::endl;
  fout.close();

  return includeFileName;
} 

void kslicer::PrintGeneratedClassImpl(const std::string& a_declTemplateFilePath, 
                                      const std::string& a_includeName, 
                                      const MainClassInfo& a_classInfo,
                                      const std::string& a_mainFuncCodeGen)
{
  std::string folderPath  = GetFolderPath(a_includeName);
  std::string mainInclude = a_includeName;
  MakeAbsolutePathRelativeTo(mainInclude, folderPath);

  std::string rawname;
  {
    size_t lastindex = a_classInfo.mainClassFileName.find_last_of("."); 
    assert(lastindex != std::string::npos);
    rawname = a_classInfo.mainClassFileName.substr(0, lastindex); 
  }

  json data;
  data["Includes"]         = "";
  data["IncludeClassDecl"] = mainInclude;
  data["MainFuncCmd"]      = a_mainFuncCodeGen;
  
  inja::Environment env;
  inja::Template temp = env.parse_template(a_declTemplateFilePath.c_str());
  std::string result  = env.render(temp, data);
  
  std::string cppFileName = rawname + "_generated.cpp";
  std::ofstream fout(cppFileName);
  fout << result.c_str() << std::endl;
  fout.close();
}
