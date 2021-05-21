#include "kslicer.h"
#include "template_rendering.h"
#include <iostream>

#ifdef WIN32
  #include <direct.h>     // for windows mkdir
#else
  #include <sys/stat.h>   // for linux mkdir
  #include <sys/types.h>
#endif


std::string GetFolderPath(const std::string& a_filePath);


void kslicer::GLSLCompiler::GenerateShaders(nlohmann::json& a_kernelsJson, const std::string& mainClassFileName, const std::vector<std::string>& includeToShadersFolders)
{
  std::string folderPath = GetFolderPath(mainClassFileName);
  std::string shaderPath = folderPath + "/" + this->ShaderFolder();
  #ifdef WIN32
  mkdir(shaderPath.c_str());
  #else
  mkdir(shaderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  #endif
  
  // generate header for all used functions in GLSL code
  //
  const std::string outFileNameH  = GetFolderPath(mainClassFileName) + "/z_generated.cl";
  kslicer::ApplyJsonToTemplate("templates_glsl/common_generated.h", shaderPath + "/common_generated.h", a_kernelsJson);  
  
  // now generate all glsl shaders
  //
  const std::string templatePath = "templates_glsl/generated.glsl";
  
  nlohmann::json copy, kernels;
  for (auto& el : a_kernelsJson.items())
  {
    //std::cout << el.key() << std::endl;
    if(std::string(el.key()) == "Kernels")
      kernels = a_kernelsJson[el.key()];
    else
      copy[el.key()] = a_kernelsJson[el.key()];
  }
  
    
  std::ofstream buildSH(shaderPath + "/build.sh");
  buildSH << "#!/bin/sh" << std::endl;
  for(auto& kernel : kernels.items())
  {
    nlohmann::json currKerneJson = copy;
    currKerneJson["Kernel"] = kernel.value();
    
    std::string kernelName  = std::string(kernel.value()["Name"]);
    std::string outFileName = kernelName + ".glsl";
    std::string outFilePath = shaderPath + "/" + outFileName;
    kslicer::ApplyJsonToTemplate(templatePath.c_str(), outFilePath, currKerneJson);
    buildSH << "glslangValidator -V " << outFileName.c_str() << " -o " << outFileName.c_str() << ".spv" << " -e " << kernelName.c_str() << " -DGLSL -I.. ";
    for(auto folder : includeToShadersFolders)
     buildSH << "-I" << folder.c_str() << " ";
    buildSH << std::endl;

    //glslangValidator -e myEntryPoint // is needed for auxilary kernels!
  }
    
  //nlohmann::json emptyJson;
  //std::string outFileServ = shaderPath + "/" + "serv_kernels.cpp";
  //kslicer::ApplyJsonToTemplate("templates/ser_circle.cxx", outFileServ, emptyJson);
  //buildSH << "../../circle -shader -c -emit-spirv " << outFileServ.c_str() << " -o " << outFileServ.c_str() << ".spv" << " -DUSE_CIRCLE_CC -I.. " << std::endl;
  
  buildSH.close();
}

std::string kslicer::GLSLCompiler::LocalIdExpr(uint32_t a_kernelDim, uint32_t a_wgSize[3]) const
{
  if(a_kernelDim == 1)
    return "gl_LocalInvocationID.x";
  else if(a_kernelDim == 2)
  {
    std::stringstream strOut;
    strOut << "gl_LocalInvocationID.x + " << a_wgSize[0] << "*gl_LocalInvocationID.y";
    return strOut.str();
  }
  else if(a_kernelDim == 3)
  {
    std::stringstream strOut;
    strOut << "gl_LocalInvocationID.x + " << a_wgSize[0] << "*gl_LocalInvocationID.y + " << a_wgSize[0]*a_wgSize[1] << "*gl_LocalInvocationID.z";
    return strOut.str();
  }
  else
  {
    std::cout << "  [GLSLCompiler::LocalIdExpr]: Error, bad kernelDim = " << a_kernelDim << std::endl;
    return "gl_LocalInvocationID.x";
  }
}

void kslicer::GLSLCompiler::GetThreadSizeNames(std::string a_strs[3]) const
{
  a_strs[0] = "kgenArgs.iNumElementsX";
  a_strs[1] = "kgenArgs.iNumElementsY";
  a_strs[2] = "kgenArgs.iNumElementsZ";
}


std::string kslicer::GLSLCompiler::ProcessBufferType(const std::string& a_typeName) const 
{ 
  std::string type = a_typeName;
  ReplaceFirst(type, "*", "");
  ReplaceFirst(type, "const", "");
  return type; 
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////  GLSLFunctionRewriter  ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

/**
\brief process local functions
*/
class GLSLFunctionRewriter : public kslicer::FunctionRewriter // 
{
public:
  
  GLSLFunctionRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, kslicer::MainClassInfo* a_codeInfo) : FunctionRewriter(R,a_compiler,a_codeInfo)
  { 
    m_vecReplacements["float2"] = "vec2";
    m_vecReplacements["float3"] = "vec3";
    m_vecReplacements["float4"] = "vec4";
    m_vecReplacements["int2"]   = "ivec2";
    m_vecReplacements["int3"]   = "ivec3";
    m_vecReplacements["int4"]   = "ivec4";
    m_vecReplacements["uint2"]  = "uvec2";
    m_vecReplacements["uint3"]  = "uvec3";
    m_vecReplacements["uint4"]  = "uvec4";
    m_vecReplacements["float4x4"] = "mat4";

    m_funReplacements["fmin"]  = "min";
    m_funReplacements["fmax"]  = "max";
    m_funReplacements["fminf"] = "min";
    m_funReplacements["fmaxf"] = "max";
    m_funReplacements["fsqrt"] = "sqrt";
    m_funReplacements["sqrtf"] = "sqrt";
  }

  ~GLSLFunctionRewriter()
  {
  }

protected:
  
  bool VisitFunctionDecl_Impl(clang::FunctionDecl* fDecl) override;
  bool VisitCallExpr_Impl(clang::CallExpr* f)             override;
  bool VisitVarDecl_Impl(clang::VarDecl* decl)            override;

  std::unordered_map<std::string, std::string> m_vecReplacements;
  std::unordered_map<std::string, std::string> m_funReplacements;

  bool        NeedsVectorTypeRewrite(const std::string& a_str);
  std::string RewriteVectorTypeStr(const std::string& a_str);
  std::string RewriteFuncDecl(clang::FunctionDecl* fDecl);

  std::string RecursiveRewrite(const clang::Stmt* expr) override;
};


std::string GLSLFunctionRewriter::RecursiveRewrite(const clang::Stmt* expr)
{
  if(expr == nullptr)
    return "";
  GLSLFunctionRewriter rvCopy = *this;
  rvCopy.TraverseStmt(const_cast<clang::Stmt*>(expr));
  return m_rewriter.getRewrittenText(expr->getSourceRange());
}

std::string GLSLFunctionRewriter::RewriteVectorTypeStr(const std::string& a_str)
{
  const bool isConst = (a_str.find("const ") != std::string::npos);
  std::string resStr;
  std::string typeStr = a_str;
  ReplaceFirst(typeStr, "LiteMath::", "");
  ReplaceFirst(typeStr, "glm::",      "");
  ReplaceFirst(typeStr, "struct ",    "");
  ReplaceFirst(typeStr, "const ",    "");
  
  auto p = m_vecReplacements.find(typeStr);
  if(p == m_vecReplacements.end())
    resStr = typeStr;
  else
    resStr = p->second;

  if(isConst)
    resStr = std::string("const ") + resStr;

  return resStr;
}

bool GLSLFunctionRewriter::NeedsVectorTypeRewrite(const std::string& a_str) // TODO: make this implementation more smart, bad implementation actually!
{
  if(a_str.find("glm::") != std::string::npos)
    return true;
  std::string name2 = std::string("LiteMath::") + a_str;
  bool need = false;
  for(auto p = m_vecReplacements.begin(); p != m_vecReplacements.end(); ++p)
  {
    if(name2.find(p->first) != std::string::npos)
    {
      need = true;
      break;
    }
  }
  return need;
}

std::string GLSLFunctionRewriter::RewriteFuncDecl(clang::FunctionDecl* fDecl)
{
  std::string retT   = RewriteVectorTypeStr(fDecl->getReturnType().getAsString()); 
  std::string fname  = fDecl->getNameInfo().getName().getAsString();
  std::string result = retT + " " + fname + "(";

  for(uint32_t i=0; i < fDecl->getNumParams(); i++)
  {
    const clang::ParmVarDecl* pParam  = fDecl->getParamDecl(i);
    const clang::QualType typeOfParam =	pParam->getType();
    result += RewriteVectorTypeStr(typeOfParam.getAsString()) + " " + pParam->getNameAsString();
    if(i!=fDecl->getNumParams()-1)
      result += ", ";
  }

  return result + ") ";
}

bool GLSLFunctionRewriter::VisitFunctionDecl_Impl(clang::FunctionDecl* fDecl) 
{ 
  if(clang::isa<clang::CXXMethodDecl>(fDecl)) // ignore methods here, for a while ... 
    return true;

  if(WasNotRewrittenYet(fDecl->getBody()))
  {
    const std::string funcDeclText = RewriteFuncDecl(fDecl);
    const std::string funcBodyText = RecursiveRewrite(fDecl->getBody());
 
    //auto debugMeIn = GetRangeSourceCode(call->getSourceRange(), m_compiler);     
    m_rewriter.ReplaceText(fDecl->getSourceRange(), funcDeclText + funcBodyText);
    MarkRewritten(fDecl->getBody());
  }

  return true; 
}

bool GLSLFunctionRewriter::VisitCallExpr_Impl(clang::CallExpr* call)
{
  if(clang::isa<clang::CXXMemberCallExpr>(call)) // process CXXMemberCallExpr else-where
    return true;

  clang::FunctionDecl* fDecl = call->getDirectCallee();
  if(fDecl == nullptr)
    return true;

  const std::string fname = fDecl->getNameInfo().getName().getAsString();
  std::string makeSmth = "";
  if(fname.substr(0, 5) == "make_")
    makeSmth = fname.substr(5);

  if(fname == "to_float3" && call->getNumArgs() == 1 && WasNotRewrittenYet(call) )
  {
    const std::string exprText = RecursiveRewrite(call->getArg(0));
    
    if(clang::isa<clang::CXXConstructExpr>(call->getArg(0)))                                 // TODO: add other similar node types process here
      m_rewriter.ReplaceText(call->getSourceRange(), exprText + ".xyz");                     // to_float3(f4Data) ==> f4Data.xyz
    else
      m_rewriter.ReplaceText(call->getSourceRange(), std::string("(") + exprText + ").xyz"); // to_float3(a+b)    ==> (a+b).xyz
      
    MarkRewritten(call);
  }
  else if( (fname == "fmin" || fname == "fmax" || fname == "fminf" || fname == "fmaxf") && call->getNumArgs() == 2 && WasNotRewrittenYet(call))
  {
    const std::string A = RecursiveRewrite(call->getArg(0));
    const std::string B = RecursiveRewrite(call->getArg(1));
    const std::string nameRewr = m_funReplacements[fname];
    m_rewriter.ReplaceText(call->getSourceRange(), nameRewr + "(" + A + "," + B + ")");
    MarkRewritten(call);
  }
  else if(makeSmth != "" && call->getNumArgs() !=0 && WasNotRewrittenYet(call) )
  {
    std::string rewrittenRes = m_vecReplacements[makeSmth] + "(";
    for(int i=0;i<call->getNumArgs(); i++)
    {
      rewrittenRes += RecursiveRewrite(call->getArg(i));
      if(i!=call->getNumArgs()-1)
        rewrittenRes += ", ";
    }
    rewrittenRes += ")";
    m_rewriter.ReplaceText(call->getSourceRange(), rewrittenRes);
    MarkRewritten(call);
  }
  else if(fname == "mul4x4x4" && call->getNumArgs() == 2 && WasNotRewrittenYet(call))
  {
    const std::string A = RecursiveRewrite(call->getArg(0));
    const std::string B = RecursiveRewrite(call->getArg(1));
    m_rewriter.ReplaceText(call->getSourceRange(), "(" + A + "*" + B + ")");
    MarkRewritten(call);
  }

  return true; 
}

bool GLSLFunctionRewriter::VisitVarDecl_Impl(clang::VarDecl* decl) 
{
  if(clang::isa<clang::ParmVarDecl>(decl)) // process else-where (VisitFunctionDecl_Impl)
    return true;

  const auto qt     = decl->getType();
  const auto pValue = decl->getAnyInitializer();
      
  //const std::string debugText = kslicer::GetRangeSourceCode(decl->getSourceRange(), m_compiler); 
  const std::string varType   = qt.getAsString();
  if(NeedsVectorTypeRewrite(varType) && WasNotRewrittenYet(pValue))
  {
    const std::string varType2 = RewriteVectorTypeStr(varType);
    const std::string varName  = decl->getNameAsString();
    const std::string varValue = RecursiveRewrite(pValue);
    
    if(varValue == "")
      m_rewriter.ReplaceText(decl->getSourceRange(), varType2 + " " + varName);
    else
      m_rewriter.ReplaceText(decl->getSourceRange(), varType2 + " " + varName + " = " + varValue);
    MarkRewritten(pValue);
  }
  return true;
}


std::shared_ptr<kslicer::FunctionRewriter> kslicer::GLSLCompiler::MakeFuncRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, kslicer::MainClassInfo* a_codeInfo)
{
  return std::make_shared<GLSLFunctionRewriter>(R, a_compiler, a_codeInfo);
}

std::shared_ptr<kslicer::KernelRewriter> kslicer::GLSLCompiler::MakeKernRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, MainClassInfo* a_codeInfo, 
                                                                                 kslicer::KernelInfo& a_kernel, const std::string& fakeOffs, bool a_infoPass)
{
  return std::make_shared<kslicer::KernelRewriter>(R, a_compiler, a_codeInfo, a_kernel, fakeOffs, a_infoPass);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


