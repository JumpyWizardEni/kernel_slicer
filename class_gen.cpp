#include "kslicer.h"
#include "class_gen.h"
#include "ast_matchers.h"

#include <sstream>
#include <algorithm>

void kslicer::MainClassInfo::AddTempBufferToKernel(const std::string buffName, const std::string a_elemTypeName, KernelInfo& a_kernel)
{
  // (1) append vector to this->allDataMembers
  //
  auto pFoundMember = allDataMembers.find(buffName);
  if(pFoundMember == allDataMembers.end())
  {
    DataMemberInfo vecMemberTmp;
    vecMemberTmp.name          = buffName;
    vecMemberTmp.type          = std::string("std::vector<") + a_elemTypeName + ">";
    vecMemberTmp.sizeInBytes   = 4; // not used by containers
    vecMemberTmp.isContainer   = true;
    vecMemberTmp.usedInKernel  = true;
    vecMemberTmp.containerType = "std::vector";
    vecMemberTmp.containerDataType = a_elemTypeName;
    vecMemberTmp.usage  = kslicer::DATA_USAGE::USAGE_SLICER_REDUCTION; // we dont have to generate code for update of vector or vector size for such vectors
    pFoundMember = allDataMembers.insert({buffName, vecMemberTmp}).first;
  }

  // (2) append vector to a_kernel.usedVectors 
  //
  a_kernel.usedVectors.insert(buffName);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ReplaceFirst(std::string& str, const std::string& from, const std::string& to) 
{
  size_t start_pos = str.find(from);
  if(start_pos == std::string::npos)
    return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

std::string kslicer::MainClassInfo::RemoveKernelPrefix(const std::string& a_funcName) const
{
  std::string name = a_funcName;
  if(ReplaceFirst(name, "kernel_", ""))
    return name;
  else
    return a_funcName;
}

bool kslicer::MainClassInfo::IsKernel(const std::string& a_funcName) const
{
  auto pos = a_funcName.find("kernel_");
  return (pos != std::string::npos);
}

bool kslicer::MainClassInfo::IsIndirect(const KernelInfo& a_kernel) const
{
  bool isIndirect = false;
  for(auto& arg : a_kernel.loopIters)
  {
    bool foundSize = (arg.sizeExpr.find(".size()") != std::string::npos);
    bool foundCap  = (arg.sizeExpr.find(".capacity()") != std::string::npos);
    bool isMember  = (allDataMembers.find(arg.sizeExpr) != allDataMembers.end());

    if(foundSize)
    {
      auto pos        = arg.sizeExpr.find(".size()");
      auto memberName = arg.sizeExpr.substr(0, pos);
      if(allDataMembers.find(memberName) == allDataMembers.end())
      {
        std::cout << "[ERROR]: Use non-member .size() expression '" << arg.sizeExpr.c_str() << "' as loop boundary for kernel " <<  a_kernel.name << std::endl;
        std::cout << "[ERROR]: Only class members and member vectors.size() are allowed." << std::endl;
      }
    }

    if(foundCap)
    {
      auto pos        = arg.sizeExpr.find(".capacity()");
      auto memberName = arg.sizeExpr.substr(0, pos);
      if(allDataMembers.find(memberName) == allDataMembers.end())
      {
        std::cout << "[ERROR]: Use non-member .capacity() expression '" << arg.sizeExpr.c_str() << "' as loop boundary for kernel " <<  a_kernel.name << std::endl;
        std::cout << "[ERROR]: Only class members and member vectors.capacity() are allowed." << std::endl;
      }
    }

    isIndirect = isIndirect || foundSize || foundCap || isMember;
  }
  return isIndirect;
} 

std::string kslicer::MainClassInfo::GetCFSourceCodeCmd(MainFuncInfo& a_mainFunc, clang::CompilerInstance& compiler)
{
  const std::string&   a_mainClassName = this->mainClassName;
  const CXXMethodDecl* a_node          = a_mainFunc.Node;
  const std::string&   a_mainFuncName  = a_mainFunc.Name;
  std::string&         a_outFuncDecl   = a_mainFunc.GeneratedDecl;

  const auto  inOutParamList = kslicer::ListPointerParamsOfMainFunc(a_node);

  Rewriter rewrite2;
  rewrite2.setSourceMgr(compiler.getSourceManager(), compiler.getLangOpts());

  a_mainFunc.startDSNumber = allDescriptorSetsInfo.size();

  kslicer::MainFunctionRewriter rv(rewrite2, compiler, a_mainFunc, inOutParamList, this); // ==> write this->allDescriptorSetsInfo during 'TraverseDecl'
  rv.TraverseDecl(const_cast<clang::CXXMethodDecl*>(a_node));

  clang::SourceLocation b(a_node->getBeginLoc()), _e(a_node->getEndLoc());
  clang::SourceLocation e(clang::Lexer::getLocForEndOfToken(_e, 0, compiler.getSourceManager(), compiler.getLangOpts()));
  
  std::string sourceCode = rewrite2.getRewrittenText(clang::SourceRange(b,e));
  
  // (1) TestClass::MainFuncCmd --> TestClass_Generated::MainFuncCmd and add input command Buffer as first argument
  // 
  const std::string replaceFrom = a_mainClassName + "::" + rv.mainFuncCmdName;
  const std::string replaceTo   = a_mainClassName + "_Generated" + "::" + rv.mainFuncCmdName;

  assert(ReplaceFirst(sourceCode, replaceFrom, replaceTo));

  if(a_mainFunc.Node->getNumParams() != 0)
  {
    assert(ReplaceFirst(sourceCode, "(", "(VkCommandBuffer a_commandBuffer, "));
  }
  else
  {
    assert(ReplaceFirst(sourceCode, "(", "(VkCommandBuffer a_commandBuffer"));
  }

  // (3) set m_currCmdBuffer with input command bufer and add other prolog to MainFunCmd
  //
  std::stringstream strOut;
  strOut << "{" << std::endl;
  strOut << "  m_currCmdBuffer = a_commandBuffer;" << std::endl;

  if(this->NeedThreadFlags())
  {
    strOut << "  const uint32_t outOfForFlags  = KGEN_FLAG_RETURN;" << std::endl;
    strOut << "  const uint32_t inForFlags     = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK;" << std::endl;
    if(a_mainFunc.needToAddThreadFlags)
    {
      const std::string buffName = a_mainFunc.Name + "_local.threadFlagsBuffer";

      strOut << "  const uint32_t outOfForFlagsN = KGEN_FLAG_RETURN | KGEN_FLAG_SET_EXIT_NEGATIVE;" << std::endl;
      strOut << "  const uint32_t inForFlagsN    = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK | KGEN_FLAG_SET_EXIT_NEGATIVE;" << std::endl;
      strOut << "  const uint32_t outOfForFlagsD = KGEN_FLAG_RETURN | KGEN_FLAG_DONT_SET_EXIT;" << std::endl;
      strOut << "  const uint32_t inForFlagsD    = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK | KGEN_FLAG_DONT_SET_EXIT;" << std::endl;
      strOut << "  vkCmdFillBuffer(a_commandBuffer, " << buffName.c_str() << ", 0, VK_WHOLE_SIZE, 0); // zero thread flags, mark all threads to be active" << std::endl;
      strOut << "  VkBufferMemoryBarrier fillBarrier = BarrierForClearFlags(" << buffName.c_str() << "); " << std::endl;
      strOut << "  vkCmdPipelineBarrier(a_commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 0, nullptr, 1, &fillBarrier, 0, nullptr); " << std::endl;
    }
    strOut << std::endl; 
  }

  size_t bracePos = sourceCode.find("{");
  sourceCode = (sourceCode.substr(0, bracePos) + strOut.str() + sourceCode.substr(bracePos+2));

  return sourceCode;
}

std::string kslicer::MainClassInfo::GetCFDeclFromSource(const std::string& sourceCode)
{
  std::string mainFuncDecl = sourceCode.substr(0, sourceCode.find(")")+1) + ";";
  assert(ReplaceFirst(mainFuncDecl, mainClassName + "_Generated" + "::", ""));
  return "virtual " + mainFuncDecl;
}

std::vector<kslicer::InOutVarInfo> kslicer::ListPointerParamsOfMainFunc(const CXXMethodDecl* a_node)
{
  std::vector<InOutVarInfo> params;
  for(int i=0;i<a_node->getNumParams();i++)
  {
    const ParmVarDecl* currParam = a_node->getParamDecl(i);
    
    const clang::QualType qt = currParam->getType();
    const auto typePtr = qt.getTypePtr(); 
    assert(typePtr != nullptr);
    
    if(!typePtr->isPointerType())
      continue;
    
    InOutVarInfo var;
    var.name = currParam->getNameAsString();
    params.push_back(var);
  }

  return params;
}

std::string kslicer::MainClassInfo::VisitAndRewrite_KF(KernelInfo& a_funcInfo, const clang::CompilerInstance& compiler, std::string& a_outLoopInitCode, std::string& a_outLoopFinishCode)
{
  const CXXMethodDecl* a_node = a_funcInfo.astNode;
  //a_node->dump();
  
  std::string names[3];
  pShaderCC->GetThreadSizeNames(names);
  std::string fakeOffsetExpr = kslicer::GetFakeOffsetExpression(a_funcInfo, GetKernelTIDArgs(a_funcInfo), names);

  Rewriter rewrite2;
  rewrite2.setSourceMgr(compiler.getSourceManager(), compiler.getLangOpts());
  
  auto pVisitor = pShaderCC->MakeKernRewriter(rewrite2, compiler, this, a_funcInfo, fakeOffsetExpr, false);
  pVisitor->TraverseDecl(const_cast<clang::CXXMethodDecl*>(a_node));
  
  clang::SourceLocation b(a_node->getBeginLoc()), _e(a_node->getEndLoc());
  clang::SourceLocation e(clang::Lexer::getLocForEndOfToken(_e, 0, compiler.getSourceManager(), compiler.getLangOpts()));
  
  return rewrite2.getRewrittenText(clang::SourceRange(b,e));
}

std::vector<kslicer::MainClassInfo::ArgTypeAndNamePair> kslicer::MainClassInfo::GetKernelTIDArgs(const KernelInfo& a_kernel) const
{
  std::vector<kslicer::MainClassInfo::ArgTypeAndNamePair> args;
  for (const auto& arg : a_kernel.args) 
  {    
    if(arg.isThreadID)
    { 
      ArgTypeAndNamePair arg2;
      arg2.argName  = arg.name;
      arg2.sizeName = arg.name;
      arg2.typeName = RemoveTypeNamespaces(arg.type);
      arg2.id       = 0;
      args.push_back(arg2);
    }
  }

  std::sort(args.begin(), args.end(), [](const auto& a, const auto & b) { return a.argName < b.argName; });

  return args;
}

std::vector<kslicer::MainClassInfo::ArgTypeAndNamePair> kslicer::MainClassInfo::GetKernelCommonArgs(const KernelInfo& a_kernel) const
{
  std::vector<kslicer::MainClassInfo::ArgTypeAndNamePair> args;
  for (const auto& arg : a_kernel.args) 
  { 
    if(!arg.isThreadID && !arg.isLoopSize && !arg.IsUser())
    { 
      ArgTypeAndNamePair arg2;
      arg2.argName  = arg.name;
      arg2.typeName = RemoveTypeNamespaces(arg.type);
      arg2.isUBO    = (arg.type.find(std::string("class ")  + mainClassName) != std::string::npos || 
                       arg.type.find(std::string("struct ") + mainClassName) != std::string::npos);
      arg2.isThreadFlags = arg.isThreadFlags;
      args.push_back(arg2);
    }
  }

  return args;
}

void kslicer::ObtainKernelsDecl(std::unordered_map<std::string, KernelInfo>& a_kernelsData, const clang::CompilerInstance& compiler, const std::string& a_mainClassName, const MainClassInfo& a_codeInfo)
{
  for (auto& k : a_kernelsData)  
  {
    assert(k.second.astNode != nullptr);
    auto sourceRange = k.second.astNode->getSourceRange();
    
    std::string funcName         = k.second.astNode->getNameInfo().getAsString();
    std::string kernelSourceCode = GetRangeSourceCode(sourceRange, compiler);
    
    auto posBeg      = kernelSourceCode.find(funcName);
    auto posEnd      = posBeg + funcName.size();
    auto posEndBrace = kernelSourceCode.find(")");

    std::string kernelCmdDecl = kernelSourceCode.substr(posBeg, posEndBrace+1-posBeg);   
    kernelCmdDecl = a_codeInfo.RemoveKernelPrefix(kernelCmdDecl);

    assert(ReplaceFirst(kernelCmdDecl,"(", "Cmd("));
    k.second.DeclCmd = kernelCmdDecl;
    k.second.RetType = kernelSourceCode.substr(0, posBeg);
    ReplaceFirst(k.second.RetType, a_mainClassName + "::", "");
    if(k.second.isBoolTyped)
      ReplaceFirst(k.second.RetType,"bool ", "void ");
  }
}

void kslicer::FunctionRewriter::MarkRewritten(const clang::Stmt* expr) { kslicer::MarkRewrittenRecursive(expr, m_rewrittenNodes); }
//void kslicer::FunctionRewriter::MarkRewritten(const clang::Decl* decl) { kslicer::MarkRewrittenRecursive(decl, m_rewrittenNodes); }

bool kslicer::FunctionRewriter::WasNotRewrittenYet(const clang::Stmt* expr)
{
  if(expr == nullptr)
    return true;
  if(clang::isa<clang::NullStmt>(expr))
    return true;
  const auto exprHash  = kslicer::GetHashOfSourceRange(expr->getSourceRange());
  return (m_rewrittenNodes.find(exprHash) == m_rewrittenNodes.end());
}

//bool kslicer::FunctionRewriter::WasNotRewrittenYet(const clang::Decl* decl)
//{
//  if(decl == nullptr)
//    return true;
//  const auto exprHash  = kslicer::GetHashOfSourceRange(decl->getSourceRange());
//  return (m_rewrittenNodes.find(exprHash) == m_rewrittenNodes.end());
//}