#include "kslicer.h"
#include "class_gen.h"
#include "ast_matchers.h"
#include "extractor.h"

#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/DeclTemplate.h"
#include "clang/Parse/ParseAST.h"

#include <sstream>
#include <algorithm>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//// tid, fakeOffset(tidX,tidY,kgen_iNumElementsX) or fakeOffset2(tidX,tidY,tidX,kgen_iNumElementsX, kgen_iNumElementsY)
//
std::string kslicer::GetFakeOffsetExpression(const kslicer::KernelInfo& a_funcInfo, const std::vector<kslicer::MainClassInfo::ArgTypeAndNamePair>& threadIds) 
{
  if(threadIds.size() == 1)
    return threadIds[0].argName;
  else if(threadIds.size() == 2)
    return std::string("fakeOffset(") + threadIds[0].argName + "," + threadIds[1].argName + ",kgen_iNumElementsX)";
  else if(threadIds.size() == 3)
    return std::string("fakeOffset(") + threadIds[0].argName + "," + threadIds[1].argName + "," + threadIds[2].argName + ",kgen_iNumElementsX,kgen_iNumElementsY)";
  else
    return "tid";
}

std::vector<std::string> kslicer::GetAllPredefinedThreadIdNamesRTV()
{
  return {"tid", "tidX", "tidY", "tidZ"};
}

uint32_t kslicer::RTV_Pattern::GetKernelDim(const kslicer::KernelInfo& a_kernel) const
{
  return uint32_t(GetKernelTIDArgs(a_kernel).size());
} 

std::string kslicer::RTV_Pattern::VisitAndRewrite_CF(MainFuncInfo& a_mainFunc, clang::CompilerInstance& compiler)
{
  const std::string&   a_mainClassName = this->mainClassName;
  const CXXMethodDecl* a_node          = a_mainFunc.Node;
  const std::string&   a_mainFuncName  = a_mainFunc.Name;
  std::string&         a_outFuncDecl   = a_mainFunc.GeneratedDecl;

  std::string sourceCode = GetCFSourceCodeCmd(a_mainFunc, compiler); // ==> write this->allDescriptorSetsInfo
  a_outFuncDecl          = GetCFDeclFromSource(sourceCode); 
  a_mainFunc.endDSNumber = allDescriptorSetsInfo.size();

  return sourceCode;
}


void kslicer::RTV_Pattern::AddSpecVars_CF(std::vector<MainFuncInfo>& a_mainFuncList, std::unordered_map<std::string, KernelInfo>& a_kernelList)
{
  // (1) first scan all main functions, if no one needed just exit
  //
  std::unordered_set<std::string> kernelsToAddFlags;
  std::unordered_set<std::string> kernelsAddedFlags;

  for(auto& mainFunc : a_mainFuncList)
  {
    if(mainFunc.ExitExprIfCond.size() != 0)
    {
      for(const auto& kernelName : mainFunc.UsedKernels)
        kernelsToAddFlags.insert(kernelName);
      mainFunc.needToAddThreadFlags = true;
    }
  }

  if(kernelsToAddFlags.empty())
    return;

  // (2) add flags to kernels which we didn't process yet
  //
  while(!kernelsToAddFlags.empty())
  {
    // (2.1) process kernels
    //
    for(auto kernName : kernelsToAddFlags)
      kernelsAddedFlags.insert(kernName);

    // (2.2) find all main functions which used processed kernels 
    //
    for(auto& mainFunc : a_mainFuncList)
    {
      for(const auto& kernelName : kernelsAddedFlags)
      {
        if(mainFunc.UsedKernels.find(kernelName) != mainFunc.UsedKernels.end())
        {
          mainFunc.needToAddThreadFlags = true;
          break;
        }
      }
    }

    // (2.3) process main functions again, check we processed all kernels for each main function which 'needToAddThreadFlags' 
    //
    kernelsToAddFlags.clear();
    for(const auto& mainFunc : a_mainFuncList)
    {
      if(!mainFunc.needToAddThreadFlags)
        continue;

      for(const auto& kName : mainFunc.UsedKernels)
      {
        const auto p = kernelsAddedFlags.find(kName);
        if(p == kernelsAddedFlags.end())
          kernelsToAddFlags.insert(kName);
      }
    }
  }

  // (3) finally add actual variables to MainFunc, arguments to kernels and reference to kernel call 
  //
  DataLocalVarInfo   tFlagsLocalVar;
  KernelInfo::Arg    tFlagsArg;

  tFlagsLocalVar.name = "threadFlags";
  tFlagsLocalVar.type = "uint";
  tFlagsLocalVar.sizeInBytes = sizeof(uint32_t);
  
  tFlagsArg.name           = "kgen_threadFlags";
  tFlagsArg.needFakeOffset = true;
  tFlagsArg.size           = 1; // array size 
  tFlagsArg.type           = "uint*";
  
  // Add threadFlags to kernel arguments
  //
  for(auto kName : kernelsAddedFlags)
  {
    auto pKernel = a_kernelList.find(kName);
    if(pKernel == a_kernelList.end())
      continue;

    size_t foundId = size_t(-1);
    for(size_t i=0;i<pKernel->second.args.size();i++)
    {
      if(pKernel->second.args[i].name == tFlagsArg.name)
      {
        foundId = i;
        break;
      }
    }

    if(foundId == size_t(-1))
    {
      pKernel->second.args.push_back(tFlagsArg);
      pKernel->second.checkThreadFlags = true;
    }
  }  
 
  // add thread flags to MainFuncions and all kernel calls for each MainFunc
  //
  for(auto& mainFunc : a_mainFuncList)
  {
    auto p = mainFunc.Locals.find(tFlagsLocalVar.name);
    if(p != mainFunc.Locals.end() || !mainFunc.needToAddThreadFlags)
      continue;

    mainFunc.Locals[tFlagsLocalVar.name] = tFlagsLocalVar;
  }

}

void kslicer::RTV_Pattern::PlugSpecVarsInCalls_CF(const std::vector<MainFuncInfo>&                   a_mainFuncList, 
                                                  const std::unordered_map<std::string, KernelInfo>& a_kernelList,
                                                  std::vector<KernelCallInfo>&                       a_kernelCalls)
{
  // list kernels and main functions
  //
  std::unordered_map<std::string, size_t> mainFuncIdByName;
  for(size_t i=0;i<a_mainFuncList.size();i++)
    mainFuncIdByName[a_mainFuncList[i].Name] = i;

  ArgReferenceOnCall tFlagsArgRef;
  tFlagsArgRef.argType     = KERN_CALL_ARG_TYPE::ARG_REFERENCE_LOCAL;
  tFlagsArgRef.varName     = "threadFlags";
  tFlagsArgRef.umpersanned = true;

  // add thread flags to MainFuncions and all kernel calls for each MainFunc
  //
  for(auto& call : a_kernelCalls)
  {
    const auto& mainFunc = a_mainFuncList[mainFuncIdByName[call.callerName]];
    if(!mainFunc.needToAddThreadFlags)
      continue;

    auto p2 = a_kernelList.find(call.originKernelName);
    if(p2 != a_kernelList.end())
      call.descriptorSetsInfo.push_back(tFlagsArgRef);
  }
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

kslicer::RTV_Pattern::MList kslicer::RTV_Pattern::ListMatchers_CF(const std::string& mainFuncName)
{
  std::vector<clang::ast_matchers::StatementMatcher> list;
  list.push_back(kslicer::MakeMatch_LocalVarOfMethod(mainFuncName));
  list.push_back(kslicer::MakeMatch_MethodCallFromMethod(mainFuncName));
  list.push_back(kslicer::MakeMatch_SingleForLoopInsideFunction(mainFuncName));
  list.push_back(kslicer::MakeMatch_IfInsideForLoopInsideFunction(mainFuncName));
  list.push_back(kslicer::MakeMatch_FunctionCallInsideForLoopInsideFunction(mainFuncName));
  list.push_back(kslicer::MakeMatch_IfReturnFromFunction(mainFuncName));
  return list;
}

kslicer::RTV_Pattern::MHandlerCFPtr kslicer::RTV_Pattern::MatcherHandler_CF(kslicer::MainFuncInfo& a_mainFuncRef, const clang::CompilerInstance& a_compiler)
{
  return std::move(std::make_unique<kslicer::MainFuncAnalyzerRT>(std::cout, *this, a_compiler.getASTContext(), a_mainFuncRef));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

kslicer::RTV_Pattern::MList kslicer::RTV_Pattern::ListMatchers_KF(const std::string& a_kernelName)
{
  std::vector<clang::ast_matchers::StatementMatcher> list;
  list.push_back(kslicer::MakeMatch_MemberVarOfMethod(a_kernelName));
  list.push_back(kslicer::MakeMatch_FunctionCallFromFunction(a_kernelName));
  return list;
}

kslicer::RTV_Pattern::MHandlerKFPtr kslicer::RTV_Pattern::MatcherHandler_KF(KernelInfo& kernel, const clang::CompilerInstance& a_compiler)
{
  return std::move(std::make_unique<kslicer::UsedCodeFilter>(std::cout, *this, &kernel, a_compiler));
}

void kslicer::RTV_Pattern::ProcessCallArs_KF(const KernelCallInfo& a_call)
{
  auto pFoundKernel = kernels.find(a_call.originKernelName);
  if(pFoundKernel != kernels.end()) 
  {
    auto& actualParameters = a_call.descriptorSetsInfo;
    for(size_t argId = 0; argId<actualParameters.size(); argId++)
    {
      if(actualParameters[argId].argType == kslicer::KERN_CALL_ARG_TYPE::ARG_REFERENCE_LOCAL)
        pFoundKernel->second.args[argId].needFakeOffset = true; 
    }
  }
}

void kslicer::RTV_Pattern::ProcessKernelArg(KernelInfo::Arg& arg, const KernelInfo& a_kernel) const 
{
  auto pdef = GetAllPredefinedThreadIdNamesRTV();
  auto id   = std::find(pdef.begin(), pdef.end(), arg.name);
  arg.isThreadID = (id != pdef.end()); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void kslicer::RTV_Pattern::AddDispatchingHierarchy(const std::string& a_className, const std::string& a_makerName)
{
  std::cout << "   found class hierarchy: " << a_className.c_str() << " from " << a_makerName.c_str() << std::endl;

  DHierarchy hdata;
  hdata.interfaceName = kslicer::CutOffStructClass(a_className);
  hdata.makerName     = a_makerName;
  hdata.implementations.clear();
  m_vhierarchy[a_className] = hdata;
} 

void kslicer::RTV_Pattern::AddDispatchingKernel(const std::string& a_className, const std::string& a_kernelName)
{
  std::cout << "   found virtual kernel dispatch: " << a_className.c_str() << "::" << a_kernelName.c_str() << std::endl;
  m_vkernelPairs.push_back(std::pair(kslicer::CutOffStructClass(a_className), a_kernelName));
} 

void kslicer::RTV_Pattern::ProcessDispatchHierarchies(const std::vector<const clang::CXXRecordDecl*>& a_decls)
{
  //
  //
  for(auto& p : m_vhierarchy)
  {
    const clang::CXXRecordDecl* pBaseClass = nullptr;
    std::string className = kslicer::CutOffStructClass(p.first);
    
    // find target base class
    //
    for(const auto& decl : a_decls)
    {
      const std::string testName = decl->getNameAsString();
      if(testName == className)
      {
        pBaseClass = decl;
        break;
      }
      //std::cout << "  found class: " << testName.c_str() << std::endl;
    }

    p.second.interfaceDecl = pBaseClass;

    // find all derived classes for target base class
    //
    for(const auto& decl : a_decls)
    {
      if(decl->isDerivedFrom(pBaseClass))
      {
        DImplClass dImpl;
        dImpl.decl = decl;
        dImpl.name = decl->getNameAsString();
        p.second.implementations.push_back(dImpl);
      }
    }
  }
  
  // debug output
  //
  for(const auto& p : m_vhierarchy)
  {
    for(const auto& impl : p.second.implementations)
      std::cout << "  found " << p.first.c_str() << " --> " << impl.name.c_str() << std::endl;
  }

}


class TagSeeker : public clang::RecursiveASTVisitor<TagSeeker>
{
public:
  
  TagSeeker(const clang::CompilerInstance& a_compiler, std::vector<kslicer::DeclInClass>& a_constants, std::unordered_map<std::string, std::string>& a_tagByName) : 
            m_compiler(a_compiler), m_sm(a_compiler.getSourceManager()), m_knownConstants(a_constants), m_tagByClassName(a_tagByName) { m_tagByClassName.clear(); }

  bool VisitCXXMethodDecl(const clang::CXXMethodDecl* f)
  { 
    if(!f->hasBody())
      return true;

    // Get name of function
    const std::string fname = f->getNameInfo().getName().getAsString();
    if(fname == "GetTag" || fname == "GetTypeId")
    {
      const clang::QualType qThisType = f->getThisType();   
      const clang::QualType classType = qThisType->getPointeeType();
      const std::string thisTypeName  = kslicer::CutOffStructClass(classType.getAsString());
      const std::string funcBody      = kslicer::GetRangeSourceCode(f->getSourceRange(), m_compiler);
      
      for(const auto& decl : m_knownConstants)
      {
        auto pos = funcBody.find(decl.name);
        if(pos != std::string::npos)
        {
          m_tagByClassName[thisTypeName] = decl.name;
          break;
        }
      }
    }
    
    return true;
  }

private:

  const clang::SourceManager&    m_sm;
  const clang::CompilerInstance& m_compiler;
  std::vector<kslicer::DeclInClass>& m_knownConstants;
  std::unordered_map<std::string, std::string>& m_tagByClassName; 
};


void kslicer::RTV_Pattern::ExtractHierarchiesConstants(const clang::CompilerInstance& compiler, clang::tooling::ClangTool& Tool)
{
  for(auto& p : m_vhierarchy)
  {
    //// (1) get all constants inside interface
    //
    std::cout << "  process " << p.second.interfaceName.c_str() << std::endl;
    p.second.usedDecls = kslicer::ExtractTCFromClass(p.second.interfaceName, p.second.interfaceDecl, compiler, Tool);

    //// (2) juxtapose constant TAG and class implementation by analyzing GetTag() function
    //
    TagSeeker visitor(compiler, p.second.usedDecls, p.second.tagByClassName);
    for(auto impl : p.second.implementations)
      visitor.TraverseDecl(const_cast<clang::CXXRecordDecl*>(impl.decl));
  }

}
