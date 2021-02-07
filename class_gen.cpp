#include "kslicer.h"
#include "class_gen.h"
#include "ast_matchers.h"

#include <sstream>
#include <algorithm>

bool kslicer::MainFuncASTVisitor::VisitCXXMethodDecl(CXXMethodDecl* f) 
{
  if (f->hasBody())
  {
    // Get name of function
    const DeclarationNameInfo dni = f->getNameInfo();
    const DeclarationName dn      = dni.getName();
    const std::string fname       = dn.getAsString();

    mainFuncCmdName = fname + "Cmd";
    m_rewriter.ReplaceText(dni.getSourceRange(), mainFuncCmdName);
  }

  return true; // returning false aborts the traversal
}

std::string kslicer::MainFuncASTVisitor::MakeKernelCallCmdString(CXXMemberCallExpr* f)
{
  const DeclarationNameInfo dni = f->getMethodDecl()->getNameInfo();
  const DeclarationName dn      = dni.getName();
  const std::string fname       = dn.getAsString();

  const auto pKernelInfo = m_kernels.find(fname);
  assert(pKernelInfo != m_kernels.end());

  const std::string kernName    = m_pCodeInfo->RemoveKernelPrefix(fname);

  // extract arguments to form correct descriptor set
  // 
  //std::stringstream strOut1;
  //strOut1 << "_" << m_dsTagId;                                                    
  const auto args     = ExtractArgumentsOfAKernelCall(f);                                          
  const auto callSign = MakeKernellCallSignature(m_mainFuncName, args, pKernelInfo->second.usedVectors); // + strOut1.str();
  auto p2 = dsIdBySignature.find(callSign);
  if(p2 == dsIdBySignature.end())
  {
    dsIdBySignature[callSign] = allDescriptorSetsInfo.size();
    p2 = dsIdBySignature.find(callSign);
    KernelCallInfo call;
    call.kernelName         = kernName;
    call.originKernelName   = fname;
    call.callerName         = m_mainFuncName;
    call.descriptorSetsInfo = args;
    allDescriptorSetsInfo.push_back(call);
  }
  m_dsTagId++;

  std::string textOfCall = GetRangeSourceCode(f->getSourceRange(), m_compiler);
  std::string textOfArgs = textOfCall.substr( textOfCall.find("("));

  std::stringstream strOut;
  {
    // understand if we are inside the loop, or outside of it
    //
    auto pKernel = m_kernels.find(fname);
    assert(pKernel != m_kernels.end()); 

    auto callSourceRangeHash = kslicer::GetHashOfSourceRange(f->getSourceRange());
    auto p3 = m_mainFunc.CallsInsideFor.find(callSourceRangeHash);
    auto p4 = m_mainFunc.ExitExprIfCall.find(callSourceRangeHash);

    std::string flagsVariableName = "";
    if(p3 != m_mainFunc.CallsInsideFor.end())
    {
      flagsVariableName = "inForFlags";
      
      if(pKernel->second.isBoolTyped && p4 == m_mainFunc.ExitExprIfCall.end())
        flagsVariableName += "D";
      else if(p3->second.isNegative)
        flagsVariableName += "N";
    }
    else 
    {
      flagsVariableName = "outOfForFlags";
      if(pKernel->second.isBoolTyped && p4 == m_mainFunc.ExitExprIfCall.end())
        flagsVariableName += "D";
      else if(p4 != m_mainFunc.ExitExprIfCall.end() && p4->second.isNegative)
        flagsVariableName += "N";
    }
    
    // m_currThreadFlags
    strOut << "vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, ";
    strOut << kernName.c_str() << "Layout," << " 0, 1, " << "&m_allGeneratedDS[" << p2->second << "], 0, nullptr);" << std::endl;
    if(m_pCodeInfo->NeedThreadFlags())
      strOut << "  m_currThreadFlags = " << flagsVariableName.c_str() << ";" << std::endl;
    strOut << "  " << kernName.c_str() << "Cmd" << textOfArgs.c_str();
  }
  
  return strOut.str();
}

std::string kslicer::MainFuncASTVisitor::MakeServiceKernelCallCmdString(CallExpr* call)
{
  std::string kernName = "copyKernelFloat"; // extract from 'call' exact name of service function;
                                     // replace it with actual name we are going to used in generated HOST(!!!) code. 
                                     // for example it can be 'MyMemcpy' for 'memcpy' if in host code we have (MyMemcpyLayout, MyMemcpyPipeline, MyMemcpyDSLayout)
                                     // please note that you should init MyMemcpyLayout, MyMemcpyPipeline, MyMemcpyDSLayout yourself in the generated code!                                      
  
  auto memCpyArgs = ExtractArgumentsOfAKernelCall(call);

  std::vector<ArgReferenceOnCall> args(2); // TODO: extract corretc arguments from memcpy (CallExpr* call)
  {
    args[0].argType = memCpyArgs[0].argType;
    args[0].varName = memCpyArgs[0].varName;

    args[1].argType = memCpyArgs[1].argType;
    args[1].varName = memCpyArgs[1].varName;
  }

  const auto callSign = MakeKernellCallSignature(m_mainFuncName, args, std::unordered_set<std::string>()); // + strOut1.str();
  auto p2 = dsIdBySignature.find(callSign);
  if(p2 == dsIdBySignature.end())
  {
    dsIdBySignature[callSign] = allDescriptorSetsInfo.size();
    p2 = dsIdBySignature.find(callSign);
    KernelCallInfo call;
    call.kernelName         = kernName;
    call.originKernelName   = kernName;
    call.callerName         = m_mainFuncName;
    call.descriptorSetsInfo = args;
    allDescriptorSetsInfo.push_back(call);
  }
  m_dsTagId++;

  std::stringstream strOut;
  strOut << "vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, ";
  strOut << kernName.c_str() << "Layout," << " 0, 1, " << "&m_allGeneratedDS[" << p2->second << "], 0, nullptr);" << std::endl;
  strOut << "  " << kernName.c_str() << "Cmd(" << memCpyArgs[2].varName << " / sizeof(float))";

  return strOut.str();
}



bool kslicer::MainFuncASTVisitor::VisitCXXMemberCallExpr(CXXMemberCallExpr* f)
{
  // Get name of function
  const DeclarationNameInfo dni = f->getMethodDecl()->getNameInfo();
  const DeclarationName dn      = dni.getName();
  const std::string fname       = dn.getAsString();

  if(m_pCodeInfo->IsKernel(fname))
  {
    std::string callStr = MakeKernelCallCmdString(f);

    auto p2 = m_alreadyProcessedCalls.find(kslicer::GetHashOfSourceRange(f->getSourceRange()));
    if(p2 == m_alreadyProcessedCalls.end())
    {
      m_rewriter.ReplaceText(f->getSourceRange(), callStr); // getExprLoc
    }
  }

  return true; 
}

bool kslicer::MainFuncASTVisitor::VisitCallExpr(CallExpr* call)
{
  if(isa<CXXMemberCallExpr>(call)) // because we process them in "VisitCXXMemberCallExpr"
    return true;

  const FunctionDecl* fDecl = call->getDirectCallee();  
  if(fDecl == nullptr)             // definitely can't process nullpointer 
    return true;

  // Get name of function
  //
  const DeclarationNameInfo dni = fDecl->getNameInfo();
  const DeclarationName dn      = dni.getName();
  const std::string fname       = dn.getAsString();
  
  if(fname == "memcpy")
  {
    std::string testStr = MakeServiceKernelCallCmdString(call);
    m_rewriter.ReplaceText(call->getSourceRange(), testStr);
  }

  return true;
}

bool kslicer::MainFuncASTVisitor::VisitIfStmt(IfStmt* ifExpr)
{
  Expr* conBody = ifExpr->getCond();
  if(isa<UnaryOperator>(conBody)) // if(!kernel_XXX(...))
  {
    const auto bodyOp = dyn_cast<UnaryOperator>(conBody);
    conBody = bodyOp->getSubExpr();
  }

  if(isa<CXXMemberCallExpr>(conBody))
  {
    CXXMemberCallExpr* f = dyn_cast<CXXMemberCallExpr>(conBody); // extract kernel_XXX(...)
    
    // Get name of function
    const DeclarationNameInfo dni = f->getMethodDecl()->getNameInfo();
    const DeclarationName dn      = dni.getName();
    const std::string fname       = dn.getAsString();
  
    if(m_pCodeInfo->IsKernel(fname))
    {
      std::string callStr = MakeKernelCallCmdString(f);
      m_rewriter.ReplaceText(ifExpr->getSourceRange(), callStr);
      m_alreadyProcessedCalls.insert( kslicer::GetHashOfSourceRange(f->getSourceRange()) );
    }
  }

  return true;
}

std::vector<kslicer::ArgReferenceOnCall> kslicer::MainFuncASTVisitor::ExtractArgumentsOfAKernelCall(CallExpr* f)
{
  std::vector<kslicer::ArgReferenceOnCall> args; 
  args.reserve(20);

  auto predefinedNames = GetAllPredefinedThreadIdNamesRTV();
  
  for(size_t i=0;i<f->getNumArgs();i++)
  {
    const Expr* currArgExpr = f->getArgs()[i];
    assert(currArgExpr != nullptr);
    auto sourceRange = currArgExpr->getSourceRange();
    std::string text = GetRangeSourceCode(sourceRange, m_compiler);
  
    ArgReferenceOnCall arg;    
    if(text[0] == '&')
    {
      arg.umpersanned = true;
      text = text.substr(1);

      auto pClassVar = m_allClassMembers.find(text);
      if(pClassVar != m_allClassMembers.end())
        arg.argType = KERN_CALL_ARG_TYPE::ARG_REFERENCE_CLASS_POD;
    }
    else if(text.find(".data()") != std::string::npos) 
    {
      std::string varName = text.substr(0, text.find(".data()"));
      auto pClassVar = m_allClassMembers.find(varName);
      if(pClassVar == m_allClassMembers.end())
        std::cout << "[KernelCallError]: vector<...> variable '" << varName.c_str() << "' was not found in class!" << std::endl; 
      else
        pClassVar->second.usedInMainFn = true;

      arg.argType = KERN_CALL_ARG_TYPE::ARG_REFERENCE_CLASS_VECTOR;
    }

    auto elementId = std::find(predefinedNames.begin(), predefinedNames.end(), text); // exclude predefined names from arguments
    if(elementId != predefinedNames.end())
      arg.argType = KERN_CALL_ARG_TYPE::ARG_REFERENCE_THREAD_ID;

    arg.varName = text;
    args.push_back(arg); 
  }

  for(auto& arg : args)  // in this loop we have to define argument (actual parameter) type
  {
    if(arg.argType != KERN_CALL_ARG_TYPE::ARG_REFERENCE_UNKNOWN_TYPE)
      continue;
    
    auto p2 = m_argsOfMainFunc.find(arg.varName);
    if(p2 != m_argsOfMainFunc.end())
      arg.argType = KERN_CALL_ARG_TYPE::ARG_REFERENCE_ARG;

    auto p3 = m_mainFuncLocals.find(arg.varName);
    if(p3 != m_mainFuncLocals.end())
      arg.argType = KERN_CALL_ARG_TYPE::ARG_REFERENCE_LOCAL;
  }

  // for(auto& arg : args) // check we don't have unknown types of arguments // well, in fact now we can ) Use arguments
  // {
  //   if(arg.argType == KERN_CALL_ARG_TYPE::ARG_REFERENCE_UNKNOWN_TYPE)
  //   {
  //     std::stringstream strOut;
  //     strOut << "  WARNING: expr '" << arg.varName.c_str() << "' was not classified"; 
  //     kslicer::PrintError(strOut.str(), f->getSourceRange(), m_sm);  
  //   }
  // }

  return args;
}

std::string kslicer::MakeKernellCallSignature(const std::string& a_mainFuncName, const std::vector<ArgReferenceOnCall>& a_args, const std::unordered_set<std::string>& a_usedVectors)
{
  std::stringstream strOut;
  for(const auto& arg : a_args)
  {
    switch(arg.argType)
    {
      case KERN_CALL_ARG_TYPE::ARG_REFERENCE_LOCAL:
      strOut << "[L]";
      break;

      case KERN_CALL_ARG_TYPE::ARG_REFERENCE_ARG:
      strOut << "[A][" << a_mainFuncName.c_str() << "]" ;
      break;

      case KERN_CALL_ARG_TYPE::ARG_REFERENCE_CLASS_VECTOR:
      strOut << "[V]";
      break;
      
      case KERN_CALL_ARG_TYPE::ARG_REFERENCE_CLASS_POD:
      strOut << "[P]";
      break;

      case KERN_CALL_ARG_TYPE::ARG_REFERENCE_THREAD_ID:
      strOut << "[T]";
      break;

      default:
      strOut << "[U]";
      break;
    };

    strOut << arg.varName.c_str();
  }

  for(const auto& vecName : a_usedVectors)
    strOut << "[MV][" << vecName.c_str() << "]";

  return strOut.str();
}

void kslicer::MainClassInfo::AddTempBufferToKernel(const std::string a_buffName, KernelInfo& a_kernel, size_t a_bufferSizeInBytes)
{
  // (0) make full name for such a vector
  //
  std::stringstream strOut;
  strOut << "tmp_" << a_buffName << "_" << a_bufferSizeInBytes;
  
  std::string buffName = strOut.str();

  // (1) append vector to this->allDataMembers
  //
  auto pFoundMember = allDataMembers.find(buffName);
  if(pFoundMember == allDataMembers.end())
  {
    DataMemberInfo vecMemberTmp;
    vecMemberTmp.name          = buffName;
    vecMemberTmp.type          = "std::vector<uint>";
    vecMemberTmp.sizeInBytes   = 4; // not used by containers
    vecMemberTmp.isContainer   = true;
    vecMemberTmp.usedInKernel  = true;
    vecMemberTmp.containerType = "std::vector";
    vecMemberTmp.containerDataType = "uint";
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

  kslicer::MainFuncASTVisitor rv(rewrite2, compiler, a_mainFunc, inOutParamList, this); // ==> write this->allDescriptorSetsInfo during 'TraverseDecl'
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
      strOut << "  const uint32_t outOfForFlagsN = KGEN_FLAG_RETURN | KGEN_FLAG_SET_EXIT_NEGATIVE;" << std::endl;
      strOut << "  const uint32_t inForFlagsN    = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK | KGEN_FLAG_SET_EXIT_NEGATIVE;" << std::endl;
      strOut << "  const uint32_t outOfForFlagsD = KGEN_FLAG_RETURN | KGEN_FLAG_DONT_SET_EXIT;" << std::endl;
      strOut << "  const uint32_t inForFlagsD    = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK | KGEN_FLAG_DONT_SET_EXIT;" << std::endl;
      strOut << "  vkCmdFillBuffer(a_commandBuffer, " << a_mainFunc.Name.c_str() << "_local.threadFlagsBuffer , 0, VK_WHOLE_SIZE, 0); // zero thread flags, mark all threads to be active" << std::endl;
      strOut << "  VkMemoryBarrier fillBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT }; " << std::endl;
      strOut << "  vkCmdPipelineBarrier(a_commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &fillBarrier, 0, nullptr, 0, nullptr); " << std::endl;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool kslicer::KernelReplacerASTVisitor::VisitMemberExpr(MemberExpr* expr)
{
  ValueDecl* pValueDecl =	expr->getMemberDecl();
  
  if(!isa<FieldDecl>(pValueDecl))
    return true;

  FieldDecl* pFieldDecl  = dyn_cast<FieldDecl>(pValueDecl);
  assert(pFieldDecl != nullptr);
  RecordDecl* pRecodDecl = pFieldDecl->getParent();
  assert(pRecodDecl != nullptr);

  const std::string thisTypeName = pRecodDecl->getNameAsString();
  if(thisTypeName != m_mainClassName)
  {
    // process access to arguments payload->xxx
    //
    Expr* baseExpr = expr->getBase(); 
    assert(baseExpr != nullptr);

    const std::string baseName = GetRangeSourceCode(baseExpr->getSourceRange(), m_compiler);

    size_t foundId  = size_t(-1);
    bool needOffset = false;
    for(size_t i=0;i<m_args.size();i++)
    {
      if(m_args[i].name == baseName)
      {
        foundId    = i;
        needOffset = m_args[i].needFakeOffset;
        break;
      }
    }

    if(foundId == size_t(-1)) // we didn't found 'payload' in kernela arguments, so just ignore it
      return true;
    
    // now split 'payload->xxx' to 'payload' (baseName) and 'xxx' (memberName); 
    // 
    const std::string exprContent = GetRangeSourceCode(expr->getSourceRange(), m_compiler);
    auto pos = exprContent.find("->");
    assert(pos != std::string::npos);

    const std::string memberName = exprContent.substr(pos+2);

    if(needOffset)
       m_rewriter.ReplaceText(expr->getSourceRange(), baseName + "[" + m_fakeOffsetExp + "]." + memberName);

    return true;
  }

  // process access to class member data
  // 

  // (1) get variable offset in buffer by its name 
  //
  const std::string fieldName = pFieldDecl->getNameAsString(); 
  const auto pMember = m_variables.find(fieldName);
  if(pMember == m_variables.end())
    return true;

  // (2) put ubo->var instead of var, leave containers as they are
  // process arrays and large data structures because small can be read once in the neggining of kernel
  //
  const bool isInLoopInitPart = expr->getSourceRange().getBegin() <= m_currKernel.loopOutsidesInit.getEnd();
  const bool hasLargeSize     = (pMember->second.sizeInBytes > kslicer::READ_BEFORE_USE_THRESHOLD);
  if(!pMember->second.isContainer && (isInLoopInitPart || pMember->second.isArray || hasLargeSize)) 
  {
    //const std::string debugMe = GetRangeSourceCode(expr->getSourceRange(), m_compiler);
    std::string rewrittenName = m_codeInfo->pShaderCC->UBOAccess(pMember->second.name);
    m_rewriter.ReplaceText(expr->getSourceRange(), rewrittenName);
  }
  
  return true;
}

bool kslicer::KernelReplacerASTVisitor::VisitCXXMemberCallExpr(CXXMemberCallExpr* f)
{
  // Get name of function
  //
  const DeclarationNameInfo dni = f->getMethodDecl()->getNameInfo();
  const DeclarationName dn      = dni.getName();
  const std::string fname       = dn.getAsString();
  
  // Get name of "this" type; we should check wherther this member is std::vector<T>  
  //
  const clang::QualType qt = f->getObjectType();
  const std::string& thisTypeName = qt.getAsString();
  //const auto* fieldTypePtr = qt.getTypePtr(); 
  //assert(fieldTypePtr != nullptr);
  //auto typeDecl = fieldTypePtr->getAsRecordDecl();  
  CXXRecordDecl* typeDecl = f->getRecordDecl(); 

  const bool isVector = (typeDecl != nullptr && isa<ClassTemplateSpecializationDecl>(typeDecl)) && thisTypeName.find("vector<") != std::string::npos; 
  
  if(isVector)
  {
    const std::string exprContent = GetRangeSourceCode(f->getSourceRange(), m_compiler);
    const auto posOfPoint         = exprContent.find(".");
    const std::string memberNameA = exprContent.substr(0, posOfPoint);

    if(fname == "size" || fname == "capacity")
    {
      const std::string memberNameB = memberNameA + "_" + fname;
      m_rewriter.ReplaceText(f->getSourceRange(), m_codeInfo->pShaderCC->UBOAccess(memberNameB) );
    }
    else if(fname == "resize")
    {
      if(f->getSourceRange().getBegin() <= m_currKernel.loopOutsidesInit.getEnd())
      {
        assert(f->getNumArgs() == 1);
        const Expr* currArgExpr  = f->getArgs()[0];
        std::string newSizeValue = kslicer::GetRangeSourceCode(currArgExpr->getSourceRange(), m_compiler); 
        std::string memberNameB  = memberNameA + "_size = " + newSizeValue;
        m_rewriter.ReplaceText(f->getSourceRange(), m_codeInfo->pShaderCC->UBOAccess(memberNameB) );
      }
    }
    else if(fname == "push_back")
    {
      m_rewriter.ReplaceText(f->getSourceRange(), "//replace push_back()");
    }
    else if(fname == "data")
    {
      m_rewriter.ReplaceText(f->getSourceRange(), memberNameA);
    }
    else 
    {
      kslicer::PrintError(std::string("Unsuppoted std::vector method") + fname, f->getSourceRange(), m_compiler.getSourceManager());
    }
  }
 
  return true;
}

bool kslicer::KernelReplacerASTVisitor::VisitReturnStmt(ReturnStmt* ret)
{
  Expr* retExpr = ret->getRetValue();
  if (!retExpr || !m_kernelIsBoolTyped)
    return true;

  std::string retExprText = GetRangeSourceCode(retExpr->getSourceRange(), m_compiler);
  m_rewriter.ReplaceText(ret->getSourceRange(), std::string("kgenExitCond = ") + retExprText + "; goto KGEN_EPILOG");
  return true;
}


bool kslicer::KernelReplacerASTVisitor::CheckIfExprHasArgumentThatNeedFakeOffset(const std::string& exprStr)
{
  bool needOffset = false;
  for(const auto arg: m_args)
  {
    if(exprStr.find(arg.name) != std::string::npos)
    {
      if(arg.needFakeOffset)
      {
        needOffset = true;
        break;
      }
    }
  }

  return needOffset;
}

bool kslicer::KernelReplacerASTVisitor::VisitUnaryOperator(UnaryOperator* expr)
{
  const auto op = expr->getOpcodeStr(expr->getOpcode());
  //const auto opCheck = std::string(op);
  //std::string opCheck2 = GetRangeSourceCode(expr->getSourceRange(), m_compiler);

  Expr* subExpr =	expr->getSubExpr();
  if(subExpr == nullptr)
    return true;

  if(op == "++" || op == "--") // detect ++ and -- for reduction
  {
    auto opRange = expr->getSourceRange();
    if(opRange.getEnd() <= m_currKernel.loopInsides.getBegin() || opRange.getBegin() >= m_currKernel.loopInsides.getEnd() ) // not inside loop
      return true;   
  
    const auto op = expr->getOpcodeStr(expr->getOpcode());
    std::string leftStr = GetRangeSourceCode(subExpr->getSourceRange(), m_compiler);
    
    auto p = m_currKernel.usedMembers.find(leftStr);
    if(p != m_currKernel.usedMembers.end())
    {
      KernelInfo::ReductionAccess access;
      access.type      = KernelInfo::REDUCTION_TYPE::UNKNOWN;
      access.rightExpr = "";
      access.dataType  = subExpr->getType().getAsString();

      if(op == "++")
        access.type    = KernelInfo::REDUCTION_TYPE::ADD_ONE;
      else if(op == "--")
        access.type    = KernelInfo::REDUCTION_TYPE::SUB_ONE;

      m_currKernel.subjectedToReduction[leftStr] = access;
      m_rewriter.ReplaceText(expr->getSourceRange(), leftStr + "Shared[get_local_id(0)]++");
    }
  }

  // detect " *(something)"
  //
  if(expr->canOverflow() || op != "*") // -UnaryOperator ...'LiteMath::uint':'unsigned int' lvalue prefix '*' cannot overflow
    return true;

  std::string exprInside = GetRangeSourceCode(subExpr->getSourceRange(), m_compiler);

  // check if this argument actually need fake Offset
  //
  const bool needOffset = CheckIfExprHasArgumentThatNeedFakeOffset(exprInside);

  if(needOffset)
    m_rewriter.ReplaceText(expr->getSourceRange(), exprInside + "[" + m_fakeOffsetExp + "]");

  return true;
}

bool kslicer::KernelReplacerASTVisitor::VisitCompoundAssignOperator(CompoundAssignOperator* expr)
{
  auto opRange = expr->getSourceRange();
  if(opRange.getEnd() <= m_currKernel.loopInsides.getBegin() || opRange.getBegin() >= m_currKernel.loopInsides.getEnd() ) // not inside loop
    return true;   

  const Expr* lhs     = expr->getLHS();
  const Expr* rhs     = expr->getRHS();
  const auto  op      = expr->getOpcodeStr();
  std::string leftStr = GetRangeSourceCode(lhs->getSourceRange(), m_compiler);

  auto p = m_currKernel.usedMembers.find(leftStr);
  if(p != m_currKernel.usedMembers.end())
  {
    KernelInfo::ReductionAccess access;
    access.type      = KernelInfo::REDUCTION_TYPE::UNKNOWN;
    access.rightExpr = GetRangeSourceCode(rhs->getSourceRange(), m_compiler);
    access.dataType  = rhs->getType().getAsString();

    if(op == "+=")
      access.type    = KernelInfo::REDUCTION_TYPE::ADD;
    else if(op == "*=")
      access.type    = KernelInfo::REDUCTION_TYPE::MUL;
    else if(op == "-=")
      access.type    = KernelInfo::REDUCTION_TYPE::SUB;
      
    m_currKernel.subjectedToReduction[leftStr] = access;
    m_rewriter.ReplaceText(expr->getSourceRange(), leftStr + "Shared[get_local_id(0)] " + access.GetOp() + " " + access.rightExpr);
  }

  return true;
}

bool kslicer::KernelReplacerASTVisitor::VisitBinaryOperator(BinaryOperator* expr) // detect reduction like m_var = F(m_var,expr)
{
  auto opRange = expr->getSourceRange();
  if(opRange.getEnd() <= m_currKernel.loopInsides.getBegin() || opRange.getBegin() >= m_currKernel.loopInsides.getEnd() ) // not inside loop
    return true;  

  const auto op = expr->getOpcodeStr();
  if(op != "=")
    return true;
  
  const Expr* lhs = expr->getLHS();
  const Expr* rhs = expr->getRHS();

  if(!isa<MemberExpr>(lhs))
    return true;

  std::string leftStr  = GetRangeSourceCode(lhs->getSourceRange(), m_compiler);
  auto p = m_currKernel.usedMembers.find(leftStr);
  if(p == m_currKernel.usedMembers.end())
    return true;
  
  if(!isa<CallExpr>(rhs))
  {
    PrintError("unsupported expression for reduction via assigment inside loop; must be 'a = f(a,b)'", rhs->getSourceRange(), m_compiler.getSourceManager());
    return true;
  }
  
  auto call    = dyn_cast<CallExpr>(rhs);
  auto numArgs = call->getNumArgs();
  if(numArgs != 2)
  {
    PrintError("function which is used in reduction must have 2 args; a = f(a,b)'", expr->getSourceRange(), m_compiler.getSourceManager());
    return true;
  }
  
  const Expr* arg0 = call->getArg(0);
  const Expr* arg1 = call->getArg(1);

  std::string arg0Str = GetRangeSourceCode(arg0->getSourceRange(), m_compiler);
  std::string arg1Str = GetRangeSourceCode(arg1->getSourceRange(), m_compiler);
  
  std::string secondArg;
  if(arg0Str == leftStr)
  {
    secondArg = arg1Str;
  }
  else if(arg1Str == leftStr)
  {
    secondArg = arg0Str;
  }
  else
  {
    PrintError("incorrect arguments of reduction function, one of them must be same as assigment result; a = f(a,b)'", call->getSourceRange(), m_compiler.getSourceManager());
    return true;
  }

  const std::string callExpr = GetRangeSourceCode(call->getSourceRange(), m_compiler);
  const std::string fname    = callExpr.substr(0, callExpr.find_first_of('('));
  
  KernelInfo::ReductionAccess access;
 
  access.type      = KernelInfo::REDUCTION_TYPE::FUNC;
  access.funcName  = fname;
  access.rightExpr = secondArg;
  access.dataType  = lhs->getType().getAsString();

  m_currKernel.subjectedToReduction[leftStr] = access;
  m_rewriter.ReplaceText(expr->getSourceRange(), "// func. reduce " + leftStr + " with " + fname + " and " + access.rightExpr);

  return true;
}

//// tid, fakeOffset(tidX,tidY,kgen_iNumElementsX) or fakeOffset2(tidX,tidY,tidX,kgen_iNumElementsX, kgen_iNumElementsY)
//
std::string GetFakeOffsetExpression(const kslicer::KernelInfo& a_funcInfo, const std::vector<kslicer::MainClassInfo::ArgTypeAndNamePair>& threadIds) 
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

std::string kslicer::MainClassInfo::VisitAndRewrite_KF(KernelInfo& a_funcInfo, const clang::CompilerInstance& compiler, std::string& a_outLoopInitCode)
{
  const CXXMethodDecl* a_node = a_funcInfo.astNode;
  //a_node->dump();

  std::string fakeOffsetExpr = GetFakeOffsetExpression(a_funcInfo, GetKernelTIDArgs(a_funcInfo));

  Rewriter rewrite2;
  rewrite2.setSourceMgr(compiler.getSourceManager(), compiler.getLangOpts());

  kslicer::KernelReplacerASTVisitor rv(rewrite2, compiler, this, a_funcInfo, fakeOffsetExpr);
  rv.TraverseDecl(const_cast<clang::CXXMethodDecl*>(a_node));
  
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
    std::string kernelSourceCode = GetRangeSourceCode(sourceRange, compiler);
    
    std::string kernelCmdDecl = kernelSourceCode.substr(0, kernelSourceCode.find(")")+1);
    assert(ReplaceFirst(kernelCmdDecl, a_mainClassName + "::", ""));
    
    kernelCmdDecl = a_codeInfo.RemoveKernelPrefix(kernelCmdDecl);

    assert(ReplaceFirst(kernelCmdDecl,"(", "Cmd("));
    if(k.second.isBoolTyped)
      ReplaceFirst(kernelCmdDecl,"bool ", "void ");
    k.second.DeclCmd = kernelCmdDecl;
  }
}
