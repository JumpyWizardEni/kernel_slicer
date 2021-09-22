#include "initial_pass.h"
#include <iostream>

void kslicer::SplitContainerTypes(const clang::ClassTemplateSpecializationDecl* specDecl, std::string& a_containerType, std::string& a_containerDataType)
{
  a_containerType = specDecl->getNameAsString();      
  const auto& templateArgs = specDecl->getTemplateArgs();
        
  if(templateArgs.size() > 0)
    a_containerDataType = templateArgs[0].getAsType().getAsString();
  else
    a_containerDataType = "unknown";
}


static kslicer::KernelInfo::Arg ProcessParameter(clang::ParmVarDecl *p) 
{
  clang::QualType q = p->getType();

  kslicer::KernelInfo::Arg arg;
  arg.name = p->getNameAsString();
  arg.type = clang::QualType::getAsString(q.split(), clang::PrintingPolicy{ {} });
  arg.size = 1;
  if (q->isPointerType()) {
    arg.size      = 1; // Because C always pass reference
    arg.isPointer = true;
  }
  else if(q->isReferenceType()) {
    arg.isReference = true;
    auto dataType = q.getNonReferenceType(); 
    auto typeDecl = dataType->getAsRecordDecl();
      
    if(typeDecl != nullptr && clang::isa<clang::ClassTemplateSpecializationDecl>(typeDecl))
    {
      arg.isContainer = true;
      auto specDecl = clang::dyn_cast<clang::ClassTemplateSpecializationDecl>(typeDecl);   
      kslicer::SplitContainerTypes(specDecl, arg.containerType, arg.containerDataType);
    }
  }

  //TODO: q->isConstantArrayType() --> isArray

  return arg;
}

void kslicer::InitialPassRecursiveASTVisitor::ProcessKernelDef(const CXXMethodDecl *f, std::unordered_map<std::string, KernelInfo>& a_funcList, const std::string& a_className) 
{
  if (!f || !f->hasBody()) 
    return;

  const QualType retType  = f->getReturnType();
  std::string retTypeName = retType.getAsString();

  DeclarationNameInfo dni = f->getNameInfo();
  DeclarationName dn      = dni.getName();
  
  KernelInfo info;
  info.name        = dn.getAsString();
  info.className   = a_className;
  info.astNode     = f;
  info.return_type = retType.getAsString();
  info.isBoolTyped = retType.isTrivialType(m_astContext) && (retTypeName == "bool" || retTypeName == "_Bool");
  
  if(retType->isPointerType())
  {
    auto qtOfClass    = retType->getPointeeType(); 
    info.return_class = qtOfClass.getAsString();
  }

  for (unsigned int i = 0; i < f->getNumParams(); ++i) {
    info.args.push_back(ProcessParameter(f->parameters()[i]));
  }

  if(a_className == MAIN_CLASS_NAME)
    a_funcList[info.name] = info;
  else
    a_funcList[a_className + "::" + info.name] = info;
}


bool kslicer::InitialPassRecursiveASTVisitor::VisitCXXRecordDecl(CXXRecordDecl* record)
{
  if(!record->hasDefinition())
    return true;

  const auto pType = record->getTypeForDecl(); 
  if(pType == nullptr)
    return true;

  const QualType qt   = pType->getLocallyUnqualifiedSingleStepDesugaredType();
  const auto typeName = qt.getAsString();

  if(typeName == std::string("class ") + MAIN_CLASS_NAME || typeName == std::string("struct ") + MAIN_CLASS_NAME)
    m_mainClassASTNode = record;
  else if(!record->isPOD())
    m_classList.push_back(record); // rememer for futher processing of complex classes
  
  return true;
}

bool kslicer::InitialPassRecursiveASTVisitor::NeedToProcessDeclInFile(std::string a_fileName)
{
  bool needInsertToKernels = false;                     // do we have to process this declaration to further insert it to GLSL/CL ?
  for(auto folder : m_codeInfo.includeCPPFolders)       //
  {
    if(a_fileName.find(folder) != std::string::npos)
    {
      needInsertToKernels = true;
      break;
    }
  }
  return needInsertToKernels;
}

bool kslicer::InitialPassRecursiveASTVisitor::VisitTypeDecl(TypeDecl* type)
{
  const FileEntry* Entry = m_sourceManager.getFileEntryForID(m_sourceManager.getFileID(type->getLocation()));
  std::string FileName   = Entry->getName().str();
  if(!NeedToProcessDeclInFile(FileName))
    return true;

  if(isa<CXXRecordDecl>(type)) 
  {
    // currently we don't put polimorphic C++ classes to shaders, in far future we need to process them in special way probably
    //
    CXXRecordDecl* pCXXDecl = dyn_cast<CXXRecordDecl>(type);
    //if(!pCXXDecl->isCLike())
    //  return true;
    if(pCXXDecl->isPolymorphic() || pCXXDecl->isAbstract())
      return true;   
  }

  kslicer::DeclInClass decl;
  if(isa<RecordDecl>(type))
  {
    RecordDecl* pRecord = dyn_cast<RecordDecl>(type);
    decl.name      = pRecord->getNameAsString();
    decl.type      = pRecord->getNameAsString();
    decl.srcRange  = pRecord->getSourceRange ();                    
    decl.srcHash   = kslicer::GetHashOfSourceRange(decl.srcRange);  
    decl.order     = m_currId;
    decl.kind      = kslicer::DECL_IN_CLASS::DECL_STRUCT;
    decl.extracted = true;
    m_transferredDecl[decl.name] = decl;
    m_currId++;
  }
  else if(isa<TypedefDecl>(type))
  {
    TypedefDecl* pTargetTpdf = dyn_cast<TypedefDecl>(type);
    const auto qt  = pTargetTpdf->getUnderlyingType();
    decl.name      = pTargetTpdf->getNameAsString();
    decl.type      = qt.getAsString();
    decl.srcRange  = pTargetTpdf->getSourceRange();                
    decl.srcHash   = kslicer::GetHashOfSourceRange(decl.srcRange);
    decl.order     = m_currId;
    decl.kind      = kslicer::DECL_IN_CLASS::DECL_TYPEDEF;
    decl.extracted = true;
    m_transferredDecl[decl.name] = decl;
    m_currId++;
  }
  else if(isa<EnumDecl>(type))
  {
    EnumDecl* pEnumDecl = dyn_cast<EnumDecl>(type);

    for(auto it = pEnumDecl->enumerator_begin(); it != pEnumDecl->enumerator_end(); ++it)
    {
      EnumConstantDecl* pConstntDecl = (*it);
      decl.name      = pConstntDecl->getNameAsString();
      decl.type      = "const uint"; 
      decl.srcRange  = pConstntDecl->getInitExpr()->getSourceRange();                    
      decl.srcHash   = kslicer::GetHashOfSourceRange(decl.srcRange);  
      decl.order     = m_currId;
      decl.kind      = kslicer::DECL_IN_CLASS::DECL_CONSTANT;
      decl.extracted = true;
      m_transferredDecl[decl.name] = decl;
      m_currId++;
    }
 
  }

  //std::string text = GetRangeSourceCode(type->getSourceRange(), m_compiler);     
  return true;
}

bool kslicer::InitialPassRecursiveASTVisitor::VisitVarDecl(VarDecl* pTargetVar)
{
  const FileEntry* Entry = m_sourceManager.getFileEntryForID(m_sourceManager.getFileID(pTargetVar->getLocation()));
  std::string FileName   = Entry->getName().str();
  if(!NeedToProcessDeclInFile(FileName))
    return true;

  kslicer::DeclInClass decl;

  if(pTargetVar->isConstexpr())
  {
    decl.name      = pTargetVar->getNameAsString();
    decl.type      = pTargetVar->getType().getAsString(); 
    auto posOfDD = decl.type.find("::");
    if(posOfDD != std::string::npos)
      decl.type = decl.type.substr(posOfDD+2);

    decl.srcRange  = pTargetVar->getInit()->getSourceRange();                    
    decl.srcHash   = kslicer::GetHashOfSourceRange(decl.srcRange);  
    decl.order     = m_currId;
    decl.kind      = kslicer::DECL_IN_CLASS::DECL_CONSTANT;
    decl.extracted = true;
    m_transferredDecl[decl.name] = decl;
    m_currId++;
  }

  return true;
}

std::vector<kslicer::DeclInClass> kslicer::InitialPassRecursiveASTVisitor::GetExtractedDecls()
{
  std::vector<kslicer::DeclInClass> generalDecls; 
  generalDecls.reserve(m_transferredDecl.size());
  for(const auto decl : m_transferredDecl)
    generalDecls.push_back(decl.second);
  std::sort(generalDecls.begin(), generalDecls.end(), [](const auto& a, const auto& b) { return a.order < b.order; } );
  return generalDecls;
}

bool kslicer::InitialPassRecursiveASTVisitor::VisitCXXMethodDecl(CXXMethodDecl* f) 
{
  if(f->isStatic())
    return true;

  if (f->hasBody())
  {
    // Get name of function
    const DeclarationNameInfo dni = f->getNameInfo();
    const DeclarationName dn      = dni.getName();
    const std::string fname       = dn.getAsString();
    
    const QualType qThisType = f->getThisType();   
    const QualType classType = qThisType.getTypePtr()->getPointeeType();
    std::string thisTypeName = classType.getAsString();

    if(m_codeInfo.IsKernel(fname))
    {
      if(thisTypeName == std::string("class ") + MAIN_CLASS_NAME || thisTypeName == std::string("struct ") + MAIN_CLASS_NAME)
      {
        ProcessKernelDef(f, functions, MAIN_CLASS_NAME); // MAIN_CLASS_NAME::f ==> functions
        std::cout << "  found member kernel " << MAIN_CLASS_NAME.c_str() << "::" << fname.c_str() << std::endl;
      }
      else // extract other kernels and classes
      {
        thisTypeName = kslicer::CutOffStructClass(thisTypeName);
        ProcessKernelDef(f, otherFunctions, thisTypeName); // thisTypeName::f ==> otherFunctions
        std::cout << "  found other kernel " << thisTypeName.c_str() << "::" << fname.c_str() << std::endl;
      }
      
    }
    else if(m_mainFuncts.find(fname) != m_mainFuncts.end())
    {
      m_mainFuncNodes[fname] = f;
      //std::cout << "main function has found:\t" << fname.c_str() << std::endl;
      //f->dump();
    }
    else
    {
      //std::cout << "  --> found member func " <<  thisTypeName.c_str() << "::" << fname.c_str() << std::endl;
    }
  }

  return true; // returning false aborts the traversal
}

kslicer::DataMemberInfo kslicer::ExtractMemberInfo(clang::FieldDecl* fd, const clang::ASTContext& astContext)
{
  const clang::QualType qt = fd->getType();

  kslicer::DataMemberInfo member;
  member.name        = fd->getName().str();
  member.type        = qt.getAsString();
  member.sizeInBytes = 0; 
  member.offsetInTargetBuffer = 0;

  // now we should check werther this field is std::vector<XXX> or just XXX; 
  //
  const clang::Type* fieldTypePtr = qt.getTypePtr(); 
  assert(fieldTypePtr != nullptr);
  if(fieldTypePtr->isPointerType()) // we ignore pointers due to we can't pass them to GPU correctly
  {
    member.isPointer = true;
    return member;
  }

  auto typeDecl = fieldTypePtr->getAsRecordDecl();  
  if(fieldTypePtr->isConstantArrayType())
  {
    auto arrayType = clang::dyn_cast<clang::ConstantArrayType>(fieldTypePtr); 
    assert(arrayType != nullptr);
    clang::QualType qtOfElem = arrayType->getElementType(); 
    member.containerDataType = qtOfElem.getAsString(); 
    member.arraySize         = arrayType->getSize().getLimitedValue();      
    auto typeInfo      = astContext.getTypeInfo(qt);
    member.sizeInBytes = typeInfo.Width / 8; 
    member.isArray     = true;
  }  
  else if (typeDecl != nullptr && clang::isa<clang::ClassTemplateSpecializationDecl>(typeDecl)) 
  {
    member.isContainer = true;
    auto specDecl = clang::dyn_cast<clang::ClassTemplateSpecializationDecl>(typeDecl); 
    kslicer::SplitContainerTypes(specDecl, member.containerType, member.containerDataType);
    //std::cout << "  found container of type " << member.containerType.c_str() << ", which data type is " <<  member.containerDataType.c_str() << std::endl;
  }
  else
  {
    auto typeInfo      = astContext.getTypeInfo(qt);
    member.sizeInBytes = typeInfo.Width / 8; 
  }

  return member;
}


bool kslicer::InitialPassRecursiveASTVisitor::VisitFieldDecl(FieldDecl* fd)
{
  const clang::RecordDecl* rd = fd->getParent();
  const clang::QualType    qt = fd->getType();
 
  const std::string& thisTypeName = rd->getName().str();

  if(thisTypeName == MAIN_CLASS_NAME)
  {
    std::cout << "  found data member: " << fd->getName().str().c_str() << " of type\t" << qt.getAsString().c_str() << ", isPOD = " << qt.isCXX11PODType(m_astContext) << std::endl;

    auto funcSourceRange = rd->getSourceRange();
    auto fileName        = m_sourceManager.getFilename(funcSourceRange.getBegin());
    this->MAIN_FILE_INCLUDE = fileName;

    DataMemberInfo member = ExtractMemberInfo(fd, m_astContext);
    if(member.isPointer) // we ignore pointers due to we can't pass them to GPU correctly
      return true;

    dataMembers[member.name] = member;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool kslicer::InitialPassASTConsumer::HandleTopLevelDecl(DeclGroupRef d)
{
  typedef DeclGroupRef::iterator iter;
  for (iter b = d.begin(), e = d.end(); b != e; ++b)
    rv.TraverseDecl(*b);
  return true; // keep going
}
