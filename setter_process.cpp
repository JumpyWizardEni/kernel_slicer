#include <stdio.h>
#include <vector>
#include <system_error>
#include <iostream>
#include <fstream>
#include <sstream>

#include <unordered_map>
#include <iomanip>

#include "kslicer.h"


class SetterVisitor : public clang::RecursiveASTVisitor<SetterVisitor>
{
public:
  
  SetterVisitor(const clang::CompilerInstance& a_compiler, clang::Rewriter &R, const std::unordered_map<std::string, const clang::CXXRecordDecl*>& a_types) : 
                m_compiler(a_compiler), m_sm(a_compiler.getSourceManager()), m_rewriter(R), m_targetTypes(a_types)
  { 
    
  }

  bool VisitMemberExpr(const clang::MemberExpr* expr) 
  {
    const clang::ValueDecl* pDecl =	expr->getMemberDecl();
    const clang::QualType qt      = pDecl->getType();
    const std::string typeName    = qt.getAsString(); 

    auto pFound = m_targetTypes.find(typeName);
    if(pFound != m_targetTypes.end())
      m_foundMembers[pDecl->getNameAsString()] = typeName;

    std::string text = kslicer::GetRangeSourceCode(expr->getSourceRange(), m_compiler);
    m_rewriter.ReplaceText(expr->getSourceRange(), text + "Vulkan");
    return true;
  }

  const std::unordered_map<std::string, std::string>& GetMembers() const { return m_foundMembers; }

private:
  const clang::CompilerInstance& m_compiler;
  const clang::SourceManager&    m_sm;
  clang::Rewriter&               m_rewriter;

  const std::unordered_map<std::string, const clang::CXXRecordDecl*>& m_targetTypes;
  std::unordered_map<std::string, std::string>                        m_foundMembers; ///<! member name --> member type name
};


std::unordered_map<std::string, const clang::CXXRecordDecl*> ListStructParamTypes(const clang::CXXMethodDecl* node)
{
  std::unordered_map<std::string, const clang::CXXRecordDecl*> structTypeNames; 
  for(unsigned paramId = 0; paramId < node->getNumParams(); paramId++)
  {
    const clang::ParmVarDecl* pParam  = node->getParamDecl(paramId);
    const clang::QualType typeOfParam = pParam->getType();
    const clang::CXXRecordDecl* pDecl = typeOfParam->getAsCXXRecordDecl();
    if(pDecl == nullptr)
      continue;
    structTypeNames[typeOfParam.getAsString()] = pDecl;
  }
  return structTypeNames;
}

std::unordered_map<std::string, kslicer::SetterStruct> kslicer::ProcessAllSetters(const std::unordered_map<std::string, const clang::CXXMethodDecl*>& a_setterFunc, clang::CompilerInstance& a_compiler,
                                                                                  std::vector<std::string>& a_rewrittenDecls,
                                                                                  std::vector<std::string>& a_rewrittenFuncs,
                                                                                  std::unordered_map<std::string, std::string>& a_variables)
{
  std::unordered_map<std::string, kslicer::SetterStruct> res;

  a_rewrittenDecls.clear();
  a_rewrittenFuncs.clear();
  a_variables.clear();

  for(const auto kv : a_setterFunc)
  {
    const clang::CXXMethodDecl* node = kv.second;
    auto structTypeNames    = ListStructParamTypes(node);
    const std::string fname = node->getNameInfo().getName().getAsString();

    // (1) traverse type decl, rewrite (pointer, texture, accel_struct) members 
    //
    for(const auto& kv : structTypeNames)
    {
      const clang::CXXRecordDecl* pDecl = kv.second;
      std::stringstream strOut;
      strOut << kv.first.c_str() << "Vulkan" << "{" << std::endl;
      
      for(const auto field : pDecl->fields()) //  clang::FieldDecl
      {
        const std::string varName = field->getNameAsString();
        const clang::QualType qt  = field->getType();

        if(qt->isPointerType())
        {
          strOut << "  VkBuffer " << varName.c_str() << "Buffer = VK_NULL_HANDLE; size_t " << varName.c_str() << "Offset = 0;" << std::endl;
        }
        else if(qt->isReferenceType() && kslicer::IsTexture(qt))
        {
          strOut << "  VkImage " << varName.c_str() << "Image = VK_NULL_HANDLE; VkImageView " << varName.c_str() << "View = VK_NULL_HANDLE;"  << std::endl;
        }
        else
        {
          strOut << "  " << qt.getAsString() << " " << varName.c_str() << ";" << std::endl;
        }
      }
      strOut << "};" << std::endl;
      a_rewrittenDecls.push_back(strOut.str());
    }

    // (2) traverse setter function node, rename all structure members and parameters of any type of 'structTypeNames' 
    //
    clang::Rewriter rewrite;
    rewrite.setSourceMgr(a_compiler.getSourceManager(), a_compiler.getLangOpts());

    SetterVisitor visitor(a_compiler, rewrite, structTypeNames);
    visitor.TraverseDecl(const_cast<clang::CXXMethodDecl*>(node));

    std::string rewrittenFunc = rewrite.getRewrittenText(node->getSourceRange());
    std::string rewrittenBody = rewrittenFunc.substr(rewrittenFunc.find("{"));

    std::stringstream strOutFD;
    strOutFD << "void " << fname.c_str() << "Vulkan(";
    for(unsigned paramId = 0; paramId < node->getNumParams(); paramId++)
    {
      const clang::ParmVarDecl* pParam  = node->getParamDecl(paramId);
      const clang::QualType typeOfParam = pParam->getType();
      const clang::CXXRecordDecl* pDecl = typeOfParam->getAsCXXRecordDecl();
      if(pDecl != nullptr)
        strOutFD << typeOfParam.getAsString() << "Vulkan " << pParam->getNameAsString();
      else
        strOutFD << kslicer::GetRangeSourceCode(pParam->getSourceRange(), a_compiler);

      if(paramId < node->getNumParams()-1)
        strOutFD << ", ";
      else
        strOutFD << ")";   
    }
    
    std::string finalFuncText = strOutFD.str() + rewrittenBody;
    a_rewrittenFuncs.push_back(finalFuncText);
    for(const auto kv : visitor.GetMembers())
      a_variables[kv.first] = kv.second;
  }
 
  return res;
}