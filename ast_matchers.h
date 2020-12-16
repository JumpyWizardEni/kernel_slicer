#ifndef KSLICER_AST_MATCHERS
#define KSLICER_AST_MATCHERS

#include "clang/Tooling/CommonOptionsParser.h"
#include "clang/Tooling/Tooling.h"
#include "clang/ASTMatchers/ASTMatchFinder.h"

#include "clang/Basic/SourceManager.h"
#include "clang/AST/ASTContext.h"

#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include <sstream>
#include <iostream>

#include "kslicer.h"

namespace kslicer
{
  using namespace clang::tooling;
  using namespace llvm;
  using namespace clang::ast_matchers;

  clang::ast_matchers::StatementMatcher MakeMatch_LocalVarOfMethod(std::string const& funcName); 
  clang::ast_matchers::StatementMatcher MakeMatch_MethodCallFromMethod(std::string const& funcName); // from method with name 'funcName'
  clang::ast_matchers::StatementMatcher MakeMatch_MethodCallFromMethod();                            // from any method

  clang::ast_matchers::StatementMatcher MakeMatch_MemberVarOfMethod(std::string const& funcName);     
  clang::ast_matchers::StatementMatcher MakeMatch_FunctionCallFromFunction(std::string const& funcName);
  clang::ast_matchers::StatementMatcher MakeMatch_SingleForLoopInsideFunction(std::string const& funcName);
  clang::ast_matchers::StatementMatcher MakeMatch_IfInsideForLoopInsideFunction(std::string const& funcName);

  std::string locationAsString(clang::SourceLocation loc, clang::SourceManager const * const sm);
  std::string sourceRangeAsString(clang::SourceRange r, clang::SourceManager const * sm);

  std::unordered_map<std::string, MainFuncNameInfo> ListAllMainRTFunctions(clang::tooling::ClangTool& Tool, 
                                                                           const std::string& a_mainClassName, 
                                                                           const clang::ASTContext& a_astContext);

  /**\brief Complain if pointer is invalid.
  \param p: pointer
  \param name: name of thing checked
  \param tabs: indentation
  \return none */
  template <typename T>
  inline void
  check_ptr(T * p,
            const std::string& name,
            const std::string& tabs = "",
            std::ostream&         s = std::cout)
  {
    if(!p) { s << tabs << "Invalid pointer " << name << "\n"; }
  }

  class MainFuncAnalyzer : public clang::ast_matchers::MatchFinder::MatchCallback 
  {
  public:

    explicit MainFuncAnalyzer(std::ostream& s, kslicer::MainClassInfo& a_allInfo, const clang::ASTContext& a_astContext, size_t a_mainFuncId) : 
                              m_out(s), m_allInfo(a_allInfo), m_astContext(a_astContext), m_mainFuncId(a_mainFuncId) 
    {
      m_namesToIngore = kslicer::GetAllPredefinedThreadIdNames(); 
    }

    MainFuncInfo& CurrMainFunc() { return m_allInfo.mainFunc[m_mainFuncId]; }

    void run(clang::ast_matchers::MatchFinder::MatchResult const & result) override
    {
      using namespace clang;
      
      // for kernel call inside MainFunc
      //
      FunctionDecl      const * func_decl = result.Nodes.getNodeAs<FunctionDecl>     ("targetFunction");
      CXXMemberCallExpr const * kern_call = result.Nodes.getNodeAs<CXXMemberCallExpr>("functionCall");
      CXXMethodDecl     const * kern      = result.Nodes.getNodeAs<CXXMethodDecl>    ("fdecl");
      
      // for local variable decl inside MainFunc
      //
      Expr    const * l_var = result.Nodes.getNodeAs<Expr>   ("localReference");
      VarDecl const * var   = result.Nodes.getNodeAs<VarDecl>("locVarName");

      // for for expression inside MainFunc
      //
      ForStmt const * forLoop = result.Nodes.getNodeAs<ForStmt>("forLoop");
      VarDecl const * forIter = result.Nodes.getNodeAs<VarDecl>("loopIter"); 

      // for if statements inside for loop, which are goint to break loop
      //
      IfStmt     const * ifCond = result.Nodes.getNodeAs<IfStmt>("ifCond"); 
      BreakStmt  const * brkExp = result.Nodes.getNodeAs<BreakStmt>("breakLoop");
      ReturnStmt const * extExp = result.Nodes.getNodeAs<ReturnStmt>("exitFunction"); 

      if(func_decl && kern_call && kern) // found kernel call in MainFunc
      {
        auto pKernel = m_allInfo.allKernels.find(kern->getNameAsString());  
        if(pKernel != m_allInfo.allKernels.end()) 
          pKernel->second.usedInMainFunc = true;                    // mark this kernel is used
        CurrMainFunc().UsedKernels.insert(kern->getNameAsString()); // add  this kernel to list of used kernels by MainFunc 
      }
      else if(func_decl && l_var && var) // found local variable in MainFunc
      {
        const clang::QualType qt = var->getType();
        const auto typePtr = qt.getTypePtr(); 
        assert(typePtr != nullptr);
        
        auto elementId = std::find(m_namesToIngore.begin(), m_namesToIngore.end(), var->getNameAsString());

        if(typePtr->isPointerType() || elementId != m_namesToIngore.end()) // ignore pointers and variables with special names
          return;

        auto typeInfo = m_astContext.getTypeInfo(qt);

        DataLocalVarInfo varInfo;
        varInfo.name        = var->getNameAsString();
        varInfo.type        = qt.getAsString();
        varInfo.sizeInBytes = typeInfo.Width / 8;
        
        varInfo.isArray   = false;
        varInfo.arraySize = 0;
        varInfo.typeOfArrayElement = ""; 
        
        if(typePtr->isConstantArrayType())
        {
          auto arrayType = dyn_cast<ConstantArrayType>(typePtr); 
          assert(arrayType != nullptr);

          QualType qtOfElem = arrayType->getElementType(); 
          auto typeInfo2 = m_astContext.getTypeInfo(qtOfElem);

          varInfo.arraySize = arrayType->getSize().getLimitedValue();      
          varInfo.typeOfArrayElement = qtOfElem.getAsString();
          varInfo.sizeInBytesOfArrayElement = typeInfo2.Width / 8;
          varInfo.isArray = true;
        }

        if(var->isLocalVarDecl()) // list only local variables, exclude function arguments 
        {
          // can also check isCXXForRangeDecl() and check for index ... in some way ...
          CurrMainFunc().Locals[varInfo.name] = varInfo;
        }
      }
      else if(func_decl && forLoop && forIter) // found for expression inside MainFunc
      {
        CurrMainFunc().ExcludeList.insert(forIter->getNameAsString());
      }
      else if(func_decl && kern_call && ifCond)
      {
        bool hasNegativeCondition = false;
        auto conBody = ifCond->getCond();
        if(isa<UnaryOperator>(conBody))
        {
          const auto bodyOp    = dyn_cast<UnaryOperator>(conBody);
          std::string opStr    = bodyOp->getOpcodeStr(bodyOp->getOpcode());
          hasNegativeCondition = (opStr == "!");
        }

        auto kernDecl = kern_call->getMethodDecl();
        auto pKernel  = m_allInfo.allKernels.find(kernDecl->getNameAsString());  
        if(pKernel != m_allInfo.allKernels.end() && (brkExp || extExp)) 
        {
          pKernel->second.usedInExitExpr = true; // mark this kernel is used in exit expression
          uint64_t hashValue = kslicer::GetHashOfSourceRange(ifCond->getSourceRange());

          kslicer::ExitStatementInfo info;
          info.kernelName      = kernDecl->getNameAsString();
          info.kernelCallRange = kern_call->getSourceRange();
          info.ifExprRange     = ifCond->getSourceRange();
          if(brkExp)
            info.exprKind = kslicer::ExitStmtKind::EXIT_TYPE_LOOP_BREAK;
          else
            info.exprKind = kslicer::ExitStmtKind::EXIT_TYPE_FUNCTION_RETURN;
          info.isNegative = hasNegativeCondition;
          CurrMainFunc().ExitExprIfCond[hashValue] = info; // ExitExprIfCond[ifCond] = kern_call;
        }
      }
      else 
      {
        check_ptr(l_var,     "l_var", "",     m_out);
        check_ptr(var,       "var",   "",     m_out);

        check_ptr(func_decl, "func_decl", "", m_out);
        check_ptr(kern_call, "kern_call", "", m_out);
        check_ptr(kern,      "kern",      "", m_out);
      }

      return;
    }  // run
    
    std::ostream& m_out;
    MainClassInfo& m_allInfo;
    const clang::ASTContext&  m_astContext;

    std::vector<std::string> m_namesToIngore;
    size_t m_mainFuncId = 0;

  };  // class MainFuncAnalyzer


  class VariableAndFunctionFilter : public clang::ast_matchers::MatchFinder::MatchCallback 
  {
  public:

    explicit VariableAndFunctionFilter(std::ostream& s, kslicer::MainClassInfo& a_allInfo, clang::SourceManager& a_sm) : 
                                       m_out(s), m_allInfo(a_allInfo), m_sourceManager(a_sm) { usedFunctions.clear(); }

    kslicer::KernelInfo* currKernel = nullptr;

    void run(clang::ast_matchers::MatchFinder::MatchResult const & result) override
    {
      using namespace clang;
     
      FunctionDecl      const * func_decl = result.Nodes.getNodeAs<FunctionDecl>("targetFunction");
      MemberExpr        const * l_var     = result.Nodes.getNodeAs<MemberExpr>("memberReference");
      FieldDecl         const * var       = result.Nodes.getNodeAs<FieldDecl>("memberName");

      CallExpr const * funcCall = result.Nodes.getNodeAs<CallExpr>("functionCall");
      FunctionDecl const * func = result.Nodes.getNodeAs<FunctionDecl>("fdecl");

      clang::SourceManager& srcMgr(const_cast<clang::SourceManager &>(result.Context->getSourceManager()));

      if(func_decl && l_var && var)
      {
        const RecordDecl* parentClass = var->getParent(); 
        if(parentClass != nullptr && parentClass->getNameAsString() == m_allInfo.mainClassName)
        {
          auto pDataMember = m_allInfo.allDataMembers.find(var->getNameAsString());
          assert(pDataMember != m_allInfo.allDataMembers.end());
          assert(currKernel  != nullptr);

          pDataMember->second.usedInKernel = true;
          if(pDataMember->second.isContainer)
            currKernel->usedVectors.insert(pDataMember->first);
        }
      }
      else if(func_decl && funcCall && func)
      {
        auto funcSourceRange = func->getSourceRange();
        auto fileName  = m_sourceManager.getFilename(funcSourceRange.getBegin());
        if(fileName == m_allInfo.mainClassFileName)
        {
          if(usedFunctions.find(func->getNameAsString()) == usedFunctions.end())
          {
            usedFunctions[func->getNameAsString()] = funcSourceRange;
          }
        }
        
        usedFiles[srcMgr.getFilename(func->getLocation()).str()] = true; // mark include files that used by functions; we need to put such includes in .cl file
      }
      else 
      {
        check_ptr(l_var,     "l_var", "", m_out);
        check_ptr(var,       "var",   "", m_out);

        check_ptr(func_decl, "func_decl", "", m_out);
        check_ptr(funcCall,  "kern_call", "", m_out);
        check_ptr(func,      "kern",      "", m_out);
      }

      return;
    }  // run
    
    std::ostream&  m_out;
    MainClassInfo& m_allInfo;
    std::string    m_mainClassName;
    clang::SourceManager& m_sourceManager;

    std::unordered_map<std::string, clang::SourceRange> usedFunctions;
    std::unordered_map<std::string, bool>               usedFiles;

    //std::vector<const clang::FunctionDecl*> GetUsedFunctions()
    //{
    //  std::vector<const clang::FunctionDecl*> res;
    //  for(auto fDecl : usedFunctions)
    //    res.push_back((const clang::FunctionDecl*)fDecl.second);
    //  return res;
    //}

  };  // class Global_Printer

}

#endif
