#include "ast_matchers.h"

namespace kslicer
{
  std::string last_fname = "";
  uint32_t last_lineno = 0xFFFFFFF;
}

std::string kslicer::locationAsString(clang::SourceLocation loc, clang::SourceManager const * const sm)
{
  std::stringstream s;
  if(!sm) {
    s << "Invalid SourceManager, cannot dump Location\n";
    return s.str();
  }
  clang::SourceLocation SpellingLoc = sm->getSpellingLoc(loc);
  clang::PresumedLoc ploc = sm->getPresumedLoc(SpellingLoc);
  if(ploc.isInvalid()) {
    s << "<invalid sloc>";
    return s.str();
  }

  std::string fname = ploc.getFilename();
  uint32_t const lineno = ploc.getLine();
  uint32_t const colno = ploc.getColumn();
  if(fname != last_fname) {
    s << fname << ':' << lineno << ':' << colno;
    last_fname = fname;
    last_lineno = lineno;
  }
  else if(lineno != last_lineno) {
    s << "line" << ':' << lineno << ':' << colno;
    last_lineno = lineno;
  }
  else {
    s << "col" << ':' << colno;
  }
  return s.str();
}  // locationAsString

std::string kslicer::sourceRangeAsString(clang::SourceRange r, clang::SourceManager const * sm)
{
  if(!sm) 
    return "";
  std::stringstream s;
  s << "<" << locationAsString(r.getBegin(), sm);
  if(r.getBegin() != r.getEnd())
    s << ", " << locationAsString(r.getEnd(), sm);
  
  s << ">";
  return s.str();
}  // sourceRangeAsString


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

clang::ast_matchers::StatementMatcher kslicer::MakeMatch_LocalVarOfMethod(std::string const& a_funcName)
{
  using namespace clang::ast_matchers;
  return
  declRefExpr(
    to(varDecl(hasLocalStorage()).bind("locVarName")),
       hasAncestor(cxxMethodDecl(hasName(a_funcName)).bind("targetFunction")
    )
  ).bind("localReference");
}

clang::ast_matchers::StatementMatcher kslicer::MakeMatch_MethodCallFromMethod(std::string const& a_funcName)
{
  using namespace clang::ast_matchers;
  return 
  cxxMemberCallExpr(
    allOf(hasAncestor( cxxMethodDecl(hasName(a_funcName)).bind("targetFunction") ),
          callee(cxxMethodDecl().bind("fdecl"))
         )
  ).bind("functionCall");
}

clang::ast_matchers::StatementMatcher kslicer::MakeMatch_MethodCallFromMethod()
{
  using namespace clang::ast_matchers;
  return 
  cxxMemberCallExpr(
    allOf(hasAncestor( cxxMethodDecl().bind("targetFunction") ),
          callee(cxxMethodDecl().bind("fdecl"))
         )
  ).bind("functionCall");
}

clang::ast_matchers::StatementMatcher kslicer::MakeMatch_FunctionCallFromFunction(std::string const& a_funcName)
{
  using namespace clang::ast_matchers;
  return 
  callExpr(
    allOf(hasAncestor( cxxMethodDecl(hasName(a_funcName)).bind("targetFunction") ),
          callee(functionDecl().bind("fdecl"))
         )
  ).bind("functionCall");
}

clang::ast_matchers::StatementMatcher kslicer::MakeMatch_MemberVarOfMethod(std::string const& a_funcName)
{
  using namespace clang::ast_matchers;
  return
  memberExpr(
    hasDeclaration(fieldDecl().bind("memberName")),
    hasAncestor(functionDecl(hasName(a_funcName)).bind("targetFunction"))
  ).bind("memberReference");
}

clang::ast_matchers::StatementMatcher kslicer::MakeMatch_SingleForLoopInsideFunction(std::string const& a_funcName)
{
  using namespace clang::ast_matchers;
  return forStmt(hasLoopInit(declStmt(hasSingleDecl(varDecl().bind("loopIter")))),
                 hasAncestor(functionDecl(hasName(a_funcName)).bind("targetFunction"))
         ).bind("forLoop");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class MainFuncSeeker : public clang::ast_matchers::MatchFinder::MatchCallback 
{
public:
  explicit MainFuncSeeker(std::ostream& s, const std::string& a_mainClassName, const clang::ASTContext& a_astContext) : 
                            m_out(s), m_mainClassName(a_mainClassName), m_astContext(a_astContext) 
  {
  }

  void run(clang::ast_matchers::MatchFinder::MatchResult const & result) override
  {
    using namespace clang;
   
    CXXMethodDecl     const * func_decl = result.Nodes.getNodeAs<CXXMethodDecl>    ("targetFunction");
    CXXMemberCallExpr const * kern_call = result.Nodes.getNodeAs<CXXMemberCallExpr>("functionCall");
    CXXMethodDecl     const * kern      = result.Nodes.getNodeAs<CXXMethodDecl>    ("fdecl");

    if(func_decl && kern_call && kern) 
    {
      const auto pClass = func_decl->getParent();
      assert(pClass != nullptr);
      if(pClass->getName().str() == m_mainClassName &&  kern->getNameAsString().find("kernel_") != std::string::npos)
      {
        //std::cout << func_decl->getNameAsString() << " --> " << kern->getNameAsString() << std::endl;
        auto p = m_mainFunctions.find(func_decl->getNameAsString());
        if(p == m_mainFunctions.end())
        {
          kslicer::MainFuncNameInfo info;
          info.name = func_decl->getNameAsString();
          info.kernelNames.push_back(kern->getNameAsString());
          m_mainFunctions[func_decl->getNameAsString()] = info;
        }
        else
        {
          auto& kernNames = p->second.kernelNames;
          auto elementId = std::find(kernNames.begin(), kernNames.end(), kern->getNameAsString());
          if(elementId == kernNames.end())
            kernNames.push_back(kern->getNameAsString());
        }
        
      }
    }
    else 
    {
      kslicer::check_ptr(func_decl, "func_decl", "", m_out);
      kslicer::check_ptr(kern_call, "kern_call", "", m_out);
      kslicer::check_ptr(kern,      "kern",      "", m_out);
    }
    return;
  }  // run
  
  std::ostream&            m_out;
  const std::string&       m_mainClassName;
  const clang::ASTContext& m_astContext;
  std::unordered_map<std::string, kslicer::MainFuncNameInfo> m_mainFunctions;
}; 

std::unordered_map<std::string, kslicer::MainFuncNameInfo> kslicer::ListAllMainRTFunctions(clang::tooling::ClangTool& Tool, 
                                                                                           const std::string& a_mainClassName, 
                                                                                           const clang::ASTContext& a_astContext)
{
  auto kernelCallMatcher = kslicer::MakeMatch_MethodCallFromMethod();
  
  MainFuncSeeker printer(std::cout, a_mainClassName, a_astContext);
  clang::ast_matchers::MatchFinder finder;
  finder.addMatcher(kernelCallMatcher,  &printer);

  auto res = Tool.run(clang::tooling::newFrontendActionFactory(&finder).get());
  if(res != 0) 
    std::cout << "[Seeking for MainFunc]: tool run res = " << res << std::endl;

  return printer.m_mainFunctions;
}