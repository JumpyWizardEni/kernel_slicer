#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <vector>
#include <system_error>
#include <iostream>
#include <fstream>

#include <unordered_map>

#include "llvm/Support/Host.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/IntrusiveRefCntPtr.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/FileSystem.h"

#include "clang/Basic/DiagnosticOptions.h"
#include "clang/Frontend/TextDiagnosticPrinter.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Basic/TargetOptions.h"
#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/FileManager.h"
#include "clang/Basic/SourceManager.h"
#include "clang/Lex/Preprocessor.h"
#include "clang/Lex/Lexer.h"
#include "clang/Lex/PreprocessorOptions.h"
#include "clang/Basic/Diagnostic.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/DeclTemplate.h"
#include "clang/Parse/ParseAST.h"
#include "clang/Rewrite/Frontend/Rewriters.h"
#include "clang/Rewrite/Core/Rewriter.h"
#include "clang/Tooling/CommonOptionsParser.h"

#include "kslicer.h"
#include "initial_pass.h"
#include "ast_matchers.h"
#include "class_gen.h"

using namespace clang;

const std::string kslicer::GetProjPrefix() { return std::string("kgen_"); };

using kslicer::KernelInfo;
using kslicer::DataMemberInfo;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static llvm::cl::OptionCategory GDOpts("global-detect options");
clang::LangOptions lopt;

std::string GetKernelSourceCode(const clang::CXXMethodDecl* node, clang::SourceManager& sm, const std::vector<std::string>& threadIdNames) 
{
  clang::SourceLocation b(node->getBeginLoc()), _e(node->getEndLoc());
  clang::SourceLocation e(clang::Lexer::getLocForEndOfToken(_e, 0, sm, lopt));
  std::string methodSource = std::string(sm.getCharacterData(b), sm.getCharacterData(e));
  
  const std::string numThreadsName = kslicer::GetProjPrefix() + "iNumElements";

  std::stringstream strOut;
  strOut << "{" << std::endl;
  strOut << "  /////////////////////////////////////////////////" << std::endl;
  for(size_t i=0;i<threadIdNames.size();i++)
    strOut << "  const uint " << threadIdNames[i].c_str() << " = get_global_id(" << i << ");"<< std::endl;
  strOut << "  if (tid >= " << numThreadsName.c_str() << ")" << std::endl;
  strOut << "    return;" << std::endl;
  strOut << "  /////////////////////////////////////////////////" << std::endl;
  return strOut.str() + methodSource.substr(methodSource.find_first_of('{')+1);
}

std::string GetRangeSourceCode(const clang::SourceRange a_range, clang::SourceManager& sm) 
{
  clang::SourceLocation b(a_range.getBegin()), _e(a_range.getEnd());
  clang::SourceLocation e(clang::Lexer::getLocForEndOfToken(_e, 0, sm, lopt));
  return std::string(sm.getCharacterData(b), sm.getCharacterData(e));
}

void ReplaceOpenCLBuiltInTypes(std::string& a_typeName)
{
  std::string lmStucts("struct LiteMath::");
  auto found1 = a_typeName.find(lmStucts);
  if(found1 != std::string::npos)
    a_typeName.replace(found1, lmStucts.length(), "");

  std::string lm("LiteMath::");
  auto found2 = a_typeName.find(lm);
  if(found2 != std::string::npos)
    a_typeName.replace(found2, lm.length(), "");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::unordered_map<std::string, std::string> ReadCommandLineParams(int argc, const char** argv, std::string& fileName)
{
  std::unordered_map<std::string, std::string> cmdLineParams;
  for(int i=0; i<argc; i++)
  {
    std::string key(argv[i]);
    if(key.size() > 0 && key[0]=='-')
    {
      if(i != argc-1) // not last argument
      {
        cmdLineParams[key] = argv[i+1];
        i++;
      }
      else
        cmdLineParams[key] = "";
    }
    else if(key.find(".cpp") != std::string::npos)
      fileName = key;
  }
  return cmdLineParams;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PrintKernelToCL(std::ostream& outFileCL, const KernelInfo& funcInfo, const std::string& kernName, clang::SourceManager& sm)
{
  assert(funcInfo.astNode != nullptr);

  bool foundThreadIdX = false; std::string tidXName = "tid";
  bool foundThreadIdY = false; std::string tidYName = "tid2";
  bool foundThreadIdZ = false; std::string tidZName = "tid3";

  outFileCL << std::endl;
  outFileCL << "__kernel void " << kernName.c_str() << "(" << std::endl;
  for (const auto& arg : funcInfo.args) 
  {
    std::string typeStr = arg.type.c_str();
    ReplaceOpenCLBuiltInTypes(typeStr);

    bool skip = false;

    if(arg.name == "tid" || arg.name == "tidX") // todo: check several names ... 
    {
      skip           = true;
      foundThreadIdX = true;
      tidXName       = arg.name;
    }

    if(arg.name == "tidY") // todo: check several names ... 
    {
      skip           = true;
      foundThreadIdY = true;
      tidYName       = arg.name;
    }

    if(arg.name == "tidZ") // todo: check several names ... 
    {
      skip           = true;
      foundThreadIdZ = true;
      tidZName       = arg.name;
    }
    
    if(!skip)
      outFileCL << "  __global " << typeStr.c_str() << " restrict " << arg.name.c_str() << "," << std::endl;
  }
  
  const std::string numThreadsName = kslicer::GetProjPrefix() + "iNumElements";
  const std::string m_dataName     = kslicer::GetProjPrefix() + "data";

  outFileCL << "  __global const uint* restrict " << m_dataName.c_str() << "," << std::endl;
  outFileCL << "  const uint " << numThreadsName.c_str() << ")" << std::endl;

  std::vector<std::string> threadIdNames;

  if(foundThreadIdX)
    threadIdNames.push_back(tidXName);

  if(foundThreadIdY)
    threadIdNames.push_back(tidYName);

  if(foundThreadIdZ)
    threadIdNames.push_back(tidZName);

  std::string sourceCode = GetKernelSourceCode(funcInfo.astNode, sm, threadIdNames);
  outFileCL << sourceCode.c_str() << std::endl << std::endl;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class HeaderLister : public clang::PPCallbacks 
{
public:

  HeaderLister(kslicer::MainClassInfo* a_pInfo) : m_pGlobInfo(a_pInfo) {}

  void InclusionDirective(clang::SourceLocation HashLoc,
                          const clang::Token &IncludeTok,
                          llvm::StringRef FileName, bool IsAngled,
                          clang::CharSourceRange FilenameRange,
                          const clang::FileEntry *File,
                          llvm::StringRef SearchPath,
                          llvm::StringRef RelativePath,
                          const clang::Module *Imported,
                          clang::SrcMgr::CharacteristicKind FileType) override
  {
    if(!IsAngled)
    {
      assert(File != nullptr);
      std::string filename = std::string(RelativePath.begin(), RelativePath.end()); 
      m_pGlobInfo->allIncludeFiles[filename] = false;   
    }
  }

private:

  kslicer::MainClassInfo* m_pGlobInfo;

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, const char **argv)
{
  struct stat sb;

  if (argc < 2)
  {
    llvm::errs() << "Usage: <filename>\n";
    return 1;
  }
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string fileName;
  auto params = ReadCommandLineParams(argc, argv, fileName);

  std::string mainClassName = "TestClass";
  std::string mainFuncName  = "PathTrace";
  std::string outGenerated  = "data/generated.cl";
  std::string stdlibFolder  = "";
  
  if(params.find("-mainClass") != params.end())
    mainClassName = params["-mainClass"];

  if(params.find("-mainFunc") != params.end())
    mainFuncName = params["-mainFunc"];

  if(params.find("-out") != params.end())
    outGenerated = params["-out"];

  if(params.find("-stdlibfolder") != params.end())
    stdlibFolder = params["-stdlibfolder"];

  llvm::ArrayRef<const char*> args(argv+1, argv+argc);

  // Make sure it exists
  if (stat(fileName.c_str(), &sb) == -1)
  {
    perror(fileName.c_str());
    exit(EXIT_FAILURE);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  kslicer::MainClassInfo inputCodeInfo;

  CompilerInstance compiler;
  DiagnosticOptions diagnosticOptions;
  compiler.createDiagnostics();  //compiler.createDiagnostics(argc, argv);

  // Create an invocation that passes any flags to preprocessor
  std::shared_ptr<CompilerInvocation> Invocation = std::make_shared<CompilerInvocation>();
  CompilerInvocation::CreateFromArgs(*Invocation, args, compiler.getDiagnostics());
  compiler.setInvocation(Invocation);

  // Set default target triple
  std::shared_ptr<clang::TargetOptions> pto = std::make_shared<clang::TargetOptions>();
  pto->Triple     = llvm::sys::getDefaultTargetTriple();
  TargetInfo *pti = TargetInfo::CreateTargetInfo(compiler.getDiagnostics(), pto);
  compiler.setTarget(pti);
  compiler.createFileManager();
  compiler.createSourceManager(compiler.getFileManager());
  
  // (0) add path dummy include files for STL and e.t.c. (we don't want to parse actually std library)
  //
  HeaderSearchOptions &headerSearchOptions = compiler.getHeaderSearchOpts();  
  headerSearchOptions.AddPath(stdlibFolder.c_str(), clang::frontend::Angled, false, false);

  // Allow C++ code to get rewritten
  LangOptions langOpts;
  langOpts.GNUMode = 1; 
  //langOpts.CXXExceptions = 1; 
  langOpts.RTTI        = 1; 
  langOpts.Bool        = 1; 
  langOpts.CPlusPlus   = 1; 
  langOpts.CPlusPlus14 = 1;
  langOpts.CPlusPlus17 = 1;
  compiler.createPreprocessor(clang::TU_Complete);
  compiler.getPreprocessorOpts().UsePredefines = false;
  compiler.getLangOpts() = langOpts;
  compiler.createASTContext();

  const FileEntry *pFile = compiler.getFileManager().getFile(fileName).get();
  compiler.getSourceManager().setMainFileID( compiler.getSourceManager().createFileID( pFile, clang::SourceLocation(), clang::SrcMgr::C_User));
  compiler.getDiagnosticClient().BeginSourceFile(compiler.getLangOpts(), &compiler.getPreprocessor());
  
  // register our header lister
  {
    auto pHeaderLister = std::make_unique<HeaderLister>(&inputCodeInfo);
    compiler.getPreprocessor().addPPCallbacks(std::move(pHeaderLister));
  }


  ////////////////////////////////////////////////////////////////////////
  std::string outName(fileName);
  {
    size_t ext = outName.rfind(".");
    if (ext == std::string::npos)
       ext = outName.length();
    outName.insert(ext, "_out");
  }
  llvm::errs() << "Output to: " << outName << "\n";
  ////////////////////////////////////////////////////////////////////////

  // (1) traverse source code of main file first
  //
  {
    kslicer::InitialPassASTConsumer astConsumer(mainFuncName.c_str(), mainClassName.c_str(), compiler.getASTContext());  
    ParseAST(compiler.getPreprocessor(), &astConsumer, compiler.getASTContext());
    compiler.getDiagnosticClient().EndSourceFile();
  
    inputCodeInfo.allKernels     = astConsumer.rv.functions;
    inputCodeInfo.allDataMembers = astConsumer.rv.dataMembers;
    inputCodeInfo.mainFuncNode   = astConsumer.rv.m_mainFuncNode;
    inputCodeInfo.mainClassName  = mainClassName;
    inputCodeInfo.mainClassFileName = fileName;
  }
  
  // init clang tooling
  //
  const char* argv2[] = {argv[0], argv[1], "--"};
  int argc2 = sizeof(argv2)/sizeof(argv2[0]);

  clang::tooling::CommonOptionsParser OptionsParser(argc2, argv2, GDOpts);
  clang::tooling::ClangTool Tool(OptionsParser.getCompilations(), OptionsParser.getSourcePathList());

  // (2) now process variables and kernel calls of main function
  //
  {
    clang::ast_matchers::StatementMatcher local_var_matcher = kslicer::MakeMatch_LocalVarOfMethod(mainFuncName.c_str());
    clang::ast_matchers::StatementMatcher kernel_matcher    = kslicer::MakeMatch_MethodCallFromMethod(mainFuncName.c_str());
    
    kslicer::MainFuncAnalyzer printer(std::cout, inputCodeInfo);
    clang::ast_matchers::MatchFinder finder;
    
    finder.addMatcher(local_var_matcher, &printer);
    finder.addMatcher(kernel_matcher,    &printer);
   
    auto res = Tool.run(clang::tooling::newFrontendActionFactory(&finder).get());
    std::cout << "tool run res = " << res << std::endl;
    
    // filter out unused kernels
    //
    inputCodeInfo.kernels.reserve(inputCodeInfo.allKernels.size());
    for (const auto& k : inputCodeInfo.allKernels)
      if(k.second.usedInMainFunc)
        inputCodeInfo.kernels.push_back(k.second);
  }

  // (3) now mark all data members, methods and functions which are actually used in kernels; we will ignore others. 
  //
  kslicer::VariableAndFunctionFilter filter(std::cout, inputCodeInfo, compiler.getSourceManager());
  { 
    for(const auto& kernel : inputCodeInfo.kernels)
    {
      clang::ast_matchers::StatementMatcher dataMemberMatcher = kslicer::MakeMatch_MemberVarOfMethod(kernel.name);
      clang::ast_matchers::StatementMatcher funcMatcher       = kslicer::MakeMatch_FunctionCallFromFunction(kernel.name);
  
      clang::ast_matchers::MatchFinder finder;
      finder.addMatcher(dataMemberMatcher, &filter);
      finder.addMatcher(funcMatcher, &filter);
    
      auto res = Tool.run(clang::tooling::newFrontendActionFactory(&finder).get());
      std::cout << "[filter] for " << kernel.name.c_str() << ";\ttool run res = " << res << std::endl;
    }

    //inputCodeInfo.localFunctions = filter.GetUsedFunctions();
  }

  // (4) calc offsets for all class variables; ingore unused members that were not marked on previous step
  //
  {
    inputCodeInfo.localVariables = kslicer::MakeClassDataListAndCalcOffsets(inputCodeInfo.allDataMembers);
    std::cout << "placed classVariables num = " << inputCodeInfo.localVariables.size() << std::endl;
  }

  // (5) traverse only main function and rename kernel_ to cmd_
  {
    std::string mainFuncCode = kslicer::ProcessMainFunc(inputCodeInfo.mainFuncNode, compiler);
    std::ofstream fout(outName);
    fout << mainFuncCode.c_str() << std::endl;
  }

  std::ofstream outFileCL(outGenerated.c_str());
  if(!outFileCL.is_open())
    llvm::errs() << "Cannot open " << outGenerated.c_str() << " for writing\n";

  // list include files
  //
  for(auto keyVal : inputCodeInfo.allIncludeFiles)
  {
    std::cout << "[include]: " << keyVal.first.c_str() << " = " << keyVal.second << std::endl;
  }

  outFileCL << "/////////////////////////////////////////////////////////////////////" << std::endl;
  outFileCL << "/////////////////// local functions /////////////////////////////////" << std::endl;
  outFileCL << "/////////////////////////////////////////////////////////////////////" << std::endl;
  outFileCL << std::endl;

  // (6) write local functions to .cl file
  //
  for (const auto& f : filter.usedFunctions)  
  {
    std::string funcSourceCode = GetRangeSourceCode(f.second, compiler.getSourceManager());
    outFileCL << funcSourceCode.c_str() << std::endl;
  }

  outFileCL << std::endl;
  outFileCL << "/////////////////////////////////////////////////////////////////////" << std::endl;
  outFileCL << "/////////////////// kernels /////////////////////////////////////////" << std::endl;
  outFileCL << "/////////////////////////////////////////////////////////////////////" << std::endl;
  outFileCL << std::endl;

  // (7) write kernels to .cl file
  //
  {
    for (const auto& k : inputCodeInfo.kernels)  
    {
      std::cout << k.name << " " << k.return_type << std::endl;
      PrintKernelToCL(outFileCL, k, k.name, compiler.getSourceManager());
    }

    outFileCL.close();
  }


  // at this step we must filter data variables to store only those which are referenced inside kernels calls
  //



  return 0;
}