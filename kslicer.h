#ifndef KSLICER_H
#define KSLICER_H

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <sstream>
#include <inja.hpp>

#include "clang/AST/DeclCXX.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/ASTMatchers/ASTMatchFinder.h"
#include "clang/Tooling/Tooling.h"

#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Rewrite/Frontend/Rewriters.h"
#include "clang/Rewrite/Core/Rewriter.h"

bool ReplaceFirst(std::string& str, const std::string& from, const std::string& to);

namespace kslicer
{
  struct IShaderCompiler;

  enum VKERNEL_IMPL_TYPE { VKERNEL_SWITCH = 0, VKERNEL_INDIRECT_DISPATCH=2 };

  /**
  \brief for each method MainClass::kernel_XXX
  */
  struct KernelInfo 
  {
    struct Arg 
    {
      std::string type;
      std::string name;
      int         size;

      bool needFakeOffset = false;
      bool isThreadID     = false; ///<! used by RTV-like patterns where loop is defined out of kernel
      bool isLoopSize     = false; ///<! used by IPV-like patterns where loop is defined inside kernel
      bool isPointer      = false;
      bool isThreadFlags  = false; 

      bool IsUser() const { return !isThreadID && !isLoopSize && !needFakeOffset && !isPointer; }
    };

    struct LoopIter 
    {
      std::string type;
      std::string name;
      std::string sizeExpr;
      uint32_t    loopNesting = 0;
    };

    struct LoopInitStatement
    {
      clang::SourceRange srcRange;
    };
    
    enum class REDUCTION_TYPE {ADD_ONE = 1, ADD = 2, MUL = 3, FUNC = 4, SUB = 5, SUB_ONE,  UNKNOWN = 255};

    struct ReductionAccess
    {
      REDUCTION_TYPE type;
      std::string    rightExpr;
      std::string    leftExpr; // altered left expression (arrays and other ... )
      bool           leftIsArray = false;
      uint32_t       arraySize   = 0;
      std::string    arrayIndex;
      std::string    arrayName;
      std::vector<std::string> arrayTmpBufferNames;
      
      std::string    funcName;
      std::string    dataType   = "UnknownType";
      std::string    tmpVarName = "UnknownReductionOutput";
      std::string    GetInitialValue()  const;
      std::string    GetOp(std::shared_ptr<IShaderCompiler> pShaderCC) const;

      bool           SupportAtomicLastStep() const;
      std::string    GetAtomicImplCode()     const;
      size_t         GetSizeOfDataType()     const;
    };
    
    std::string           return_type;          ///<! func. return type
    std::string           return_class;         ///<! class name of pointer if pointer is returned
    std::string           name;                 ///<! func. name
    std::string           className;            ///<! Class::kernel_XXX --> 'Class'
    std::string           interfaceName;        ///<! Name of the interface if the kernel is virtual
    std::vector<Arg>      args;                 ///<! all arguments of a kernel
    std::vector<LoopIter> loopIters;            ///<! info about internal loops inside kernel which should be eliminated (so these loops are transformed to kernel call); For IPV pattern.
    
    uint32_t GetDim() const { return uint32_t(loopIters.size());}

    clang::SourceRange    loopInsides;          ///<! used by IPV pattern to extract loops insides and make them kernel source
    clang::SourceRange    loopOutsidesInit;     ///<! used by IPV pattern to extract code before loops and then make additional initialization kernel
    clang::SourceRange    loopOutsidesFinish;   ///<! used by IPV pattern to extract code after  loops and then make additional finalization kernel
    bool                  hasInitPass   = false;///<! used by IPV pattern (currently); indicate that we need insert additional single-threaded run before current kernel (for reduction init or indirect dispatch buffer init)
    bool                  hasFinishPass = false;///<! used by IPV pattern (currently); indicate that we need insert additional passes              after  current kernel
    bool                  hasFinishPassSelf = false; ///<! if we need to do some-thing after loop and after generated loop finish pass

    const clang::CXXMethodDecl* astNode = nullptr;
    bool usedInMainFunc = false;                ///<! wherther kernel is actually used or just declared
    bool isBoolTyped    = false;                ///<! used by RTV pattern; special case: if kernel return boolean, we analyze loop exit (break) or function exit (return) expression
    bool usedInExitExpr = false;                ///<! used by RTV pattern; if kernel is used in Control Function in if(kernelXXX()) --> break or return extression
    bool checkThreadFlags = false;              ///<! used by RTV pattern; if Kernel.shouldCheckExitFlag --> insert check flags code in kernel
    bool isVirtual      = false;                ///<! used by RTV pattern; if kernel is a 'Virtual Kernel'
    bool isMaker        = false;                ///<! used by RTV pattern; if kernel is an object Maker

    std::string RetType;                         ///<! kernel return type
    std::string DeclCmd;                         ///<! used during class header to print declaration of current 'XXXCmd' for current 'kernel_XXX'
    std::unordered_set<std::string> usedVectors; ///<! list of all std::vector<T> member names which is referenced inside kernel
    std::unordered_set<std::string> usedMembers; ///<! list of all other variables used inside kernel
    std::unordered_map<std::string, ReductionAccess> subjectedToReduction; ///<! if member is used in reduction expression

    std::string rewrittenText;                   ///<! rewritten source code of a kernel
    std::string rewrittenInit;                   ///<! rewritten loop initialization code for kernel
    std::string rewrittenFinish;                 ///<! rewritten loop finish         code for kernel

    uint32_t wgSize[3] = {256,1,1};              ///<! workgroup size for the case when setting wgsize with spec constant is not allowed
    uint32_t warpSize  = 32;                     ///<! warp size in which we can rely on to omit sync in reduction and e.t.c.

    bool      isIndirect = false;                ///<! IPV pattern; if loop size is defined by class member variable or vector size, we interpret it as indirect dispatching
    uint32_t  indirectBlockOffset = 0;           ///<! IPV pattern; for such kernels we have to know some offset in indirect buffer for thread blocks number (use int4 data for each kernel)
    uint32_t  indirectMakerOffset = 0;           ///<! RTV pattern; kernel-makers have to update size in the indirect buffer  
  };

  enum class DATA_USAGE{ USAGE_USER = 0, USAGE_SLICER_REDUCTION = 1 };

  /**
  \brief for each data member of MainClass
  */
  struct DataMemberInfo 
  {
    std::string name;
    std::string type;
    size_t      sizeInBytes;              ///<! may be not needed due to using sizeof in generated code, but it is useful for sorting members by size and making apropriate aligment
    size_t      alignedSizeInBytes   = 0; ///<! aligned size will be known when data will be placed to a buffer
    size_t      offsetInTargetBuffer = 0; ///<! offset in bytes in terget buffer that stores all data members
    
    bool isContainerInfo   = false; ///<! auto generated std::vector<T>::size() or capacity() or some analogue
    bool isContainer       = false; ///<! if std::vector<...>
    bool isArray           = false; ///<! if is array, element type stored incontainerDataType;
    bool usedInKernel      = false; ///<! if any kernel use the member --> true; if no one uses --> false;
    bool usedInMainFn      = false; ///<! if std::vector is used in MainFunction like vector.data();
    
    DATA_USAGE  usage = DATA_USAGE::USAGE_USER; ///<! if this is service and 'implicit' data which was agged by generator, not by user;

    size_t      arraySize = 0;     ///<! 'N' if data is declared as 'array[N]';
    std::string containerType;     ///<! std::vector usually
    std::string containerDataType; ///<! data type 'T' inside of std::vector<T>
  };

  /**
  \brief for local variables of MainFunc
  */
  struct DataLocalVarInfo 
  {
    std::string name;
    std::string type;
    size_t      sizeInBytes;

    bool        isArray   = false;
    size_t      arraySize = 0;
    std::string typeOfArrayElement;
    size_t      sizeInBytesOfArrayElement = 0;
  };

  /**
  \brief for arguments of MainFunc which are pointers
  */
  struct InOutVarInfo 
  {
    std::string name;
  };

  // assume there could be only 4 form of kernel arg when kernel is called
  //
  enum class KERN_CALL_ARG_TYPE{
    ARG_REFERENCE_LOCAL         = 0, // Passing the address of a local variable or local array by pointer (for example "& rayPosAndNear" or just randsArray);
    ARG_REFERENCE_ARG           = 1, // Passing the pointer that was supplied to the argument of MainFunc (for example, just "out_color") 
    ARG_REFERENCE_CLASS_VECTOR  = 2, // Passing a pointer to a class member of type std::vector<T>::data() (for example m_materials.data())
    ARG_REFERENCE_CLASS_POD     = 3, // Passing a pointer to a member of the class of type plain old data. For example, "&m_worldViewProjInv"
    ARG_REFERENCE_THREAD_ID     = 4, // Passing tidX, tidY or tidZ
    ARG_REFERENCE_UNKNOWN_TYPE  = 9  // Unknown type of arument yet. Generaly means we need to furthe process it, for example find among class variables or local variables
    };

  struct ArgReferenceOnCall
  {
    KERN_CALL_ARG_TYPE argType = KERN_CALL_ARG_TYPE::ARG_REFERENCE_UNKNOWN_TYPE;
    std::string        varName = "";
    bool umpersanned           = false; // just signal that '&' was applied to this argument, and thus it is likely to be (ARG_REFERENCE_LOCAL or ARG_REFERENCE_CLASS_POD)
  };

  struct KernelCallInfo
  {
    std::string                     kernelName;
    std::string                     originKernelName;
    std::string                     callerName;
    std::vector<ArgReferenceOnCall> descriptorSetsInfo;
    bool isService = false; ///<! indicate that this call is added by the slicer itself. It is not user kernel.
  };

  struct MainFuncNameInfo
  {
    std::string              name;
    std::vector<std::string> kernelNames;
  };

  enum class KernelStmtKind { CALL_TYPE_SIMPLE          = 1,
                              EXIT_TYPE_FUNCTION_RETURN = 2, 
                              EXIT_TYPE_LOOP_BREAK      = 3};

  struct KernelStatementInfo
  {
    std::string        kernelName;
    clang::SourceRange kernelCallRange;
    clang::SourceRange ifExprRange;
    KernelStmtKind     exprKind = KernelStmtKind::CALL_TYPE_SIMPLE;
    bool               isNegative = false;
  };

  struct MainFuncInfo
  {
    std::string                                       Name;
    const clang::CXXMethodDecl*                       Node;
    std::unordered_map<std::string, DataLocalVarInfo> Locals;
    std::vector<InOutVarInfo>                         InOuts;
    std::unordered_set<std::string>                   ExcludeList;
    std::unordered_set<std::string>                   UsedKernels;
    
    std::string GeneratedDecl;
    std::string CodeGenerated;

    size_t startDSNumber = 0;
    size_t endDSNumber   = 0;

    // RT template specific
    //
    std::unordered_map<uint64_t, KernelStatementInfo> ExitExprIfCond;
    std::unordered_map<uint64_t, KernelStatementInfo> ExitExprIfCall;
    std::unordered_map<uint64_t, KernelStatementInfo> CallsInsideFor;

    bool   needToAddThreadFlags = false;
  };
  
  /**
  \brief Both for common functions and member functions called from kernels
  */
  struct FuncData
  {
    const clang::FunctionDecl* astNode;
    std::string        name;
    clang::SourceRange srcRange;
    uint64_t           srcHash;
    bool               isMember = false;
    bool               isKernel = false;
    int                depthUse = 0;    ///!< depth Of Usage; 0 -- for kernels; 1 -- for functions called from kernel; 2 -- for functions called from functions called from kernels
                                        ///!< please note that if function is called under different depth, maximum depth should be stored in this variable;
  };
  
  enum class DECL_IN_CLASS{ DECL_STRUCT, DECL_TYPEDEF, DECL_CONSTANT, DECL_UNKNOWN};

  /**
  \brief declaration of types and constant inside main class
  */
  struct DeclInClass
  {
    std::string        name;
    std::string        type;
    clang::SourceRange srcRange;
    uint64_t           srcHash;
    uint32_t           order = 0; ///<! to sort them before put in generated kernels source code
    DECL_IN_CLASS      kind  = DECL_IN_CLASS::DECL_UNKNOWN;
    bool               extracted = false;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////  FunctionRewriter  ////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  class MainClassInfo;
  void MarkRewrittenRecursive(const clang::Stmt* currNode, std::unordered_set<uint64_t>& a_rewrittenNodes);
  void MarkRewrittenRecursive(const clang::Decl* currNode, std::unordered_set<uint64_t>& a_rewrittenNodes);

  /**
  \brief process local functions (data["LocalFunctions"]), float3 --> make_float3, std::max --> fmax and e.t.c.
  */
  class FunctionRewriter : public clang::RecursiveASTVisitor<FunctionRewriter> // 
  {
  public:
    
    FunctionRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, MainClassInfo* a_codeInfo) : 
                     m_rewriter(R), m_compiler(a_compiler), m_codeInfo(a_codeInfo)
    { 
      
    }

    virtual ~FunctionRewriter(){}

    bool VisitCallExpr(clang::CallExpr* f)                   { return VisitCallExpr_Impl(f); }
    bool VisitCXXConstructExpr(clang::CXXConstructExpr* call);

    bool VisitFunctionDecl(clang::FunctionDecl* fDecl)       { return VisitFunctionDecl_Impl(fDecl); }
    bool VisitCXXMethodDecl(clang::CXXMethodDecl* fDecl)     { return VisitCXXMethodDecl_Impl(fDecl); }

    bool VisitVarDecl(clang::VarDecl* decl)                  { return VisitVarDecl_Impl(decl);        }

    bool VisitMemberExpr(clang::MemberExpr* expr)            { return VisitMemberExpr_Impl(expr);     }
    bool VisitCXXMemberCallExpr(clang::CXXMemberCallExpr* f) { return VisitCXXMemberCallExpr_Impl(f); }
    bool VisitFieldDecl(clang::FieldDecl* decl)              { return VisitFieldDecl_Impl(decl);      }
    bool VisitUnaryOperator(clang::UnaryOperator* op)        { return VisitUnaryOperator_Impl(op);    }

    const std::unordered_set<uint64_t>& GetProcessedNodes() const { return m_rewrittenNodes; }
    void SetProcessedNodes(const std::unordered_set<uint64_t>& a_rhs) { m_rewrittenNodes = a_rhs; }

  protected:

    clang::Rewriter&               m_rewriter;
    const clang::CompilerInstance& m_compiler;
    MainClassInfo*                 m_codeInfo;
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    
    std::unordered_set<uint64_t> m_rewrittenNodes;
    void MarkRewritten(const clang::Stmt* expr);
    virtual std::string RecursiveRewrite(const clang::Stmt* expr); // double/multiple pass rewrite purpose

    bool WasNotRewrittenYet(const clang::Stmt* expr);
  
    std::string FunctionCallRewrite(const clang::CallExpr* call);
    std::string FunctionCallRewrite(const clang::CXXConstructExpr* call);
    
    virtual bool VisitFunctionDecl_Impl(clang::FunctionDecl* fDecl)       { return true; } // override this in Derived class
    virtual bool VisitCXXMethodDecl_Impl(clang::CXXMethodDecl* fDecl)     { return true; } // override this in Derived class

    virtual bool VisitVarDecl_Impl(clang::VarDecl* decl)                  { return true; } // override this in Derived class

    virtual bool VisitMemberExpr_Impl(clang::MemberExpr* expr)            { return true; } // override this in Derived class
    virtual bool VisitCXXMemberCallExpr_Impl(clang::CXXMemberCallExpr* f) { return true; } // override this in Derived class
    virtual bool VisitFieldDecl_Impl(clang::FieldDecl* decl)              { return true; } // override this in Derived class
    virtual bool VisitUnaryOperator_Impl(clang::UnaryOperator* op)        { return true; } // override this in Derived class

    virtual bool VisitCallExpr_Impl(clang::CallExpr* f);
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////  KernelRewriter  //////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  class KernelRewriter : public clang::RecursiveASTVisitor<KernelRewriter> // replace all expressions with class variables to kgen_data buffer access
  {
  public:
    
    KernelRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, MainClassInfo* a_codeInfo, kslicer::KernelInfo& a_kernel, const std::string& a_fakeOffsetExpr, const bool a_infoPass);
    virtual ~KernelRewriter() {}
    
    bool VisitVarDecl(clang::VarDecl* decl)                    { return VisitVarDecl_Impl(decl);        }

    bool VisitMemberExpr(clang::MemberExpr* expr)              { return VisitMemberExpr_Impl(expr); }
    bool VisitCXXMemberCallExpr(clang::CXXMemberCallExpr* f)   { return VisitCXXMemberCallExpr_Impl(f); }
    bool VisitCallExpr(clang::CallExpr* f)                     { return VisitCallExpr_Impl(f); }
    bool VisitCXXConstructExpr(clang::CXXConstructExpr* call)  { return VisitCXXConstructExpr_Impl(call); }
    bool VisitReturnStmt(clang::ReturnStmt* ret)               { return VisitReturnStmt_Impl(ret); }
                                                                           
    bool VisitUnaryOperator(clang::UnaryOperator* expr)        { return VisitUnaryOperator_Impl(expr);  }                       
    bool VisitBinaryOperator(clang::BinaryOperator* expr)      { return VisitBinaryOperator_Impl(expr); }    

    bool VisitCompoundAssignOperator(clang::CompoundAssignOperator* expr) { return VisitCompoundAssignOperator_Impl(expr); } 
    bool VisitCXXOperatorCallExpr   (clang::CXXOperatorCallExpr* expr)    { return VisitCXXOperatorCallExpr_Impl(expr); }

  protected:

    bool CheckIfExprHasArgumentThatNeedFakeOffset(const std::string& exprStr);
    void ProcessReductionOp(const std::string& op, const clang::Expr* lhs, const clang::Expr* rhs, const clang::Expr* expr);

    clang::Rewriter&                                         m_rewriter;
    const clang::CompilerInstance&                           m_compiler;
    MainClassInfo*                                           m_codeInfo;
    std::string                                              m_mainClassName;
    std::unordered_map<std::string, kslicer::DataMemberInfo> m_variables;
    const std::vector<kslicer::KernelInfo::Arg>&             m_args;
    const std::string&                                       m_fakeOffsetExp;
    bool                                                     m_kernelIsBoolTyped;
    bool                                                     m_kernelIsMaker;
    kslicer::KernelInfo&                                     m_currKernel;
    bool                                                     m_infoPass;
    std::unordered_set<uint64_t>                             m_rewrittenNodes;

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    std::string FunctionCallRewrite(const clang::CallExpr* call);
    std::string FunctionCallRewrite(const clang::CXXConstructExpr* call);
    std::string RecursiveRewrite   (const clang::Stmt* expr); // double/multiple pass rewrite purpose

    bool WasNotRewrittenYet(const clang::Stmt* expr);
    void MarkRewritten(const clang::Stmt* expr);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    virtual bool VisitMemberExpr_Impl(clang::MemberExpr* expr);
    virtual bool VisitCXXMemberCallExpr_Impl(clang::CXXMemberCallExpr* f);
    virtual bool VisitCallExpr_Impl(clang::CallExpr* f);
    virtual bool VisitCXXConstructExpr_Impl(clang::CXXConstructExpr* call);
    virtual bool VisitReturnStmt_Impl(clang::ReturnStmt* ret);
    
    // to detect reduction inside IPV programming template
    //
    virtual bool VisitUnaryOperator_Impl(clang::UnaryOperator* expr);                   // ++, --, (*var) =  ...
    virtual bool VisitCompoundAssignOperator_Impl(clang::CompoundAssignOperator* expr); // +=, *=, -=; to detect reduction
    virtual bool VisitCXXOperatorCallExpr_Impl(clang::CXXOperatorCallExpr* expr);       // +=, *=, -=; to detect reduction for custom data types (float3/float4 for example)
    virtual bool VisitBinaryOperator_Impl(clang::BinaryOperator* expr);                 // m_var = f(m_var, expr)

    virtual bool VisitVarDecl_Impl(clang::VarDecl* decl) { return true; } // override this in Derived class
  };

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct IShaderCompiler
  {
    IShaderCompiler(){}
    virtual ~IShaderCompiler(){}
    virtual std::string UBOAccess(const std::string& a_name) const = 0;
    virtual std::string ProcessBufferType(const std::string& a_typeName) const { return a_typeName; };

    virtual bool        IsSingleSource()   const = 0;
    virtual std::string ShaderSingleFile() const = 0;
    virtual std::string ShaderFolder()     const = 0;

    virtual void        GenerateShaders(nlohmann::json& a_kernelsJson, const std::string& mainClassFileName, const std::vector<std::string>& includeToShadersFolders) = 0;

    virtual std::string LocalIdExpr(uint32_t a_kernelDim, uint32_t a_wgSize[3]) const = 0;

    virtual std::string ReplaceCallFromStdNamespace(const std::string& a_call, const std::string& a_typeName) const { return a_call; }
    virtual bool        IsVectorTypeNeedsContructorReplacement(const std::string& a_typeName) const { return false; }
    virtual std::string VectorTypeContructorReplace(const std::string& a_typeName, const std::string& a_call) const { return a_call; }

    virtual bool        UseSeparateUBOForArguments() const { return false; }
    virtual bool        UseSpecConstForWgSize() const { return false; }
    virtual void        GetThreadSizeNames(std::string a_strs[3]) const = 0;

    virtual std::shared_ptr<kslicer::FunctionRewriter> MakeFuncRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, MainClassInfo* a_codeInfo) = 0;
    virtual std::shared_ptr<KernelRewriter>            MakeKernRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, MainClassInfo* a_codeInfo,
                                                                        kslicer::KernelInfo& a_kernel, const std::string& fakeOffs, bool a_infoPass) = 0;
  };

  struct ClspvCompiler : IShaderCompiler
  {
    ClspvCompiler(bool a_useCPP = false);
    std::string UBOAccess(const std::string& a_name) const override { return std::string("ubo->") + a_name; };
    bool        IsSingleSource()   const override { return true; }
    std::string ShaderFolder()     const override { return "clspv_shaders_aux"; }
    std::string ShaderSingleFile() const override { return "z_generated.cl"; }

    void        GenerateShaders(nlohmann::json& a_kernelsJson, const std::string& mainClassFileName, const std::vector<std::string>& includeToShadersFolders) override;

    bool        UseSeparateUBOForArguments() const override { return m_useCpp; }
    bool        UseSpecConstForWgSize()      const override { return m_useCpp; }
    
    std::string LocalIdExpr(uint32_t a_kernelDim, uint32_t a_wgSize[3])                               const override;
    std::string ReplaceCallFromStdNamespace(const std::string& a_call, const std::string& a_typeName) const override;
    bool        IsVectorTypeNeedsContructorReplacement(const std::string& a_typeName)                 const override;
    std::string VectorTypeContructorReplace(const std::string& a_typeName, const std::string& a_call) const override;
    void        GetThreadSizeNames(std::string a_strs[3])                                             const override;
    
    std::shared_ptr<kslicer::FunctionRewriter> MakeFuncRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, MainClassInfo* a_codeInfo) override;
    std::shared_ptr<KernelRewriter>            MakeKernRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, MainClassInfo* a_codeInfo, 
                                                                kslicer::KernelInfo& a_kernel, const std::string& fakeOffs, bool a_infoPass) override;

  private:
    std::string BuildCommand() const;

    std::unordered_map<std::string, std::string> m_ctorReplacement;
    bool m_useCpp;
  };

  struct GLSLCompiler : IShaderCompiler
  {
    std::string UBOAccess(const std::string& a_name) const override { return std::string("ubo.") + a_name; };
    bool        IsSingleSource()                     const override { return false; }
    std::string ShaderFolder()                       const override { return "shaders_generated"; }
    std::string ShaderSingleFile()                   const override { return ""; }
   
    void GenerateShaders(nlohmann::json& a_kernelsJson, const std::string& mainClassFileName, const std::vector<std::string>& includeToShadersFolders) override;

    std::string LocalIdExpr(uint32_t a_kernelDim, uint32_t a_wgSize[3]) const override;
    std::string ProcessBufferType(const std::string& a_typeName)        const override;
    void        GetThreadSizeNames(std::string a_strs[3])               const override;

    std::shared_ptr<kslicer::FunctionRewriter> MakeFuncRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, MainClassInfo* a_codeInfo) override;
    std::shared_ptr<KernelRewriter>            MakeKernRewriter(clang::Rewriter &R, const clang::CompilerInstance& a_compiler, MainClassInfo* a_codeInfo, 
                                                                kslicer::KernelInfo& a_kernel, const std::string& fakeOffs, bool a_infoPass) override;

  private:
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  class UsedCodeFilter;

  /**
  \brief collector of all information about input main class
  */
  struct MainClassInfo
  {
    std::unordered_map<std::string, KernelInfo>     allKernels;      ///<! list of all kernels; used only on the second pass to identify Control Functions; it is not recommended to use it anywhere else
    std::unordered_map<std::string, KernelInfo>     allOtherKernels; ///<! kernels from other classes. we probably need them if they are used.
    std::unordered_map<std::string, DataMemberInfo> allDataMembers;  ///<! list of all class data members;

    std::unordered_map<std::string, KernelInfo> kernels;         ///<! only those kernels which are called from 'Main'/'Control' functions
    std::vector<std::string>                    indirectKernels; ///<! list of all kernel names which require indirect dispatch; The order is essential because it is used for indirect buffer offsets 
    std::vector<DataMemberInfo>                 dataMembers;     ///<! only those member variables which are referenced from kernels 
    std::vector<MainFuncInfo>                   mainFunc;        ///<! list of all control functions

    std::string mainClassName;
    std::string mainClassFileName;
    std::string mainClassFileInclude;
    const clang::CXXRecordDecl* mainClassASTNode = nullptr;

    std::vector<std::string> includeToShadersFolders;
    std::vector<std::string> includeCPPFolders;  
   

    std::unordered_map<std::string, bool> allIncludeFiles; // true if we need to include it in to CL, false otherwise
    std::vector<KernelCallInfo>           allDescriptorSetsInfo;

    std::shared_ptr<IShaderCompiler>      pShaderCC = nullptr;
    
    uint32_t                              m_indirectBufferSize = 0; ///<! size of indirect buffer

    typedef std::vector<clang::ast_matchers::StatementMatcher>               MList;
    typedef std::unique_ptr<clang::ast_matchers::MatchFinder::MatchCallback> MHandlerCFPtr;
    typedef std::unique_ptr<kslicer::UsedCodeFilter>                         MHandlerKFPtr;

    virtual std::string RemoveKernelPrefix(const std::string& a_funcName) const;                       ///<! "kernel_XXX" --> "XXX"; 
    virtual bool        IsKernel(const std::string& a_funcName) const;                                 ///<! return true if function is a kernel
    virtual void        ProcessKernelArg(KernelInfo::Arg& arg, const KernelInfo& a_kernel) const { }   ///<!  
    virtual bool        IsIndirect(const KernelInfo& a_kernel) const; 
   
    //// Processing Control Functions (CF)
    // 
    virtual MList         ListMatchers_CF(const std::string& mainFuncName) = 0;
    virtual MHandlerCFPtr MatcherHandler_CF(kslicer::MainFuncInfo& a_mainFuncRef, const clang::CompilerInstance& a_compiler) = 0;
    virtual std::string   VisitAndRewrite_CF(MainFuncInfo& a_mainFunc, clang::CompilerInstance& compiler) = 0;

    virtual void AddSpecVars_CF(std::vector<MainFuncInfo>& a_mainFuncList, std::unordered_map<std::string, KernelInfo>& a_kernelList) {}

    virtual void PlugSpecVarsInCalls_CF(const std::vector<MainFuncInfo>&                      a_mainFuncList, 
                                        const std::unordered_map<std::string, KernelInfo>&    a_kernelList,
                                        std::vector<KernelCallInfo>&                          a_kernelCalls) {}    
    
    virtual bool SupportVirtualKernels() const { return false; }
    virtual void AddDispatchingHierarchy(const std::string& a_className, const std::string& a_makerName) { } ///<! for Virtual Kernels
    virtual void AddDispatchingKernel   (const std::string& a_className, const std::string& a_kernelName) { } ///<! for Virtual Kernels
    virtual void ProcessDispatchHierarchies(const std::vector<const clang::CXXRecordDecl*>& a_decls, const clang::CompilerInstance& a_compiler) {}
    virtual void ExtractHierarchiesConstants(const clang::CompilerInstance& compiler, clang::tooling::ClangTool& Tool) {}


    //// \\

    //// Processing Kernel Functions (KF)
    //
    virtual MList         ListMatchers_KF(const std::string& mainFuncName) = 0; 
    virtual MHandlerKFPtr MatcherHandler_KF(KernelInfo& kernel, const clang::CompilerInstance& a_compiler) = 0;
    virtual std::string   VisitAndRewrite_KF(KernelInfo& a_funcInfo, const clang::CompilerInstance& compiler, 
                                             std::string& a_outLoopInitCode, std::string& a_outLoopFinishCode);
    virtual void          VisitAndPrepare_KF(KernelInfo& a_funcInfo, const clang::CompilerInstance& compiler) { } // additional informational pass, does not rewrite the code! 

    virtual void ProcessCallArs_KF(const KernelCallInfo& a_call) { };
    virtual bool IsExcludedLocalFunction(const std::string& a_name) const { return false; }

    //// These methods used for final template text rendering
    //
    virtual uint32_t GetKernelDim(const KernelInfo& a_kernel) const = 0; 

    struct ArgTypeAndNamePair
    {
      std::string typeName;
      std::string argName;
      std::string sizeName;
      uint32_t    id;       ///<! used to preserve or change loops order
      bool        isUBO         = false;
      bool        isThreadFlags = false;
    };
   
    virtual std::vector<ArgTypeAndNamePair> GetKernelTIDArgs(const KernelInfo& a_kernel) const; 
    virtual std::vector<ArgTypeAndNamePair> GetKernelCommonArgs(const KernelInfo& a_kernel) const;

    virtual std::string GetCFSourceCodeCmd(MainFuncInfo& a_mainFunc, clang::CompilerInstance& compiler);
    virtual std::string GetCFDeclFromSource(const std::string& sourceCode);

    virtual bool NeedThreadFlags() const { return false; }
    virtual std::string RemoveTypeNamespaces(const std::string& a_str) const;

    virtual void AddTempBufferToKernel(const std::string a_buffName, const std::string a_elemTypeName, KernelInfo& a_kernel); ///<! if kernel need some additional buffers (for reduction for example) use this function 
    
    struct DImplFunc
    {
      const clang::CXXMethodDecl* decl = nullptr;
      std::string                 name;
      std::string                 srcRewritten;
      bool                        isEmpty       = false;
      bool                        isConstMember = false;
    };

    struct DImplClass
    {
      const clang::CXXRecordDecl* decl = nullptr;
      std::string                 name;
      std::vector<DImplFunc>      memberFunctions;
      std::vector<std::string>    fields;
      bool                        isEmpty = false; ///<! empty if all memberFunctions are empty
    };

    struct DHierarchy
    {
      const clang::CXXRecordDecl* interfaceDecl = nullptr;
      std::string                 interfaceName;
      std::string                 makerName;   
      std::string                 objBufferName;
      std::vector<DImplClass>     implementations;

      std::vector<kslicer::DeclInClass>            usedDecls;
      std::unordered_map<std::string, std::string> tagByClassName; 

      VKERNEL_IMPL_TYPE dispatchType = VKERNEL_SWITCH; ///<! simple variant by default
      uint32_t indirectBlockOffset   = 0;
    };
    
    kslicer::VKERNEL_IMPL_TYPE defaultVkernelType = kslicer::VKERNEL_SWITCH;

    std::unordered_map<std::string, DHierarchy> m_vhierarchy;
    virtual const std::unordered_map<std::string, DHierarchy>& GetDispatchingHierarchies() const { return m_vhierarchy; }
    virtual std::unordered_map<std::string, DHierarchy>&       GetDispatchingHierarchies()       { return m_vhierarchy; }
    
  };


  struct RTV_Pattern : public MainClassInfo
  {
    MList         ListMatchers_CF(const std::string& mainFuncName) override;
    MHandlerCFPtr MatcherHandler_CF(kslicer::MainFuncInfo& a_mainFuncRef, const clang::CompilerInstance& a_compiler) override;
    std::string   VisitAndRewrite_CF(MainFuncInfo& a_mainFunc, clang::CompilerInstance& compiler) override;

    void AddSpecVars_CF(std::vector<MainFuncInfo>& a_mainFuncList, std::unordered_map<std::string, KernelInfo>&  a_kernelList) override;

    void PlugSpecVarsInCalls_CF(const std::vector<MainFuncInfo>&                   a_mainFuncList, 
                                const std::unordered_map<std::string, KernelInfo>& a_kernelList,
                                std::vector<KernelCallInfo>&                       a_kernelCalls) override;    
    
    MList         ListMatchers_KF(const std::string& mainFuncName) override;
    MHandlerKFPtr MatcherHandler_KF(KernelInfo& kernel, const clang::CompilerInstance& a_compiler) override;
    void          ProcessCallArs_KF(const KernelCallInfo& a_call) override;  
    void          VisitAndPrepare_KF(KernelInfo& a_funcInfo, const clang::CompilerInstance& compiler) override; 

    uint32_t      GetKernelDim(const KernelInfo& a_kernel) const override;
    void          ProcessKernelArg(KernelInfo::Arg& arg, const KernelInfo& a_kernel) const override;   
    
    bool          SupportVirtualKernels() const { return true; }
    void          AddDispatchingHierarchy(const std::string& a_className, const std::string& a_makerName) override;  ///<! for Virtual Kernels 
    void          AddDispatchingKernel   (const std::string& a_className, const std::string& a_kernelName) override; ///<! for Virtual Kernels 
    void          ProcessDispatchHierarchies(const std::vector<const clang::CXXRecordDecl*>& a_decls, const clang::CompilerInstance& a_compiler) override;
    void          ExtractHierarchiesConstants(const clang::CompilerInstance& compiler, clang::tooling::ClangTool& Tool) override;

    bool NeedThreadFlags() const override { return true; } 
    bool IsExcludedLocalFunction(const std::string& a_name) const override 
    { 
      return (a_name == "MakeObjPtr"); 
    }                 

  private:
    std::vector< std::pair< std::string, std::string> > m_vkernelPairs;

  };

  struct IPV_Pattern : public MainClassInfo
  {
    std::string   RemoveKernelPrefix(const std::string& a_funcName) const override; ///<! "kernel2D_XXX" --> "XXX"; 
    bool          IsKernel(const std::string& a_funcName) const override;           ///<! return true if function is a kernel

    MList         ListMatchers_CF(const std::string& mainFuncName) override;
    MHandlerCFPtr MatcherHandler_CF(kslicer::MainFuncInfo& a_mainFuncRef, const clang::CompilerInstance& a_compiler) override;
    std::string   VisitAndRewrite_CF(MainFuncInfo& a_mainFunc, clang::CompilerInstance& compiler) override; 

    MList         ListMatchers_KF(const std::string& mainFuncName) override;
    MHandlerKFPtr MatcherHandler_KF(KernelInfo& kernel, const clang::CompilerInstance& a_compiler) override; 
    std::string   VisitAndRewrite_KF(KernelInfo& a_funcInfo, const clang::CompilerInstance& compiler, 
                                     std::string& a_outLoopInitCode, std::string& a_outLoopFinishCode) override;
    void          VisitAndPrepare_KF(KernelInfo& a_funcInfo, const clang::CompilerInstance& compiler) override;

    uint32_t      GetKernelDim(const KernelInfo& a_kernel) const override;
    void          ProcessKernelArg(KernelInfo::Arg& arg, const KernelInfo& a_kernel) const override; 

    std::vector<ArgTypeAndNamePair> GetKernelTIDArgs(const KernelInfo& a_kernel) const override; 
    bool NeedThreadFlags() const override { return false; }                   
  };


  /**
  \brief select local variables of main class that can be placed in auxilary buffer
  */
  std::vector<DataMemberInfo> MakeClassDataListAndCalcOffsets(std::unordered_map<std::string, DataMemberInfo>& vars);


  void ReplaceOpenCLBuiltInTypes(std::string& a_typeName);
  std::vector<std::string> GetAllPredefinedThreadIdNamesRTV();

  std::string GetRangeSourceCode(const clang::SourceRange a_range, const clang::CompilerInstance& compiler);
  std::string GetRangeSourceCode(const clang::SourceRange a_range, const clang::SourceManager& sm);
  std::string CutOffFileExt(const std::string& a_filePath);
  std::string CutOffStructClass(const std::string& a_typeName);
  std::string ReplaceSizeCapacityExpr(const std::string& a_str);
  

  uint64_t GetHashOfSourceRange(const clang::SourceRange& a_range);
  static constexpr size_t READ_BEFORE_USE_THRESHOLD = sizeof(float)*4;

  void PrintError(const std::string& a_msg, const clang::SourceRange& a_range, const clang::SourceManager& a_sm);
  //const clang::SourceManager&

};

template <typename Cont, typename Pred>
Cont filter(const Cont &container, Pred predicate) 
{
  Cont result;
  std::copy_if(container.begin(), container.end(), std::back_inserter(result), predicate);
  return result;
}


#endif