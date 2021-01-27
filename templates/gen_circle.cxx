/////////////////////////////////////////////////////////////////////
/////////////////// include files ///////////////////////////////////
/////////////////////////////////////////////////////////////////////

## for Incl in Includes  
#include "{{Incl}}"
## endfor

/////////////////////////////////////////////////////////////////////
/////////////////// declarations in class ///////////////////////////
/////////////////////////////////////////////////////////////////////
## for Decl in ClassDecls  
{{Decl}}
## endfor

#include "include/{{UBOIncl}}"

/////////////////////////////////////////////////////////////////////
/////////////////// local functions /////////////////////////////////
/////////////////////////////////////////////////////////////////////

## for LocalFunc in LocalFunctions  
{{LocalFunc}}

## endfor
#define KGEN_FLAG_RETURN 1
#define KGEN_FLAG_BREAK  2
#define KGEN_FLAG_DONT_SET_EXIT 4
#define KGEN_FLAG_SET_EXIT_NEGATIVE 8

/////////////////////////////////////////////////////////////////////
/////////////////// kernels /////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
## for Kernel in Kernels  

## for Arg in Kernel.Args 
[[using spirv: buffer, binding({{ loop.index }})]] {{Arg.Type}} {{Arg.Name}}[];  
## endfor
[[using spirv: buffer, binding({{ Kernel.UBOBinding }})]] struct {{MainClassName}}_UBO_Data ubo;

extern "C" [[using spirv: comp, local_size(16,16), push]]
void {{Kernel.Name}}(
## for UserArg in Kernel.UserArgs 
  {{UserArg.Type}} {{UserArg.Name}},
## endfor
  const uint {{Kernel.threadIdName1}}, 
  const uint {{Kernel.threadIdName2}},
  const uint {{Kernel.threadIdName3}},
  const uint kgen_tFlagsMask)
{
  /////////////////////////////////////////////////////////////////
  {% for name in Kernel.threadNames %}const uint {{name}} = get_global_id({{ loop.index }}); 
  {% endfor %}{% if Kernel.threadDim == 3 %}if({{Kernel.threadName1}} >= {{Kernel.threadIdName1}} || {{Kernel.threadName2}} >= {{Kernel.threadIdName2}} || {{Kernel.threadName3}} >= {{Kernel.threadIdName3}})
    return;{% else if Kernel.threadDim == 2 %}if({{Kernel.threadName1}} >= {{Kernel.threadIdName1}} || {{Kernel.threadName2}} >= {{Kernel.threadIdName2}})
    return;{% else %}if({{Kernel.threadName1}} >= {{Kernel.threadIdName1}})
    return;{% endif %}
  {% if Kernel.shouldCheckExitFlag %}if((kgen_threadFlags[{{Kernel.ThreadOffset}}] & kgen_tFlagsMask) != 0) 
    return;{% endif %}
  {% for Vec in Kernel.Vecs %}const uint {{Vec.Name}}_size = ubo->{{Vec.Name}}_size; 
  {% endfor %}{% for Member in Kernel.Members %}const {{Member.Type}} {{Member.Name}} = ubo->{{Member.Name}};
  {% endfor %}/////////////////////////////////////////////////////////////////
{{Kernel.Source}}
}

## endfor