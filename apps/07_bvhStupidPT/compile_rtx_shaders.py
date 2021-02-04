import os

if __name__ == '__main__':
    glslang_cmd = "/home/vs/Software/glslang/bin/glslangValidator"
    #glslang_cmd = "glslangValidator"
    os.system("{} --target-env spirv1.4 -V -S rgen raygen.rgen -o raygen.rgen.spv".format(glslang_cmd))
    os.system("{} --target-env spirv1.4 -V -S rchit rayhit.rchit -o rayhit.rchit.spv".format(glslang_cmd))
    os.system("{} --target-env spirv1.4 -V -S rmiss raymiss.rmiss -o raymiss.rmiss.spv".format(glslang_cmd))
    os.system("{} --target-env vulkan1.2 -V  rtx.comp -o rtx.comp.spv".format(glslang_cmd))

