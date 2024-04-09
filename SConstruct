#!/usr/bin/env python
import os
import sys

env = SConscript("godot-cpp/SConstruct")


# For reference:
# - CCFLAGS are compilation flags shared between C and C++
# - CFLAGS are for C-specific compilation flags
# - CXXFLAGS are for C++-specific compilation flags
# - CPPFLAGS are for pre-processor flags
# - CPPDEFINES are for pre-processor defines
# - LINKFLAGS are for linking flags

# tweak this if you want to use different folders, or more folders, to store your source code in.
env.Append(CPPPATH=["src/", "demo/jsbsim/include/JSBSim"])
env.Append(LIBPATH=["demo/jsbsim/lib"])
env.Append(LIBS=["libjsbsim"])
sources = Glob("src/*.cpp")


if env["platform"] == "macos":
    library = env.SharedLibrary(
        "demo/bin/libfdm.{}.{}.framework/libfdm.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "demo/bin/libfdm{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)

env.Tool("compilation_db")
cdb = env.CompilationDatabase("compile_commands.json")
Alias("cdb", cdb)
