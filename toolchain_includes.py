import os

Import("env")

# Include toolchain paths
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)
