Import("env")
import subprocess
import os
from datetime import datetime

# sources
# - https://docs.platformio.org/en/latest/projectconf/sections/env/options/build/build_flags.html#dynamic-build-flags
# - https://docs.platformio.org/en/latest/scripting/examples/platformio_ini_custom_options.html
# - https://docs.platformio.org/en/latest/scripting/examples/asking_for_input.html

# get branch name using "git rev-parse --abbrev-ref HEAD"
branch = (
    subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"])
    .strip()
    .decode("utf-8")
)
print(f"branch name is: {branch}")

# get short hash of most recent commit using "git rev-parse --short HEAD"
commit = (
    subprocess.check_output(["git", "rev-parse", "--short", "HEAD"])
    .strip()
    .decode("utf-8")
)
print(f"commit short hash is: {commit}")

# get current local time
current_time = datetime.now()
formatted_time = current_time.strftime('%Y-%m-%d %H:%M:%S')
print(f"current time is: {formatted_time}")

# create version string
# should look like "branch 'production' at commit a01c1e9, built/uploaded 2024-08-21 22:17:10"
version = f"branch '{branch}' at commit {commit}, built/uploaded {formatted_time}"

# add C preprocessor define 'PR_CODEBASE_VERSION' as the version string
env.Append(
    CPPDEFINES=[("PR_CODEBASE_VERSION", env.StringifyMacro(version))],
)

# Get the value of PR_GUI_BOT_INDEX' passed from the command line or environment variables
bot_index = os.environ.get('PR_GUI_BOT_INDEX', '0')  # Default to 0 if not provided

# Append the build flag to the environment
env.Append(
    CPPDEFINES=[("BOT_INDEX", bot_index)],
)
print(f"bot index is: {bot_index}")