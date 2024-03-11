# from SCons.Script import Import
Import("env")

# Do not run a script when external applications, such as IDEs,
# dump integration data. Otherwise, input() will block the process
# waiting for the user input
if env.IsIntegrationDump():
    # stop the current script execution
    Return()

# Ask user name
print("Enter the Robot Index:")
# print()
user_name = input()
env.Append(
    CPPDEFINES=[("USER_NAME",  env.StringifyMacro(user_name))],
)

