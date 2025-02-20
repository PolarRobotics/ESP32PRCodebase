import subprocess

branch = (
    subprocess.check_output(["pio", "run", "--environment", "robot", "--target", "upload"])
    .strip()
    .decode("utf-8")
)
print(f"branch name is: {branch}") 
# you need to research scripting