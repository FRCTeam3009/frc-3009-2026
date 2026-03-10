import subprocess
import shutil
import os.path

if __name__ == "__main__":
    # Delete logs
    if os.path.isdir("logs"):
        shutil.rmtree("logs")
    # Deploy
    subprocess.run(["python3", "-m", "robotpy", "deploy", "--skip-tests"])
