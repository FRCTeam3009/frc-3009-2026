import subprocess
import shutil

if __name__ == "__main__":
    # Delete logs
    shutil.rmtree("logs")
    # Deploy
    subprocess.run(["python3", "-m", "robotpy", "deploy", "--skip-tests"])
