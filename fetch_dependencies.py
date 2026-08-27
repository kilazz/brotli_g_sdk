#!/usr/bin/env python3
"""
Simple script to update external dependencies required by the Brotli-G project.

Usage:
    python fetch_dependencies.py [latest]
"""

import os
import shutil
import subprocess
import sys
import tarfile
import zipfile

IS_PYTHON_3 = sys.version_info[0] >= 3
if IS_PYTHON_3:
    import urllib.request
else:
    import urllib

# Paths configuration
SCRIPT_ROOT = os.path.dirname(os.path.realpath(__file__))
BROTLI_G_ROOT = SCRIPT_ROOT

# Shell execution argument for Windows
SHELL_ARG = sys.platform.startswith("win32")

# Verify Git installation
try:
    subprocess.call(["git", "--version"], shell=SHELL_ARG)
except OSError:
    print("Error: Unable to find git executable. Please ensure Git is in your PATH.")
    sys.exit(1)

# Retrieve repository origin URL
try:
    git_url_bytes = subprocess.check_output(
        ["git", "-C", SCRIPT_ROOT, "remote", "get-url", "origin"],
        shell=SHELL_ARG,
    )
except subprocess.CalledProcessError:
    print("Error: Unable to determine Git origin for Brotli-G project.")
    sys.exit(1)

git_url_str = str(git_url_bytes).lstrip("b'").rstrip("'")
git_root = git_url_str.rsplit("/", 1)[0] + "/"

# Branch fallback
git_branch = "master" if "github" in git_url_str else "amd-master"

print(f"\nFetching dependencies from: {git_root} (default branch: {git_branch})")

# Target folders
EXTERNAL_LOC = "external/"
SAMPLE_EXTERNAL_LOC = "sample/external/"

# Git dependencies
gitMapping = {
    "https://github.com/google/brotli.git": [
        EXTERNAL_LOC + "brotli",
        "v1.2.0",
    ],
    "https://github.com/microsoft/DirectX-Headers.git": [
        SAMPLE_EXTERNAL_LOC + "DirectX-Headers",
        "v1.619.5",
    ],
}

# Prebuilt binary archives (DXC)
downloadMapping = {
    "https://github.com/microsoft/DirectXShaderCompiler/releases/download/v1.9.2607/dxc_2026_07_29.zip": (
        SAMPLE_EXTERNAL_LOC + "dxc_2026_07_29/"
    )
}

USER_AGENT = (
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
    "AppleWebKit/537.36 (KHTML, like Gecko) "
    "Chrome/120.0.0.0 Safari/537.36"
)


def download_git_repo(repo_url: str, dest_path: str, reqd_commit: str) -> None:
    """Clones or updates a Git repository to a specific commit or tag."""
    if (len(sys.argv) > 1 and sys.argv[1] == "latest") or reqd_commit is None:
        reqd_commit = git_branch

    print(f"\nChecking out commit/tag: '{reqd_commit}' for {repo_url}")
    os.chdir(BROTLI_G_ROOT)

    if os.path.isdir(dest_path):
        print(
            f"Directory '{dest_path}' exists.\n\tFetching latest changes from {repo_url}"
        )
        sys.stdout.flush()
        try:
            subprocess.check_call(
                ["git", "-C", dest_path, "fetch", "origin", "--tags"], shell=SHELL_ARG
            )
            subprocess.check_call(
                ["git", "-C", dest_path, "checkout", reqd_commit], shell=SHELL_ARG
            )
        except subprocess.CalledProcessError as err:
            print(f"Git operation failed with error code: {err.returncode}")
        sys.stderr.flush()
        sys.stdout.flush()
    else:
        print(f"Directory '{dest_path}' does not exist.\n\tCloning from {repo_url}")
        sys.stdout.flush()
        try:
            subprocess.check_call(
                ["git", "clone", repo_url, dest_path], shell=SHELL_ARG
            )
            subprocess.check_call(
                ["git", "-C", dest_path, "checkout", reqd_commit], shell=SHELL_ARG
            )
        except subprocess.CalledProcessError as err:
            print(f"Git clone/checkout failed with return code: {err.returncode}")
            sys.exit(1)
        sys.stderr.flush()
        sys.stdout.flush()


def download_and_extract(url: str, dest_relative_path: str) -> None:
    """Downloads and extracts a ZIP/TAR.GZ archive."""
    target_path = os.path.normpath(os.path.join(SCRIPT_ROOT, dest_relative_path))
    if not os.path.isdir(target_path):
        os.makedirs(target_path)

    filename = url.split("/")[-1].split("#")[0].split("?")[0]
    archive_path = os.path.join(target_path, filename)

    if not os.path.isfile(archive_path):
        print(f"\nDownloading {url} -> {archive_path}")
        if IS_PYTHON_3:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with (
                urllib.request.urlopen(req) as response,
                open(archive_path, "wb") as out_file,
            ):
                shutil.copyfileobj(response, out_file)
        else:
            urllib.urlretrieve(url, archive_path)

        ext = os.path.splitext(archive_path)[1].lower()
        if ext == ".zip":
            with zipfile.ZipFile(archive_path, "r") as zip_ref:
                zip_ref.extractall(target_path)
            os.remove(archive_path)
        elif ext in [".gz", ".tgz", ".tar"]:
            with tarfile.open(archive_path, "r:*") as tar_ref:
                tar_ref.extractall(target_path)
            os.remove(archive_path)


def main() -> None:
    # 1. Fetch Git dependencies
    for repo_url, (dest_dir, tag) in gitMapping.items():
        download_git_repo(repo_url, dest_dir, tag)

    # 2. Download prebuilt archives
    for archive_url, dest_dir in downloadMapping.items():
        download_and_extract(archive_url, dest_dir)

    print("\nFetching dependencies finished successfully.\n")


if __name__ == "__main__":
    main()
