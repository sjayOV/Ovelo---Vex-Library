# Upload Ovelo to your GitHub

These steps publish your reviewed files. They do not upload a program to a V5 Brain; that uses `pros upload` from the generated PROS project.

## Before uploading

1. Run `make test` and the available verification commands.
2. Review `git status --short`. Do not commit build outputs, downloaded SDK directories, credentials, or private robot/team data.
3. Read the README's validation status. Keep the unverified SDK/hardware statement until those checks actually pass.
4. Retain `LICENSE` and any third-party notices. Add your own contributor details without deleting the supplied attribution.

The ignore file excludes generated desktop build products. A generated PROS project outside this checkout is a separate project; decide deliberately whether to publish that robot-specific project or this portable template.

## Option A: use your existing `sjayOV/Ovelo-` repository

Because the cloud checkout could not retrieve its remote history, the safest local route is a fresh authenticated clone. This preserves existing history rather than assuming the remote repository is empty.

1. Download the generated Ovelo files from this task onto your computer and extract them into a temporary folder.
2. Open your normal terminal and clone your repository:

   ```sh
   git clone https://github.com/sjayOV/Ovelo-.git
   cd Ovelo-
   git switch main
   git pull --ff-only origin main
   ```

3. Copy the reviewed template files into the clone. Include `.gitignore` and `.github/`; hidden files may not appear in the file manager by default. **Do not copy a `.git` directory** from an archive or another checkout. Inspect existing files before replacing them.
4. Review and commit:

   ```sh
   git status --short
   git diff
   git add README.md CHANGELOG.md LICENSE Makefile .gitignore include examples tests tools docs .github
   git diff --cached --stat
   git commit -m "Add Ovelo educational robotics template"
   git push origin main
   ```

If your default branch has another name, use that branch consistently. If `main` does not exist because the remote is genuinely empty, create it with `git switch -c main` before adding files. If pushing to `main` is protected, create a branch such as `ovelo-template`, push that branch, and open a pull request.

If GitHub rejects the push because the remote changed, fetch and review the new commits, then integrate them. Do not use `--force` to bypass the rejection. If your files conflict with an existing implementation, resolve the differences deliberately instead of overwriting it blindly.

## Option B: create a new repository

1. Sign in to GitHub and choose **New repository**.
2. Name it `Ovelo` or a name you prefer. Choose its visibility. Leave automatic README, license, and ignore-file creation unchecked because this project already supplies them.
3. Extract the Ovelo archive, open a terminal in that folder, and run:

   ```sh
   git init -b main
   git add README.md CHANGELOG.md LICENSE Makefile .gitignore include examples tests tools docs .github
   git diff --cached --stat
   git commit -m "Initial Ovelo template"
   git remote add origin https://github.com/YOUR-USERNAME/Ovelo.git
   git push -u origin main
   ```

Replace `YOUR-USERNAME` with your GitHub username. If the folder is already a Git checkout, inspect `git remote -v` and its history first; do not rerun initialization or replace the remote without understanding which repository it belongs to.

Use GitHub's supported credential manager, GitHub Desktop, or `gh auth login` on your own machine if authentication is needed. Never put a token in source files or paste it into chat. The cloud environment's injected Git authentication does not imply that your local computer is signed in.

## A browser-only upload

For a small initial upload, open the intended repository and choose **Add file → Upload files**. Upload the extracted project contents, retain the directory structure, and ensure hidden files such as `.github/workflows/ci.yml` and `.gitignore` are included. Git is more reliable for preserving hidden files and reviewing changes.

## After the push

1. Open the repository's **Actions** tab. Inspect the desktop and real SDK job results separately. A skipped or failed SDK job is not a passing V5 build.
2. Follow the [hardware acceptance checklist](verification.md) and record actual results before claiming robot compatibility.
3. To let others start from it, open **Settings → General** and enable **Template repository** if you have permission.
4. Create a tagged release only after reviewing the contents and stating the validation level. Do not describe a hosted simulation as hardware verification.

No remote push is performed automatically by this task. These steps let you choose the repository, review the files, and publish them yourself.
