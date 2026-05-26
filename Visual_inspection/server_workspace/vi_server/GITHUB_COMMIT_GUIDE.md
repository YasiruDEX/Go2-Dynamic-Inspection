# GitHub Desktop Commit Instructions

## 🚨 Current Issue
GitHub Desktop is warning about large files (100MB+) in:
- `app/pipelines/gauge/archive.zip` (2.4GB!)
- Other large model files

## ✅ Solution: Use .gitignore

I've created a `.gitignore` file that excludes:
- Large model files (*.pt, *.pth)
- Virtual environments (.venv/)
- Database files (*.db)
- Generated data (data/jobs/)
- Archive files (archive.zip)

---

## 📝 Step-by-Step Instructions

### Step 1: Close the Warning Dialog
Click **"Cancel"** on the "Files too large" warning.

### Step 2: Refresh GitHub Desktop
1. Click **Repository** → **Refresh** (or press `Ctrl+R`)
2. The `.gitignore` file should now appear in the changes list
3. Large files should disappear from the list

### Step 3: Review Changes
You should now see only these types of files:
- ✅ Python source files (*.py)
- ✅ Configuration files (*.yaml, *.toml, *.md)
- ✅ Documentation (*.md)
- ✅ Scripts (*.bat, *.sh)
- ✅ `.gitignore` file
- ❌ NO large files (*.zip, *.pt, *.pth, .venv/)
- ❌ NO database files (*.db)
- ❌ NO generated data (data/jobs/)

### Step 4: Write Commit Message
In the bottom-left corner:

**Summary (required):**
```
Integrate VLM and Gauge pipelines for visual inspection
```

**Description (optional):**
```
- Added VLM pipeline with 6 object types (door, fire_extinguisher, emergency_exit, main_cylinder, unknown, gauge)
- Integrated analog gauge reader pipeline
- Implemented queue worker for background processing
- Added database storage for jobs and results
- Created comprehensive documentation
- Configured proper .gitignore for large files
```

### Step 5: Commit
1. Click **"Commit to main"** button (bottom-left)
2. Wait for commit to complete

### Step 6: Push to GitHub
1. Click **"Push origin"** button (top toolbar)
2. Wait for upload to complete

---

## 🔍 What Gets Committed

### ✅ Included (Source Code):
```
app/
├── main.py
├── models.py
├── schemas.py
├── settings.py
├── storage.py
├── queue_worker.py
├── db.py
├── pipelines/
│   ├── gauge_pipeline.py
│   ├── vlm/
│   │   ├── __init__.py
│   │   ├── vlm_router.py
│   │   ├── vlm_client.py
│   │   ├── schemas.py
│   │   ├── postprocess.py
│   │   └── prompts/*.yaml
│   └── gauge/
│       ├── pipeline.py
│       ├── plots.py
│       ├── pyproject.toml
│       ├── README.md
│       └── (source code only)
scripts/
├── upload_test.py
├── upload_test_vlm.py
pyproject.toml
.env.example
README.md
(and all documentation .md files)
```

### ❌ Excluded (Large/Generated Files):
```
.venv/                          # Virtual environment
app/pipelines/gauge/.venv/      # Gauge venv
app/pipelines/gauge/archive.zip # 2.4GB archive
app/pipelines/gauge/models/*.pt # Model weights
app/pipelines/gauge/dependencies/*.pth # Dependencies
data/vi_server.db               # Database
data/jobs/                      # Uploaded images
.env                            # Environment secrets
__pycache__/                    # Python cache
*.log                           # Log files
```

---

## 🚨 Troubleshooting

### Issue: Large files still showing
**Solution:**
1. Close GitHub Desktop
2. Open Command Prompt in project folder
3. Run:
   ```bash
   git rm --cached app/pipelines/gauge/archive.zip
   git rm --cached -r app/pipelines/gauge/.venv
   git rm --cached -r app/pipelines/gauge/models/*.pt
   ```
4. Reopen GitHub Desktop
5. Commit the changes

### Issue: .gitignore not working
**Solution:**
1. Make sure `.gitignore` is in the root of `vi_server/` folder
2. Refresh GitHub Desktop (`Ctrl+R`)
3. If still not working, restart GitHub Desktop

### Issue: Too many files (174 changed files)
**This is normal!** You're committing:
- All Python source files
- All configuration files
- All documentation
- All scripts

Just make sure NO large files (>100MB) are in the list.

---

## ✅ After Successful Commit

Your repository will contain:
- ✅ Complete source code
- ✅ Documentation
- ✅ Configuration files
- ✅ Scripts
- ✅ `.gitignore` for future commits

**NOT included** (and that's good!):
- ❌ Virtual environments (can be recreated)
- ❌ Large model files (can be downloaded)
- ❌ Database files (generated at runtime)
- ❌ Uploaded images (generated at runtime)

---

## 📋 Quick Checklist

Before committing:
- [ ] `.gitignore` file is in `vi_server/` folder
- [ ] Refreshed GitHub Desktop (`Ctrl+R`)
- [ ] No files over 100MB in changes list
- [ ] No `.venv/` folders in changes list
- [ ] No `archive.zip` in changes list
- [ ] No `*.pt` or `*.pth` files in changes list
- [ ] Commit message is written
- [ ] Ready to commit!

---

## 🎯 Next Steps After Commit

1. **Push to GitHub** - Click "Push origin"
2. **Verify on GitHub.com** - Check your repository online
3. **Add README** - Update main README with project info
4. **Tag release** - Optional: Tag this as v1.0.0

---

**You're ready to commit!** Just follow the steps above. 🚀
