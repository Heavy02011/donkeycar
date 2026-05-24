# PLAN: Test PR #1233 on a Donkeycar 1/28 (Legacy Setup)

## Goal
Use this plan to reproducibly validate **PR #1233** from the official Donkeycar repository on a **Donkeycar 1/28 legacy (1:28 scale) setup**.

- PR: https://github.com/autorope/donkeycar/pull/1233
- 1/28 legacy reference: https://www.diyrobocars.com/2026/01/06/running-donkeycar-on-a-128-scale-car/

## Scope
1. Confirm PR #1233 can be checked out and installed on the target system.
2. Confirm baseline vehicle behavior still works (startup, camera/control loop, manual driving).
3. Validate PR-affected behavior and check for regressions.
4. Run an optional training/inference sanity check if relevant for the 1/28 setup.

## Prerequisites
- Working Donkeycar 1/28 hardware according to the legacy guide.
- Functional camera, steering, throttle, and power.
- Linux host/Raspberry Pi with Git, Python, and network access.
- Existing Donkeycar car app directory (with `manage.py`, `myconfig.py`, `vehicle.py`).

## Complete Command Checklist

> Replace placeholders:
> - `<REPO_DIR>`: local donkeycar repo path
> - `<CAR_DIR>`: local car app path
> - `<BASE_BRANCH>`: baseline branch (for example `dev` or `main`)
> - `<PYTHON>`: `python` or `python3`

### 0) Environment snapshot
```bash
date -u
uname -a
cd <REPO_DIR>
git remote -v
git status --short
git rev-parse --abbrev-ref HEAD
git rev-parse HEAD
```

### 1) Baseline validation (without PR #1233)
```bash
cd <REPO_DIR>
git fetch --all --prune
git checkout <BASE_BRANCH>
git pull --ff-only

# Optional: install/update editable package
<PYTHON> -m pip install -e .

# Record baseline commit
BASELINE_SHA=$(git rev-parse HEAD)
echo "BASELINE_SHA=$BASELINE_SHA"
```

Drive test from car directory:
```bash
cd <CAR_DIR>
# adjust options for your controller/camera settings
<PYTHON> manage.py drive
```

Record baseline observations:
- Startup successful (no exceptions)
- Steering responds correctly
- Throttle responds correctly
- Loop remains stable for several minutes

### 2) Checkout PR #1233
Option A (GitHub CLI):
```bash
cd <REPO_DIR>
gh pr checkout 1233
```

Option B (plain git):
```bash
cd <REPO_DIR>
git fetch origin pull/1233/head:pr-1233
git checkout pr-1233
```

Then install/update dependencies:
```bash
cd <REPO_DIR>
<PYTHON> -m pip install -U pip
<PYTHON> -m pip install -e .
PR_SHA=$(git rev-parse HEAD)
echo "PR_SHA=$PR_SHA"
```

### 3) Startup/build sanity on PR branch
```bash
cd <REPO_DIR>
<PYTHON> -c "import donkeycar; print(donkeycar.__version__)"

cd <CAR_DIR>
<PYTHON> manage.py drive
```

Verify:
- No startup exception
- Required parts load
- Control loop stays stable

### 4) On-vehicle functional test
From `<CAR_DIR>`:
```bash
<PYTHON> manage.py drive
```

Manual checks:
- Low-speed steering left/right response
- Throttle forward/stop response
- Record a short tub run for later sanity checks

### 5) Regression comparison
Compare baseline and PR behavior using your notes/logs:
```bash
cd <REPO_DIR>
echo "Baseline: $BASELINE_SHA"
echo "PR:       $PR_SHA"
git log --oneline --decorate -n 10
```

Classify result:
- No regression
- Acceptable behavior change
- Blocking issue found

### 6) Optional training/inference sanity
If you captured a tub:
```bash
cd <CAR_DIR>
# Example command names can vary by Donkeycar version;
# use the exact training/inference commands from your setup.
<PYTHON> manage.py train --tub <TUB_PATH> --model models/pr1233-test.h5
<PYTHON> manage.py drive --model models/pr1233-test.h5
```

## Result Template
- Date (UTC):
- Tester:
- Hardware variant (1/28 details):
- Baseline branch + SHA:
- PR branch + SHA:
- Python version / OS:

Checklist:
- [ ] Baseline startup/drive passed
- [ ] PR branch checkout/install passed
- [ ] PR startup/drive passed
- [ ] Manual steering/throttle checks passed
- [ ] No critical regression observed
- [ ] Optional training/inference sanity passed (if executed)

Observations:
- Positive findings:
- Regressions/anomalies:
- Repro steps for issues:

Decision:
- [ ] Approve PR #1233 for 1/28
- [ ] Approve with conditions
- [ ] Do not approve
