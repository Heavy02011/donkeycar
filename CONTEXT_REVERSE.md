# CONTEXT_REVERSE.md

## 1) Objective
This context file provides the input parameters consumed by `PLAN_REVERSE.md` to regenerate this repository (or retarget the same plan to another domain by editing only this file).

## 2) Domain Profile (editable for reuse)
- DOMAIN_NAME: Autonomous RC/self-driving robotics
- PRIMARY_LANGUAGE: Python
- PACKAGE_MANAGER: pip + setuptools (PEP 517 build backend)
- PRIMARY_PACKAGE: donkeycar
- EXECUTABLE_NAME: donkey
- ARCH_STYLE: modular pipeline of reusable "parts" + runtime vehicle loop
- HARDWARE_INTEGRATION: Yes (Raspberry Pi, Arduino, sensors/actuators)
- UI_SURFACES: web templates + CLI + optional Kivy UI
- ML_SCOPE: TensorFlow/TFLite/PyTorch support via optional dependencies

## 3) Repository Identity (current target)
- REPO_OWNER: Heavy02011
- REPO_NAME: donkeycar
- TARGET_ROOT: `/tmp/workspace/Heavy02011/donkeycar`
- PACKAGE_VERSION: `5.3.dev1`
- PYTHON_RANGE: `>=3.11,<3.12`
- LICENSE: MIT

## 4) Top-Level Layout Contract
Required top-level entries:
- `.github/` (contains CI workflows + linter config)
- `arduino/` (firmware sketches)
- `docs/`
- `donkeycar/` (main Python package)
- `scripts/`
- `README.md`, `LICENSE`, `.gitignore`, `.coveragerc`, `MANIFEST.in`, `Makefile`, `pyproject.toml`, `setup.cfg`

## 5) Package Subtree Contract (`donkeycar/`)
Must include at least these functional areas:
- Core runtime: `__init__.py`, `vehicle.py`, `memory.py`, `config.py`, `utils.py`
- CLI/management: `management/`
- Runtime components: `parts/`
- Training/data pipeline: `pipeline/`
- App templates: `templates/`
- Tests: `tests/`
- Additional modules: `benchmarks/`, `gym/`, `contrib/`, `utilities/`

Observed **tracked** file-volume anchors from `git ls-files` (used for parity checks, tolerance ±10% when retargeting domains):
- `donkeycar/` files: 203
- `donkeycar/parts/` files: 88
- `donkeycar/tests/` files: 32
- `donkeycar/templates/` files: 19
- `donkeycar/pipeline/` files: 6
- `scripts/` files: 16
- `arduino/` files: 2
- `.github/workflows/` files: 2

## 6) Behavioral Contract
- Importing package prints banner and version (`donkeycar/__init__.py`).
- `Vehicle` class orchestrates part execution loop and memory I/O (`donkeycar/vehicle.py`).
- CLI entry point `donkey` routes to `donkeycar.management.base:execute_from_command_line`.
- `createcar` command materializes an app from `donkeycar/templates/*`.

## 7) Build/Test/Validation Contract
Primary validation commands for this repository:
1. `python -m pip install -e . --ignore-requires-python` (environment-dependent)
2. `python -m pytest`
3. `python -m pip wheel . --no-deps --no-build-isolation --ignore-requires-python`

Known current constraint in this environment:
- Native install with strict metadata fails on Python 3.12 because project requires `<3.12`.
- Baseline `pytest` collection fails without TensorFlow (`ModuleNotFoundError: tensorflow` from `donkeycar/tests/test_train.py`).

## 8) Regeneration Parameters (edit these to retarget other domains)
- PACKAGE_NAME
- CLI_COMMANDS map
- TEMPLATE_SET names
- OPTIONAL_DEPENDENCY_GROUPS
- HARDWARE_MODULE_FAMILIES
- TEST_STRATEGY + required ML backend(s)
- FILE_COUNT_ANCHORS per directory

When adapting to another domain, keep section structure identical and only change values.
