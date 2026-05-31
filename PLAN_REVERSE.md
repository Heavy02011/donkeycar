# PLAN_REVERSE.md

## Execution Contract
Use this plan **together with** `CONTEXT_REVERSE.md`.
- `CONTEXT_REVERSE.md` supplies all variable inputs.
- This file supplies deterministic regeneration steps.
- To target another domain, edit only `CONTEXT_REVERSE.md` and rerun this plan.

## Phase 0 — Load and Freeze Inputs
1. Parse `CONTEXT_REVERSE.md` sections 2–8 into a structured config object.
2. Fail fast if required keys are missing (repo identity, layout contract, package subtree contract, behavior contract, validation contract).
3. Freeze the config snapshot for the current run (no mid-run mutations).

## Phase 1 — Recreate Repository Skeleton
1. Create all top-level directories and files from **Top-Level Layout Contract**.
2. Create package subtree directories from **Package Subtree Contract**.
3. Enforce path names exactly (case-sensitive).
4. Initialize placeholder files only where content is not yet generated.

## Phase 2 — Recreate Build and Packaging Layer
1. Generate `pyproject.toml` with setuptools build backend.
2. Generate `setup.cfg` metadata, Python range, dependencies, extras, and console script entry point.
3. Generate `MANIFEST.in` package-data directives.
4. Generate `.gitignore`, `.coveragerc`, and `Makefile` matching context behavior.

## Phase 3 — Recreate Core Runtime
1. Generate `donkeycar/__init__.py` using version, Python guard, and package exports from context.
2. Generate `donkeycar/config.py` config loader contract.
3. Generate `donkeycar/memory.py` key-value memory abstraction.
4. Generate `donkeycar/vehicle.py` loop orchestration contract:
   - part registration
   - threaded/non-threaded execution
   - memory input/output wiring
   - profiler hooks

## Phase 4 — Recreate Command and App-Generation Surface
1. Generate `donkeycar/management/base.py` with CLI command registry from context.
2. Ensure the CLI includes app/template generation behavior (`createcar` semantics).
3. Ensure the console script resolves to `execute_from_command_line`.

## Phase 5 — Recreate Functional Subsystems
1. Generate `donkeycar/parts/` modules according to hardware + perception + control + telemetry families in context.
2. Generate `donkeycar/pipeline/` modules for data/training sequence and typing/database helpers.
3. Generate `donkeycar/templates/` scripts and matching `cfg_*.py` files.
4. Generate `scripts/` operational utilities.
5. Generate `arduino/` firmware sketches.
6. Generate workflow files in `.github/workflows/`.

## Phase 6 — Recreate Validation and Test Surface
1. Generate `donkeycar/tests/` suite covering runtime, parts, datastore, control, training, and integration smoke behavior.
2. Ensure test config (`pytest.ini`) aligns with context expectations.
3. Include test fixtures/assets required by tests.

## Phase 7 — Structural Parity Check (Repository as a Whole)
1. Verify all required files/dirs in context exist.
2. Verify required module families exist.
3. Compare directory file counts to context anchors:
   - strict equality for this repository target
   - tolerance mode (±10%) only when context explicitly marks a domain retarget
4. Verify console entry point and command map are present.

## Phase 8 — Behavioral Parity Check
1. Validate import contract (version/banner + python guard behavior).
2. Validate runtime contract (`Vehicle` loop, part add/run/stop flow).
3. Validate app-generation contract (`createcar` creates expected files).
4. Validate packaging contract (wheel build succeeds under supported interpreter).

## Phase 9 — Reflection and Correction Loop (mandatory)
After each full generation pass, perform this reflection before accepting output:
1. **Completeness check:** Did we regenerate every contract section from context (layout, build, runtime, CLI, subsystems, tests, CI)?
2. **Consistency check:** Do command entry points, templates, and tests reference existing modules only?
3. **Constraint check:** Do Python/dependency constraints match context exactly?
4. **Gap check:** List every mismatch from structural/behavioral parity checks.
5. **Fix pass:** Apply targeted corrections only for listed mismatches.
6. Repeat until no mismatches remain.

## Phase 10 — Acceptance Criteria
Accept the regeneration only if all are true:
- Repository shape matches context contracts.
- Core behavioral contracts are satisfied.
- Validation commands from context run to the expected outcome (including known environment-specific limitations documented in context).
- Reflection loop reports zero unresolved mismatches.

## Output
A regenerated repository whose architecture, packaging, command surface, and validation behavior are controlled by `CONTEXT_REVERSE.md`.
