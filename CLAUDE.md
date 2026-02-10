# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BEHAVIOR-1K is a simulation benchmark for embodied AI research, providing 1,000 everyday household activities built on NVIDIA's Omniverse/Isaac Sim platform. This is a monorepo with several interconnected components.

## Repository Components

- **OmniGibson/** — Core simulation engine and robotics framework (environments, robots, sensors, tasks, object states). Gym-compatible. This is the largest and most active component.
- **bddl3/** — Behavior Domain Definition Language: predicate-logic DSL for defining activities. Contains knowledge base models and 1000+ activity definitions in `bddl/activity_definitions/`.
- **joylo/** — Teleoperation system for robot data collection (Gello hardware integration).
- **knowledgebase/** — Flask web app / static site for browsing BDDL entities. Has its own `CLAUDE.md`.
- **docs/** — mkdocs-based documentation site (Material theme).
- **asset_pipeline/** — 3D asset processing pipelines using DVC.
- **eval-jobqueue/** — Distributed evaluation job management.
- **docker/** — Dockerfiles based on NVIDIA Isaac Sim base images.

## Installation

```bash
# Full setup (modular — pick components you need)
./setup.sh --new-env --bddl --omnigibson --dataset

# Development install from source
pip install -e bddl3/
pip install -e OmniGibson/[dev,primitives,eval] --no-build-isolation
```

Requires Python 3.10, NVIDIA GPU with CUDA 12.1+, and conda for environment management. See `setup.sh --help` for all flags. The `--omnigibson` flag requires `--bddl`; `--primitives` and `--eval` require `--omnigibson`.

## Common Commands

### Testing (OmniGibson)
Tests are in `OmniGibson/tests/` and require GPU + downloaded datasets (self-hosted CI runners).

```bash
# Run all tests
cd OmniGibson && pytest tests/

# Run a single test file
cd OmniGibson && pytest -s tests/test_envs.py

# Run with JUnit output (as CI does)
cd OmniGibson && pytest -s tests/test_transform_utils.py --junitxml=results.xml
```

### Linting & Formatting
Pre-commit hooks run ruff on `OmniGibson/` only. Root `ruff.toml` excludes `joylo/`.

```bash
# Run linter + formatter (from repo root)
pre-commit run --all-files

# Or directly
ruff check OmniGibson/
ruff format OmniGibson/
```

Ruff config is in `OmniGibson/pyproject.toml`: line-length 120, target Python 3.10, rules E4/E7/E9/F (with E731, E722, E741 ignored).

### Documentation
```bash
# Serve docs locally
mkdocs serve

# Build knowledgebase static site
cd knowledgebase && python static_generator.py -o build
```

### Docker
```bash
docker pull stanfordvl/behavior:main
docker run --gpus all -it stanfordvl/behavior:main
```

## Architecture Notes

- **Gym interface**: Environments in `OmniGibson/omnigibson/envs/` follow the Gymnasium API.
- **BDDL ↔ OmniGibson integration**: `OmniGibson/omnigibson/utils/bddl_utils.py` bridges BDDL predicates to simulation object states.
- **Object states**: `OmniGibson/omnigibson/object_states/` implements semantic/kinematic predicates (e.g., cooked, sliced, inside) that BDDL task definitions reference.
- **Modular controllers**: Robot control schemes are pluggable in `OmniGibson/omnigibson/robots/`.
- **Action primitives**: High-level robot actions in `OmniGibson/omnigibson/action_primitives/`.
- **Task definitions**: BDDL problem files in `bddl3/bddl/activity_definitions/` define goals; reward/termination logic lives in `OmniGibson/omnigibson/reward_functions/` and `OmniGibson/omnigibson/termination_conditions/`.

## CI/CD

Tests run on self-hosted GPU runners via `.github/workflows/tests.yml` — a matrix of 18 test files executed in parallel. CI installs BDDL and OmniGibson from source before running pytest. Other workflows handle Docker builds, PyPI publishing, documentation, and profiling.
