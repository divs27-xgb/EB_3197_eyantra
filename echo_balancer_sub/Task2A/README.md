# Task 2A — Echo Balancer bike

Task 2A balances the bike bot with PID and LQR. PID and LQR submission artifacts are under [`submissions/`](submissions/).

## Contents

- [`docs/bike_lagrangian_model.md`](docs/bike_lagrangian_model.md): bike-specific constrained Lagrangian derivation and the continuous-time A/B candidate entered in the tuner.
- [`scripts/derive_bike_lqr_model.py`](scripts/derive_bike_lqr_model.py): offline NumPy calculation of the model matrices.
- [`scripts/capture_lqr_telemetry.py`](scripts/capture_lqr_telemetry.py): passive CoppeliaSim telemetry logger; it reads values and does not command the bike.
- `scripts/inspect_*.py`: read-only CoppeliaSim scene and joint inspectors.
- `results/`: recorded controller settings and evaluator output.
- `submissions/`: preserved submitted ZIP and output files.

The model candidate stabilized the bike immediately in the LQR Tuner with default `Q=I`, `R=1`. Record a full evaluator-duration run separately before treating that as the final validation result.

## Local-only files

Official downloads and executables are kept locally under `reference/official/`; raw captures are in `local_data/`; detailed working notes are in `private_notes/`. Those files are ignored by Git. Obtain the official scene and evaluators from the e-Yantra portal if you need to reproduce the simulation on another machine.

The telemetry scripts need a running CoppeliaSim scene and its ZeroMQ Remote API Python client. The matrix calculator only needs Python and NumPy.

