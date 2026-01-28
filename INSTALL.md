# Installation & Setup Guide

## Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Run the simulator
python run.py
```

That's it! The simulation will run and generate `simulation_result.png`.

---

## Dependencies

- Python 3.10+
- numpy >= 1.24.0
- matplotlib >= 3.7.0

---

## Running the Simulation

### Method 1: Using run.py (Recommended)
```bash
python run.py
```

### Method 2: Direct execution
```bash
python src/main.py
```

### Method 3: With explicit PYTHONPATH
```bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)/src"
python src/main.py
```

---

## Running Tests

```bash
python run_tests.py
```

All 24 tests should pass.

---

## Docker

Build the container:
```bash
docker build -t pid-navigator .
```

Run the simulation:
```bash
docker run pid-navigator
```

Note: To get the output image from Docker:
```bash
docker run -v $(pwd)/output:/app/output pid-navigator
```

---

## Troubleshooting

### Import Errors

**Problem**: `ImportError: No module named 'core'`

**Solution**: Use `run.py` instead of running `main.py` directly, or set PYTHONPATH:
```bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)/src"
```

### Matplotlib Display Issues

**Problem**: Matplotlib tries to open a window

**Solution**: Set the backend to non-interactive:
```bash
export MPLBACKEND=Agg
python run.py
```

This is already configured in the Dockerfile.

### Missing Dependencies

**Problem**: `ModuleNotFoundError: No module named 'numpy'`

**Solution**: Install requirements:
```bash
pip install -r requirements.txt
```

---

## Development Setup

For development with editing capabilities:

```bash
# Install in development mode
pip install -e .

# Or manually set PYTHONPATH
export PYTHONPATH="${PYTHONPATH}:$(pwd)/src"
```

---

## IDE Setup

### VS Code
Add to `.vscode/settings.json`:
```json
{
    "python.analysis.extraPaths": ["./src"]
}
```

### PyCharm
1. Right-click on `src` folder
2. Select "Mark Directory as" → "Sources Root"

---

## Project Structure

```
pid-waypoint-navigator/
├── src/               # Source code
│   ├── core/          # Robot, PID, Waypoint
│   ├── planning/      # A* pathfinding
│   ├── control/       # Path following
│   ├── simulation/    # Simulator & Visualizer
│   └── main.py        # Entry point
├── tests/             # Unit tests
├── run.py             # Run script (handles imports)
├── run_tests.py       # Test runner
└── requirements.txt   # Dependencies
```

---

## Next Steps

After installation:
1. Run the simulation: `python run.py`
2. View the output: `simulation_result.png`
3. Run tests: `python run_tests.py`
4. Experiment with parameters in `src/main.py`

