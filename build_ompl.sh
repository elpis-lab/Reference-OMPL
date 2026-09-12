#!/usr/bin/env bash
set -euo pipefail

# --- sudo handling ---
if [[ "$(id -u)" == "0" ]]; then
  SUDO=""
else
  SUDO="sudo -H"
fi

# --- sanity check: must be in OMPL source root ---
if [[ ! -f "CMakeLists.txt" ]] || ! grep -q "OMPL" CMakeLists.txt 2>/dev/null; then
  echo "Error: Please run this script from the OMPL source root (where CMakeLists.txt lives)."
  exit 1
fi

echo "OMPL source root detected."

# --- config ---
JOBS="${JOBS:-10}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
INSTALL_BINDINGS="${INSTALL_BINDINGS:-1}"

usage() {
  echo "Usage: $0 [bindings|no-bindings]"
  echo
  echo "Options:"
  echo "  bindings      Build/install Python bindings (default)"
  echo "  no-bindings   Skip Python bindings installation"
  echo
  echo "Environment variables:"
  echo "  JOBS=<n>              Number of parallel build jobs (default: 10)"
  echo "  PYTHON_BIN=<path>     Python executable to use (default: python3)"
  echo "  INSTALL_BINDINGS=0/1  Whether to install bindings by default (default: 1)"
  exit 1
}

# --- parse args ---
for arg in "$@"; do
  case "$arg" in
    bindings)
      INSTALL_BINDINGS=1
      ;;
    no-bindings)
      INSTALL_BINDINGS=0
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Invalid argument: $arg"
      usage
      ;;
  esac
done

if [[ "$INSTALL_BINDINGS" -eq 1 ]]; then
  echo "Python bindings installation requested."
else
  echo "Python bindings installation skipped."
fi

# --- submodules ---
git submodule update --init --recursive

# --- configure ---
if [[ ! -d "build/Release" ]]; then
  echo "Configuring OMPL into build/Release..."
  mkdir -p build/Release
  cd build/Release
  cmake ../.. -DPython_EXECUTABLE="$(command -v "$PYTHON_BIN")"
else
  echo "build/Release already exists; skipping configuration."
  cd build/Release
fi

# --- build & install C++ OMPL ---
echo "Building OMPL..."
make -j"$JOBS"

echo "Installing OMPL C++ library (may prompt for password)..."
$SUDO make install

cd ../..

# --- optionally install Python bindings ---
if [[ "$INSTALL_BINDINGS" -eq 1 ]]; then
  echo "Installing Python bindings with $PYTHON_BIN ..."
  "$PYTHON_BIN" -m pip install ./py-bindings

  echo "Verifying Python import..."
  "$PYTHON_BIN" -c "import ompl; print('OMPL Python package:', ompl.__file__)"
fi

echo "Done."
