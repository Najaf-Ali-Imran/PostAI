#!/bin/bash
# PostAI WebAssembly Build Script
# Requirements: Emscripten SDK (emsdk) must be installed and activated
# Install: https://emscripten.org/docs/getting_started/downloads.html

set -e

echo "=== PostAI WASM Build ==="
echo ""

# Check if emcmake is available
if ! command -v emcmake &> /dev/null; then
    echo "ERROR: Emscripten not found!"
    echo ""
    echo "Install Emscripten SDK:"
    echo "  git clone https://github.com/emscripten-core/emsdk.git"
    echo "  cd emsdk"
    echo "  ./emsdk install latest"
    echo "  ./emsdk activate latest"
    echo "  source ./emsdk_env.sh"
    echo ""
    exit 1
fi

# Create build directory
mkdir -p build_wasm
cd build_wasm

# Configure with Emscripten
echo "[1/2] Configuring CMake with Emscripten..."
emcmake cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
echo "[2/2] Building WebAssembly..."
emmake make -j$(nproc 2>/dev/null || echo 4)

# Copy output files to project root
echo ""
echo "Copying WASM files to project root..."
cp postai.js ../postai.js
cp postai.wasm ../postai.wasm

echo ""
echo "=== Build Complete! ==="
echo ""
echo "Output files:"
echo "  postai.js   - JavaScript loader"
echo "  postai.wasm - WebAssembly binary"
echo ""
echo "To test locally:"
echo "  cd .."
echo "  python -m http.server 8000"
echo "  Open http://localhost:8000/app.html"
echo ""
echo "To deploy to Vercel/Netlify:"
echo "  Upload these files: index.html, app.html, documentation.html,"
echo "  postai.js, postai.wasm, assets/"
echo ""
