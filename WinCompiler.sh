#!/usr/bin/env bash
set -e

# ----------------------------
# Configuration
# ----------------------------
PROJECT_NAME="posgen"
BUILD_WIN_DIR="build-win"
TARGET_ARCH="x86_64-w64-mingw32"   # Windows 64-bit
TOOLCHAIN_FILE="$BUILD_WIN_DIR/mingw-toolchain.cmake"

# ----------------------------
# Check for MinGW
# ----------------------------
if ! command -v ${TARGET_ARCH}-g++ &> /dev/null; then
    echo "❌ MinGW not found! Please install it first:"
    echo "   sudo apt install mingw-w64"
    exit 1
fi

echo "✅ MinGW detected: $(${TARGET_ARCH}-g++ --version | head -n 1)"

# ----------------------------
# Prepare build directory
# ----------------------------
mkdir -p "$BUILD_WIN_DIR"

# ----------------------------
# Generate CMake toolchain file
# ----------------------------
cat > "$TOOLCHAIN_FILE" <<EOF
# Auto-generated MinGW toolchain file
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(CMAKE_C_COMPILER ${TARGET_ARCH}-gcc)
set(CMAKE_CXX_COMPILER ${TARGET_ARCH}-g++)
set(CMAKE_RC_COMPILER ${TARGET_ARCH}-windres)

set(CMAKE_FIND_ROOT_PATH /usr/${TARGET_ARCH})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
EOF

echo "🛠️  Toolchain file created at: $TOOLCHAIN_FILE"

# ----------------------------
# Configure and build with static linking
# ----------------------------
cmake -B "$BUILD_WIN_DIR" -S . \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXE_LINKER_FLAGS="-static-libgcc -static-libstdc++ -static"

cmake --build "$BUILD_WIN_DIR" --parallel

echo "✅ Windows build complete: $BUILD_WIN_DIR/${PROJECT_NAME}.exe"
echo "🎉 Fully static Windows executable is ready. No DLLs needed!"
