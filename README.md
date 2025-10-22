For the developer
=================

Install building tools:

sudo dnf install clang clang-tools-extra
sudo dnf install clang-tools-extra 
sudo dnf install cmake

Install development libraries

sudo dnf install vtk-devel eigen3-devel spdlog-devel fmt-devel gtest-devel

Build OpenCascade with STEP support
===================================

cd ~/software/OCCT-7_8_1/build

# Clean previous build
rm -rf *

# Configure with proper DataExchange options
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/local/opencascade \
    -DBUILD_LIBRARY_TYPE=Shared \
    -DBUILD_MODULE_ApplicationFramework=OFF \
    -DBUILD_MODULE_DataExchange=ON \
    -DBUILD_MODULE_Draw=OFF \
    -DBUILD_MODULE_FoundationClasses=ON \
    -DBUILD_MODULE_ModelingAlgorithms=ON \
    -DBUILD_MODULE_ModelingData=ON \
    -DBUILD_MODULE_Visualization=OFF \
    -DUSE_TK=OFF \
    -DUSE_FREETYPE=ON \
    -DUSE_RAPIDJSON=OFF

# Build and install
make -j$(nproc)
make install