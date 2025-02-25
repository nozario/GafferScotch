# GafferScotch

GafferScotch is a smol extension for Gaffer that adds cool functionalities not available in the default state of the software but are still useful on a daily basis!

## Installation Instructions

### Prerequisites

- CMake
- Gaffer installation

### Windows Installation

1. Clone or download this repository
2. Set the following environment variables:

   ```
   set GAFFER_ROOT=<path to your Gaffer installation>
   set GAFFERSCOTCH_INSTALL_PREFIX=<desired extension install path>
   ```

3. Use the build script to compile and install:

   ```
   .\build.bat
   ```

   This will:

   - Create the cmake-build-default directory
   - Configure the build with the correct Gaffer root
   - Compile and install the extension

### Manual Installation

If you prefer to install manually:

```
# Set environment variables
set GAFFER_ROOT=<path to your Gaffer installation>
set GAFFERSCOTCH_INSTALL_PREFIX=<desired extension install path>

# Create build directory
mkdir cmake-build-default
cd cmake-build-default

# Configure and build
cmake -DGAFFER_ROOT=%GAFFER_ROOT% -DCMAKE_INSTALL_PREFIX=%GAFFERSCOTCH_INSTALL_PREFIX% ..
cmake --build . --target install
```

## Usage

After installation, configure Gaffer to use the extension:

```
set GAFFER_EXTENSION_PATHS=%GAFFERSCOTCH_INSTALL_PREFIX%;%GAFFER_EXTENSION_PATHS%
```

To verify your installation:

```
gaffer test GafferScotchTest GafferScotchUITest
```

Then run Gaffer as normal, and enjoy the enhanced functionality provided by GafferScotch!
