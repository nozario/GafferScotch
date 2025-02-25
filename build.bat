@echo off
setlocal

REM Set environment variables
set "GAFFER_ROOT=C:\Program Files\Gaffer\1.5.6.0"
set "GAFFERSCOTCH_INSTALL_PREFIX=C:\Users\Lucas\ActiveWork\Dev\GafferScotch"
set "PYTHONPATH=%GAFFERSCOTCH_INSTALL_PREFIX%\python;%PYTHONPATH%"
set "PATH=%GAFFERSCOTCH_INSTALL_PREFIX%\lib;%PATH%"

echo Setting environment variables:
echo GAFFER_ROOT=%GAFFER_ROOT%
echo GAFFERSCOTCH_INSTALL_PREFIX=%GAFFERSCOTCH_INSTALL_PREFIX%
echo PYTHONPATH=%PYTHONPATH%

REM Check if GAFFER_ROOT exists
if not exist "%GAFFER_ROOT%" (
    echo Error: Gaffer installation not found at %GAFFER_ROOT%
    exit /b 1
)

REM Remove old build directory if it exists
if exist cmake-build-default (
    echo Cleaning old build directory...
    rmdir /s /q cmake-build-default
)

REM Create new build directory
echo Creating new build directory...
mkdir cmake-build-default
cd cmake-build-default

REM Configure with CMake
echo Configuring with CMake...
cmake -DGAFFER_ROOT="%GAFFER_ROOT%" -DCMAKE_INSTALL_PREFIX="%GAFFERSCOTCH_INSTALL_PREFIX%" -DPYTHON_VERSION=3.10 ..
if errorlevel 1 (
    echo CMake configuration failed
    exit /b 1
)

REM Build and install
echo Building and installing...
cmake --build . --config Release --target install
if errorlevel 1 (
    echo Build failed
    exit /b 1
)

REM Copy the DLL to where Gaffer expects it
echo Copying module files...
xcopy /y /i Release\python\GafferScotch\*.* "%GAFFERSCOTCH_INSTALL_PREFIX%\python\GafferScotch\"

echo Build completed successfully!
cd .. 