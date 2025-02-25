cmake_minimum_required(VERSION 3.0)
project(GafferScotch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE OFF)

# Add MSVC specific flags
if(MSVC)
    add_compile_options(/W4 /EHsc)
endif()

if(APPLE)
  set(CMAKE_SHARED_LIBRARY_SUFFIX ".so")
endif(APPLE)

# build the library
file( GLOB GafferScotchSrc src/GafferScotch/*.cpp )
link_directories( GafferScotch ${GAFFER_ROOT}/lib ${GAFFER_ROOT}/libs )
add_library( GafferScotch SHARED ${GafferScotchSrc} )
target_compile_definitions( GafferScotch PRIVATE 
    BOOST_SIGNALS_NO_DEPRECATION_WARNING=1 
    -D_GLIBCXX_USE_CXX11_ABI=0
    GafferScotch_EXPORTS
)
target_include_directories( GafferScotch PRIVATE 
    include 
    ${GAFFER_ROOT}/include
    ${GAFFER_ROOT}/include/Imath
    ${GAFFER_ROOT}/include/OpenEXR
    ${GAFFER_ROOT}/include/boost
    ${GAFFER_ROOT}/include/bin
    ${GAFFER_ROOT}/include/tbb
)
target_link_libraries( GafferScotch Gaffer GafferScene IECore IECoreScene fmt)
install( TARGETS GafferScotch DESTINATION lib)
string(REPLACE "." "" PYTHON_VERSION_STRIPPED ${PYTHON_VERSION})
# Convert Python version format from "3.10" to "310" for library names
string(REGEX REPLACE "([0-9]+)[.]([0-9]+)" "\\1\\2" PYTHON_LIB_VERSION ${PYTHON_VERSION})

# build the python bindings
file( GLOB GafferScotchModuleSrc src/GafferScotchModule/*.cpp )
add_library( GafferScotchModule SHARED ${GafferScotchModuleSrc} )

target_compile_definitions( GafferScotchModule PRIVATE BOOST_SIGNALS_NO_DEPRECATION_WARNING=1 -D_GLIBCXX_USE_CXX11_ABI=0 BOOST_ALL_NO_LIB=1)
target_include_directories( GafferScotchModule PRIVATE 
    include 
    ${GAFFER_ROOT}/include
    ${GAFFER_ROOT}/include/boost
    ${GAFFER_ROOT}/include/Imath
)

if(MSVC)
    target_link_libraries( GafferScotchModule 
        GafferScotch 
        GafferBindings 
        IECorePython 
        boost_python${PYTHON_VERSION_STRIPPED} 
        python${PYTHON_LIB_VERSION}
        Gaffer 
        GafferScene 
        IECore 
        IECoreScene
    )
else()
    target_link_libraries( GafferScotchModule 
        GafferScotch 
        GafferBindings 
        IECorePython 
        boost_python${PYTHON_VERSION_STRIPPED} 
        python${PYTHON_LIB_VERSION}
        Gaffer 
        GafferScene 
        IECore 
        IECoreScene
    )
endif()

# a possible modification: rename GafferScotchModule.dll to .pyd on Windows
if(WIN32)
    set_target_properties(GafferScotchModule PROPERTIES SUFFIX ".pyd")
endif()

# Install the Python module and its dependencies
install(TARGETS GafferScotchModule DESTINATION python/GafferScotch)

# Install Python modules
file(GLOB GafferScotchModuleFiles python/GafferScotch/*.py)
install(FILES ${GafferScotchModuleFiles} DESTINATION python/GafferScotch)
file(GLOB GafferScotchTestModuleFiles python/GafferScotchTest/*.py)
install(FILES ${GafferScotchTestModuleFiles} DESTINATION python/GafferScotchTest)
file(GLOB GafferScotchUIModuleFiles python/GafferScotchUI/*.py)
install(FILES ${GafferScotchUIModuleFiles} DESTINATION python/GafferScotchUI)
file(GLOB GafferScotchUITestModuleFiles python/GafferScotchUITest/*.py)
install(FILES ${GafferScotchUITestModuleFiles} DESTINATION python/GafferScotchUITest)

# Install startup configs
install(DIRECTORY startup DESTINATION . FILES_MATCHING PATTERN "*.py")

# Install resources
install(DIRECTORY resources DESTINATION . FILES_MATCHING PATTERN "*.gfr" PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)


