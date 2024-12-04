@echo off

setlocal

for %%A in (%*) do (
    if "%%A"=="clean" (
        echo Performing clean rebuild...
        rmdir /s /q build
        goto endclean
    )
)

:endclean

for %%A in (%*) do (
    if "%%A"=="Release" (
        set BUILD_TYPE=Release
        goto endtype
    ) else if "%%A"=="Debug" (
        set BUILD_TYPE=Debug
        goto endtype
    )
)

set BUILD_TYPE=Release

:endtype

for %%A in (%*) do (
    if "%%A"=="clang" (
        echo Configuring for Clang %BUILD_TYPE% build...
        cmake -B build . -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -G "Ninja Multi-Config"
        goto endconfigure
    ) else if "%%A"=="web" (
        echo Configuring for WebAssembly build...
        call emcmake cmake -B build . -G "Ninja Multi-Config"
        goto endconfigure
    )
)

echo Configuring for default build...
cmake -B build . -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

:endconfigure

echo Building project...
cmake --build build --config %BUILD_TYPE% --parallel

echo BUILD_TYPE:STRING=%BUILD_TYPE%>> .\build\CMakeCache.txt

endlocal
