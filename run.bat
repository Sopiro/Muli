@echo off
setlocal

set CACHE_FILE=".\build\CMakeCache.txt"

if not exist "%CACHE_FILE%" (
    echo Could not find CMake cache
    exit /b 1
)

for /f "tokens=2 delims==" %%a in ('findstr /b "BUILD_TYPE:STRING=" %CACHE_FILE%') do (
    set BUILD_TYPE=%%a
)

findstr "Emscripten.cmake" %CACHE_FILE%

if %errorlevel%==0 (
    emrun .\build\bin\%BUILD_TYPE%\demo.html
) else (
    .\build\bin\%BUILD_TYPE%\demo
)

endlocal