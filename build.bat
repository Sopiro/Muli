rmdir /s /q build
mkdir build
cd build
cmake ..
cmake --build . --config Release
start muli.sln