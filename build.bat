rmdir /s /q build
mkdir build
pushd build
cmake ..
cmake --build .
popd
start build/muli.sln