cd build
cmake ..
cmake --build .
cmake --install . --config Debug
cmake --install . --config Release
cpack -G ZIP -C Debug
cpack -G ZIP -C Release
