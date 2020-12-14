conan install . -s build_type=Debug --build missing -if out\build\x64-Debug %1 
conan install . -s build_type=Release --build missing  -if out\build\x64-Release %1