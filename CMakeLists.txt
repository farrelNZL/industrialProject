# Ajustez en fonction de votre version de CMake

cmake_minimum_required (VERSION 2.8)


# Nom du projet

project (TELLUS)

find_package (OpenCV REQUIRED)


ADD_EXECUTABLE(exe src/exe.cpp)
ADD_EXECUTABLE(calib src/calib.cpp)

# link avec les bibliothèques d'OpenCV

target_link_libraries(exe ${OpenCV_LIBS} )
target_link_libraries(calib ${OpenCV_LIBS} )
