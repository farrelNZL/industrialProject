# Ajustez en fonction de votre version de CMake

cmake_minimum_required (VERSION 2.8)


# Nom du projet

project (TELLUS)

find_package (OpenCV REQUIRED)

#ADD_EXECUTABLE(calib_openCV src/calib_openCV.cpp)
ADD_EXECUTABLE(calib src/calib.cpp)
ADD_EXECUTABLE(projet src/projet.cpp)
ADD_EXECUTABLE(testdecompo src/testdecompo.cpp)
# link avec les bibliothèques d'OpenCV

target_link_libraries(calib ${OpenCV_LIBS} )
target_link_libraries(projet ${OpenCV_LIBS} )
target_link_libraries(testdecompo ${OpenCV_LIBS} )
#target_link_libraries(calib_openCV ${OpenCV_LIBS} )

