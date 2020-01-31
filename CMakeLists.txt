Project(NASAJPLTensegrity)

link_libraries(tgcreator core sensors)

add_executable(AppNASA_JPL_Tensegrity
    T6Model.cpp
    AppSUPERball.cpp
)
