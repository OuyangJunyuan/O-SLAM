aux_source_directory(./oslam_typedef   oslam_lib_src)
add_library(oslam SHARED
       ${oslam_lib_src})
target_link_libraries(oslam )

############################################################################################
##################################### Generate Ros node  ###################################
############################################################################################
aux_source_directory(.                     oslam_src)

add_executable(${PROJECT_NAME}_node ${oslam_src})
target_link_libraries(${PROJECT_NAME}_node
        oslam ${ALL_LIBS}
        )