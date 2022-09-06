set(SOURCE_FILES
    dynamics/world.cpp
    dynamics/island.cpp

    dynamics/rigidbody.cpp
    dynamics/polygon.cpp
    dynamics/circle.cpp
    dynamics/box.cpp
    dynamics/util.cpp

    collision/aabbtree.cpp
    collision/collision.cpp
    collision/simplex.cpp
    collision/polytope.cpp
    collision/broad_phase.cpp
    collision/contact_manager.cpp

    constraint/contact/contact.cpp
    constraint/contact/contact_solver.cpp
    constraint/contact/block_solver.cpp
    constraint/contact/position_solver.cpp

    constraint/joint/joint.cpp
    constraint/joint/grab_joint.cpp
    constraint/joint/revolute_joint.cpp
    constraint/joint/distance_joint.cpp
)

set(HEADER_FILES
    ../include/spe/aabb.h
    ../include/spe/aabbtree.h
    ../include/spe/broad_phase.h
    ../include/spe/contact_manager.h

    ../include/spe/collision.h
    ../include/spe/simplex.h
    ../include/spe/polytope.h
    ../include/spe/contact_point.h
    ../include/spe/edge.h

    ../include/spe/block_solver.h

    ../include/spe/rigidbody.h
    ../include/spe/box.h
    ../include/spe/circle.h
    ../include/spe/polygon.h

    ../include/spe/constraint.h

    ../include/spe/joint.h
    ../include/spe/grab_joint.h
    ../include/spe/revolute_joint.h
    ../include/spe/distance_joint.h

    ../include/spe/contact.h
    ../include/spe/position_solver.h
    ../include/spe/contact_solver.h

    ../include/spe/entity.h
    ../include/spe/island.h
    ../include/spe/world.h

    ../include/spe/settings.h
    ../include/spe/growable_array.h
    ../include/spe/common.h
    ../include/spe/util.h
    ../include/spe/spe.h
)

add_library(spe ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(spe
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
    ${DEP_INCLUDE_DIR}
)

install(
    TARGETS spe
    EXPORT speConfig
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    EXPORT speConfig
    NAMESPACE spe::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spe
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/speConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/speConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spe
)