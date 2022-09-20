set(SOURCE_FILES
    common/util.cpp
    common/math.cpp

    dynamics/world.cpp
    dynamics/island.cpp

    dynamics/rigidbody.cpp
    dynamics/circle.cpp
    dynamics/polygon.cpp
    dynamics/box.cpp
    dynamics/capsule.cpp

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
    ../include/muli/aabb.h
    ../include/muli/aabbtree.h
    ../include/muli/broad_phase.h
    ../include/muli/contact_manager.h

    ../include/muli/collision.h
    ../include/muli/simplex.h
    ../include/muli/polytope.h
    ../include/muli/contact_point.h
    ../include/muli/edge.h

    ../include/muli/block_solver.h

    ../include/muli/rigidbody.h
    ../include/muli/box.h
    ../include/muli/circle.h
    ../include/muli/polygon.h
    ../include/muli/capsule.h

    ../include/muli/constraint.h

    ../include/muli/joint.h
    ../include/muli/grab_joint.h
    ../include/muli/revolute_joint.h
    ../include/muli/distance_joint.h

    ../include/muli/contact.h
    ../include/muli/position_solver.h
    ../include/muli/contact_solver.h

    ../include/muli/island.h
    ../include/muli/world.h

    ../include/muli/settings.h
    ../include/muli/growable_array.h
    ../include/muli/common.h
    ../include/muli/util.h
    ../include/muli/muli.h
    ../include/muli/math.h
    ../include/muli/types.h
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${HEADER_FILES})

add_library(muli ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(muli
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
    ${DEP_INCLUDE_DIR}
)

install(
    TARGETS muli
    EXPORT muliConfig
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    EXPORT muliConfig
    NAMESPACE muli::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/muli
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/muliConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/muliConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/muli
)