file(GLOB_RECURSE UI_SOURCES ${PROJECT_SOURCE_DIR}/src/*.ui)
qt4_wrap_ui(UI_H ${UI_SOURCES})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB_RECURSE ALL_HEADERS ${PROJECT_SOURCE_DIR}/src/*.h)
set(MOC_H)
foreach(header ${ALL_HEADERS})
  file(STRINGS "${header}" lines REGEX "Q_OBJECT")
  if(lines)
    list(APPEND MOC_H "${header}")
  endif()
endforeach()
qt4_wrap_cpp(MOC_CPP ${MOC_H})

file(GLOB_RECURSE RES_QRC ${PROJECT_SOURCE_DIR}/src/*.qrc)
qt4_add_resources(RES_CPP ${RES_QRC})

add_library(roah_rsbb_qt SHARED ${UI_H} ${MOC_CPP} ${RES_CPP})

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE)
