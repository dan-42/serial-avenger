#------------------------------------
# Global Vars
#------------------------------------


#------------------------------------
# Dependencies
#------------------------------------
# Boost
set(BOOST_ALL_DYN_LINK ON)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)    
find_package(Boost 1.55 COMPONENTS  system REQUIRED )
link_directories (${Boost_LIBRARY_DIRS})
include_directories (${Boost_INCLUDE_DIRS})
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

# Threads
find_package(Threads)

if(WIN32)
    find_library(PSAPI_LIBRARIES psapi)
endif()

#------------------------------------
# Project Setup
#------------------------------------
#### Project Files

include_directories(${CMAKE_CURRENT_LIST_DIR} )

file(GLOB_RECURSE SOURCES "*.cpp")


#### Project Build Targets
add_executable(serial-tester  ${SOURCES} )


#### Linking
target_link_libraries(serial-tester ${Boost_LIBRARIES} ${PSAPI_LIBRARIES} pthread)


	
if(WIN32)
  target_link_libraries(main wsock32 ws2_32)
endif()

#------------------------------------
# Installation
#------------------------------------


INSTALL(TARGETS serial-tester 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

