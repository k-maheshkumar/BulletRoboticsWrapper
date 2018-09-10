#MESSAGE("Looking for Bullet")

#IF (Bullet_INCLUDE_DIR AND Bullet_LIBRARY)
#  SET(Bullet_FIND_QUIETLY TRUE)
#ENDIF (Bullet_INCLUDE_DIR AND Bullet_LIBRARY)

#FIND_PACKAGE(PkgConfig)
#PKG_SEARCH_MODULE(Bullet bullet)
#SET(Bullet_INCLUDE_DIR ${Bullet_INCLUDE_DIRS})
#SET(Bullet_LIB_DIR ${Bullet_LIBDIR})
#SET(Bullet_LIBRARIES ${Bullet_LIBRARIES} CACHE STRING "")

#SEPARATE_ARGUMENTS(Bullet_INCLUDE_DIR)
#SEPARATE_ARGUMENTS(Bullet_LIBRARIES)

#SET(Bullet_INCLUDE_DIR ${Bullet_INCLUDE_DIR} CACHE PATH "")
#SET(Bullet_LIBRARIES ${Bullet_LIBRARIES} CACHE STRING "")
#SET(Bullet_LIB_DIR ${Bullet_LIB_DIR} CACHE PATH "")

#IF (Bullet_INCLUDE_DIR AND Bullet_LIBRARIES)
#    SET(Bullet_FOUND TRUE)
#ENDIF (Bullet_INCLUDE_DIR AND Bullet_LIBRARIES)

#IF (Bullet_FOUND)
#    IF (NOT Bullet_FIND_QUIETLY)
#        MESSAGE(STATUS "  libraries : ${Bullet_LIBRARIES} from ${Bullet_LIB_DIR}")
#        MESSAGE(STATUS "  includes  : ${Bullet_INCLUDE_DIR}")
#    ENDIF (NOT Bullet_FIND_QUIETLY)
#ELSE (Bullet_FOUND)
#    IF (Bullet_FIND_REQUIRED)
#        MESSAGE(FATAL_ERROR "Could not find Bullet")
#    ENDIF (Bullet_FIND_REQUIRED)
#ENDIF (Bullet_FOUND)


# - Find Bullet includes and library
#
# This module defines
#  BULLET_INCLUDE_DIR
#  BULLET_LIBRARIES, the libraries to link against to use Bullet.
#  BULLET_LIB_DIR, the location of the libraries
#  BULLET_FOUND, If false, do not try to use Bullet
#
# Copyright &Acirc;&copy; 2007, Matt Williams
# Changes for Bullet detection by Garvek, 2009
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

IF (BULLET_LIBRARIES AND BULLET_INCLUDE_DIR)
    SET(BULLET_FIND_QUIETLY TRUE) # Already in cache, be silent
ENDIF (BULLET_LIBRARIES AND BULLET_INCLUDE_DIR)

IF (WIN32) #Windows
    MESSAGE(STATUS "Looking for Bullet")
    SET(BULLET_SDK $ENV{BULLET_HOME})
    IF (BULLET_SDK)
        MESSAGE(STATUS "Using Bullet SDK")
        STRING(REGEX REPLACE "[\]" "/" BULLET_SDK "${BULLET_SDK}")
        SET(BULLET_INCLUDE_DIR ${BULLET_SDK}/src)
        SET(BULLET_LIB_DIR ${BULLET_SDK}/lib)
        SET(BULLET_LIBRARIES debug BulletCollision BulletDynamics LinearMath optimized BulletCollision BulletDynamics LinearMath)
    ENDIF (BULLET_SDK)
ELSE (WIN32) #Unix
    message("---------ffvdgsdgs--------")
    # No .pc for this one
    FIND_PATH(BULLET_INCLUDE_DIR btBulletDynamicsCommon.h PATHS /usr/local/include /usr/include)

    # FIXME: ensure we find /usr/lib/libBulletDynamics.a /usr/lib/libBulletCollision.a /usr/lib/libLinearMath.a
    SET(BULLET_NAMES ${BULLET_NAMES} BulletCollision BulletDynamics LinearMath)
    FIND_LIBRARY(BULLET_LIBRARY
      NAMES ${BULLET_NAMES}
      PATHS /usr/lib /usr/local/lib)

    IF (BULLET_LIBRARY)
        SET(BULLET_LIBRARIES ${BULLET_LIBRARY})
        SET(BULLET_FOUND "YES")
    ELSE (BULLET_LIBRARY)
        SET(BULLET_FOUND "NO")
    ENDIF (BULLET_LIBRARY)
ENDIF (WIN32)

#Do some preparation
SEPARATE_ARGUMENTS(BULLET_INCLUDE_DIR)
SEPARATE_ARGUMENTS(BULLET_LIBRARIES)

SET(BULLET_INCLUDE_DIR ${BULLET_INCLUDE_DIR} CACHE PATH "")
SET(BULLET_LIBRARIES ${BULLET_LIBRARIES} CACHE STRING "")
SET(BULLET_LIB_DIR ${BULLET_LIB_DIR} CACHE PATH "")

IF (BULLET_INCLUDE_DIR AND BULLET_LIBRARIES)
    SET(BULLET_FOUND TRUE)
ENDIF (BULLET_INCLUDE_DIR AND BULLET_LIBRARIES)

set(BULLET_FIND_QUIETLY false)
IF (BULLET_FOUND)
    message("---------efefe--------")

    IF (NOT BULLET_FIND_QUIETLY)
        MESSAGE(STATUS "  libraries : ${BULLET_LIBRARIES} from ${BULLET_LIB_DIR}")
        MESSAGE(STATUS "  includes  : ${BULLET_INCLUDE_DIR}")
    ENDIF (NOT BULLET_FIND_QUIETLY)
ELSE (BULLET_FOUND)
    IF (BULLET_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find Newton")
    ENDIF (BULLET_FIND_REQUIRED)
ENDIF (BULLET_FOUND)
