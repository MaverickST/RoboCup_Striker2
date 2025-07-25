# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/esp-idf-v5.2.2/components/bootloader/subproject"
  "E:/esp/RoboCup/RoboCup_Striker2/APDS9960-driver/build/bootloader"
  "E:/esp/RoboCup/RoboCup_Striker2/APDS9960-driver/build/bootloader-prefix"
  "E:/esp/RoboCup/RoboCup_Striker2/APDS9960-driver/build/bootloader-prefix/tmp"
  "E:/esp/RoboCup/RoboCup_Striker2/APDS9960-driver/build/bootloader-prefix/src/bootloader-stamp"
  "E:/esp/RoboCup/RoboCup_Striker2/APDS9960-driver/build/bootloader-prefix/src"
  "E:/esp/RoboCup/RoboCup_Striker2/APDS9960-driver/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/esp/RoboCup/RoboCup_Striker2/APDS9960-driver/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/esp/RoboCup/RoboCup_Striker2/APDS9960-driver/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
