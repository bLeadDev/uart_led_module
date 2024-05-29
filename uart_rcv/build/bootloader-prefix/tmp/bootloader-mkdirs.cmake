# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/IDF/v5.2.1/esp-idf/components/bootloader/subproject"
  "C:/bLeadDev/esp_ex_uart_echo/uart_rcv/build/bootloader"
  "C:/bLeadDev/esp_ex_uart_echo/uart_rcv/build/bootloader-prefix"
  "C:/bLeadDev/esp_ex_uart_echo/uart_rcv/build/bootloader-prefix/tmp"
  "C:/bLeadDev/esp_ex_uart_echo/uart_rcv/build/bootloader-prefix/src/bootloader-stamp"
  "C:/bLeadDev/esp_ex_uart_echo/uart_rcv/build/bootloader-prefix/src"
  "C:/bLeadDev/esp_ex_uart_echo/uart_rcv/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/bLeadDev/esp_ex_uart_echo/uart_rcv/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/bLeadDev/esp_ex_uart_echo/uart_rcv/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
