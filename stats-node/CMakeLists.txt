cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS ../zephyr)
project(zmk)

target_sources(app PRIVATE src/main.c src/matthew.c src/snow.c)
target_include_directories(app PRIVATE "~/projects/modules/lib/gui/lvgl")

zephyr_cc_option(-Wfatal-errors)
