cmake_minimum_required(VERSION 3.28.3)

project(STMPID_lib VERSION 0.1.0 LANGUAGES C)

add_library(PID STATIC src/PID.c)
target_include_directories(PID PUBLIC inc/)