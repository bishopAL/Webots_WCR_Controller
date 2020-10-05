CMAKE_MINIMUM_REQUIRED(VERSION 2.8) # CMake 最低版本要求，低于2.6 构建过程会被终止。 
PROJECT(hello)              #定义工程名称
MESSAGE(STATUS "Project: hello")   # 打印相关消息消息 
MESSAGE(STATUS "Project Directory: ${PROJECT_SOURCE_DIR}")

SET(CMAKE_BUILE_TYPE DEBUG)         # 指定编译类型，debug 或者为 release
                                    # debug 版会生成相关调试信息，可以使用 GDB 进行 
                                    # release不会生成调试信息。当无法进行调试时查看此处是否设置为 debug.
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall")          # 指定编译器 
                                             # CMAKE_C_FLAGS_DEBUG            ----  C 编译器
                                             # CMAKE_CXX_FLAGS_DEBUG        ----  C++ 编译器
                                             # -g：只是编译器，在编译的时候，产生调试信息。
                                             # -Wall：生成所有警告信息。一下是具体的选项，可以单独使用

#设定可执行二进制文件的目录编译完成之后生成的.a（动态库）或者.o （静态库）文件的存储目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
    
#设定存放编译出来的库文件的目录，这里放组成工程的各个”模块“！
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

#设定头文件目录,主程序中的#include的.h文件坐在的目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)

# 添加子目录，.cpp源码的所在目录，正常一般都用src啦~
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src)
