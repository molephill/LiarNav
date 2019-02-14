
CPPFLAG = -Wall
LFLAGS =
# args
RELEASE = 1
BITS =

# 生成lib后移动到的目录
LIB_DIR = ./../../../ebin/

# 平台 0代表win,1代表其他
PLATFORM = 0

# 生成类型 1代表动态,0代表静态
STATIC = 1

# [args] 生成模式. 0代表debug模式, 1代表release模式. make RELEASE=1.
ifeq ($(RELEASE),0)
    # debug
    CPPFLAG += -g -static
else
    # release
    CPPFLAG += -static -O3 -DNDEBUG
    LFLAGS += -static
endif

# [args] 程序位数. 32代表32位程序, 64代表64位程序, 其他默认. make BITS=32.
ifeq ($(BITS),32)
    CPPFLAG += -m32
    LFLAGS += -m32
else
    ifeq ($(BITS),64)
        CPPFLAG += -m64
        LFLAGS += -m64
    else
    endif
endif

#CPPFLAG = -Wall -g -std=c++11

OBJDIR=./obj

#DeBugBase.o: DeBugBase.cpp
#	$(CC) -c DeBugBase.cpp

#编译包含的头文件所在目录  
INCLUDES	:= -I 

ifeq ($(PLATFORM),0)
	CC	:= g++
	INCLUDES += d:/inittools/erl8.3/erts-8.3/include
else
	CC	:= g++720
	INCLUDES += /usr/local/lib/erlang/erts-8.3/include
endif

#所有用到的源文件，注意：非当前目录的要+上详细地址  
# SRCS	:= main.cpp \
# 			NifMap.cpp Map.cpp Polygon.cpp NavMesh.cpp Vector2f.cpp Heap.cpp WayPoint.cpp Cell.cpp Triangle.cpp Line2d.cpp \
# 			GeomPrint.cpp Delaunay.cpp nifmaps.cpp

ifeq ($(STATIC),1)
SRCS	:=  nifmaps.cpp NifMap.cpp Map.cpp Polygon.cpp NavMesh.cpp Vector2f.cpp Heap.cpp WayPoint.cpp Cell.cpp Triangle.cpp Line2d.cpp \
			MapSource.cpp Delaunay.cpp
else
SRCS	:=  NifMap.cpp Map.cpp Polygon.cpp NavMesh.cpp Vector2f.cpp Heap.cpp WayPoint.cpp Cell.cpp Triangle.cpp Line2d.cpp \
			MapSource.cpp Delaunay.cpp
endif
#把源文件SRCS字符串的后缀.c改为.o   
OBJS	:= $(SRCS:.cpp=.o)
OBJS1	:= $(SRCS:.cpp=.obj)
#目标
ifeq ($(PLATFORM),0)
	TARGET = libNifmaps.dll
else
	TARGET = libNifmaps.so
endif


ifeq ($(PLATFORM),0)
%.o:%.cpp
	$(CC) $(CPPFLAG) -c $(SRCS) $(INCLUDES)
else
%.o:%.cpp
	$(CC) -fPIC $(CPPFLAG) $(INCLUDES) -c $< -o $@
endif

#clean:	
	#rm -rf $(basename $(TARGET)) $(SRCS:.cpp=.o)

# main.exe:$(OBJS)
# 	$(CC) $(CPPFLAG) -o main.exe $(OBJS)

# %.out : %.cpp
# 	$(CC) $(INCLUDES) $< -g -c $@

ifeq ($(STATIC),1)
    ifeq ($(PLATFORM),0)
        $(TARGET):$(OBJS)
			rm -f $@
			$(CC) $(CPPFLAG) -shared -o -fPIC -o $@ $^
			rm -f $(OBJS)
			mv $(TARGET) $(LIB_DIR)
    else
        $(TARGET):$(OBJS)
			rm -f $@
			$(CC) -fPIC -shared -o $@ $(OBJS)
			rm -f $(OBJS)
			mv $(TARGET) $(LIB_DIR)
    endif
else
	ifeq ($(PLATFORM),0)
		nifmaps.lib:$(OBJS)
			ar rcs -o $@ $^
	else
		nifmaps.a:$(OBJS)
			ar rcs -o $@ $^
	endif
endif

# ifeq ($(PLATFORM),0)
# 	nifmaps.dll:$(OBJS)
# 		$(CC) $(CPPFLAG) -shared -o -fPIC -o $@ $^
# else
# 	nifmaps.so:$(OBJS)
# 		$(CC) $(CPPFLAG) -shared -o -fPIC -o $@ $^
# endif

# $(OBJS):%.o : %.cpp  
#     $(CC) $(CFLAGS) $< -o $@ $(INCLUDES) 


# nifmaps.o: nifmaps.cpp
# 	$(CC) -c $(CPPFLAG) nifmaps.cpp $(INCLUDES)
# main.o: main.cpp
# 	$(CC) -c $(CPPFLAG) main.cpp $(INCLUDES)
# GeomPrint.o: GeomPrint.cpp
# 	$(CC) -c $(CPPFLAG) GeomPrint.cpp $(INCLUDES)
# NifMap.o: NifMap.cpp
# 	$(CC) -c $(CPPFLAG) NifMap.cpp $(INCLUDES)
# Map.o: Map.cpp
# 	$(CC) -c $(CPPFLAG) Map.cpp $(INCLUDES)
# Polygon.o: Polygon.cpp
# 	$(CC) -c $(CPPFLAG) Polygon.cpp $(INCLUDES)
# NavMesh.o: NavMesh.cpp
# 	$(CC) -c $(CPPFLAG) NavMesh.cpp $(INCLUDES)
# Vector2f.o: Vector2f.cpp
# 	$(CC) -c $(CPPFLAG) Vector2f.cpp $(INCLUDES)
# Heap.o: Heap.cpp
# 	$(CC) -c $(CPPFLAG) Heap.cpp $(INCLUDES)
# Delaunay.o: Delaunay.cpp
# 	$(CC) -c $(CPPFLAG) Delaunay.cpp $(INCLUDES)
# WayPoint.o: WayPoint.cpp
# 	$(CC) -c $(CPPFLAG) WayPoint.cpp $(INCLUDES)
# Cell.o: Cell.cpp
# 	$(CC) -c $(CPPFLAG) Cell.cpp $(INCLUDES)
# Triangle.o: Triangle.cpp
# 	$(CC) -c $(CPPFLAG) Triangle.cpp $(INCLUDES)
# Line2d.o: Line2d.cpp
# 	$(CC) -c $(CPPFLAG) Line2d.cpp $(INCLUDES)

clean:
	rm -f $(OBJS)

cleandll:
	rm -f *.dll
