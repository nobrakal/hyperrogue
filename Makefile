# This Makefile works for Mac OS X (El Capitan), MinGW, and Linux.
# 
# Environmental vairables:
#   If you want to build with Glew, set
#     HYPERROGUE_USE_GLEW=1
#   If you want to use libpng, set
#     HYPERROGUE_USE_PNG=1
#
# For Mac OS X:
#   Run `brew install sdl12-compat sdl_gfx sdl_mixer sdl_ttf`
#   Run `brew install glew libpng` to install the optional dependencies
#   Run `make` to build HyperRogue as `./hyperrogue`.
#
# For MSYS2 and MinGW-w64:
#   To install SDL and other required libraries, run these commands
#   from the MSYS2 shell:
#       pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-glew
#       pacman -S mingw-w64-ucrt-x86_64-SDL mingw-w64-ucrt-x86_64-SDL_mixer
#       pacman -S mingw-w64-ucrt-x86_64-SDL_ttf mingw-w64-ucrt-x86_64-SDL_gfx
#       pacman -S make
#   Then run "make" to build HyperRogue as ./hyperrogue.exe.
#
# For Ubuntu Linux:
#   Run "sudo apt-get install libsdl-dev" to install SDL in /usr/local.
#   Run "make" to build HyperRogue as ./hyperrogue.


ifeq ($(OS),Windows_NT)
  OS := mingw
  TOOLCHAIN := mingw
else
  UNAME_S := $(shell uname -s)
  ifeq ($(UNAME_S),Linux)
    OS := linux
  else
    ifeq ($(UNAME_S),Darwin)
      OS := osx
    endif
  endif
  TOOLCHAIN_VERSION_S := $(shell $(CXX) --version)
  ifneq (,$(findstring clang,$(TOOLCHAIN_VERSION_S)))
    TOOLCHAIN := clang
  else
    TOOLCHAIN := gcc
  endif
endif


## We have now finished inspecting the environment via $(shell).
## Begin customization points for each OS and TOOLCHAIN we support.


ifeq (${OS},linux)
  CXXFLAGS_EARLY += -DLINUX
  EXE_EXTENSION :=
  LDFLAGS_GL := -lGL
  LDFLAGS_GLEW := -lGLEW
  LDFLAGS_PNG := -lpng
  LDFLAGS_SDL := -lSDL -lSDL_gfx -lSDL_mixer -lSDL_ttf -lpthread -lz
  OBJ_EXTENSION := .o
  hyper_RES :=
endif

ifeq (${OS},mingw)
  CXXFLAGS_EARLY += -DWINDOWS -mwindows -D_A_VOLID=8 -I/ucrt64/include/SDL
  EXE_EXTENSION := .exe
  LDFLAGS_GL := -lopengl32
  LDFLAGS_GLEW := -lglew32
  LDFLAGS_PNG := -lpng
  LDFLAGS_SDL := -lSDL -lSDL_gfx -lSDL_mixer -lSDL_ttf -lpthread -lz
  OBJ_EXTENSION := .o
  hyper_RES := hyper.res
  ifeq (${HYPERROGUE_USE_GLEW},)
    HYPERROGUE_USE_GLEW := 1
  endif
endif

ifeq (${OS},osx)
  HOMEBREW_PREFIX := $(shell brew --prefix)
  CXXFLAGS_EARLY += -DMAC -I$(HOMEBREW_PREFIX)/include -I$(HOMEBREW_PREFIX)/include/SDL
  EXE_EXTENSION :=
  LDFLAGS_EARLY += -L$(HOMEBREW_PREFIX)/lib
  LDFLAGS_GL := -framework AppKit -framework OpenGL
  LDFLAGS_GLEW := -lGLEW
  LDFLAGS_PNG := -lpng
  LDFLAGS_SDL := -lSDL -lSDLMain -lSDL_gfx -lSDL_mixer -lSDL_ttf -lpthread -lz
  OBJ_EXTENSION := .o
  hyper_RES :=
endif

ifeq (${TOOLCHAIN},clang)
  CXXFLAGS_STD = -std=c++14
  CXXFLAGS_EARLY += -fPIC
  CXXFLAGS_EARLY += -W -Wall -Wextra -Wsuggest-override -pedantic
  CXXFLAGS_EARLY += -Wno-unused-parameter -Wno-implicit-fallthrough -Wno-maybe-uninitialized -Wno-char-subscripts -Wno-unknown-warning-option
  CXXFLAGS_EARLY += -Wno-invalid-offsetof
endif

ifeq (${TOOLCHAIN},gcc)
  CXXFLAGS_STD = -std=c++14
  CXXFLAGS_EARLY += -fPIC
  CXXFLAGS_EARLY += -W -Wall -Wextra -pedantic
  CXXFLAGS_EARLY += -Wno-unused-parameter -Wno-implicit-fallthrough -Wno-maybe-uninitialized
  CXXFLAGS_EARLY += -Wno-invalid-offsetof
endif

ifeq (${TOOLCHAIN},mingw)
  CXXFLAGS_STD = -std=c++14
  CXXFLAGS_EARLY += -W -Wall -Wextra
  CXXFLAGS_EARLY += -Wno-unused-parameter -Wno-implicit-fallthrough -Wno-maybe-uninitialized
  CXXFLAGS_EARLY += -Wno-invalid-offsetof
endif

ifeq (${FONTCONFIG},1)
  CXXFLAGS_EARLY += -DFONTCONFIG `pkg-config --cflags fontconfig`
  LDFLAGS_EARLY += `pkg-config --libs fontconfig`
endif

## We have now finished OS-specific and TOOLCHAIN-specific computations.
## Begin customization points for user-specifiable HYPERROGUE_USE_XXX macros.


hyper_OBJS = hyper$(OBJ_EXTENSION)
hyper_LDFLAGS = $(LDFLAGS_GL) $(LDFLAGS_SDL)

ifeq (${HYPERROGUE_USE_GLEW},1)
  CXXFLAGS_EARLY += -DCAP_GLEW=1
  hyper_LDFLAGS += $(LDFLAGS_GLEW)
else
  CXXFLAGS_EARLY += -DCAP_GLEW=0
endif

ifeq (${HYPERROGUE_USE_PNG},1)
  CXXFLAGS_EARLY += -DCAP_PNG=1
  hyper_LDFLAGS += $(LDFLAGS_PNG)
  hyper_OBJS += savepng$(OBJ_EXTENSION)
else
  CXXFLAGS_EARLY += -DCAP_PNG=0
endif

ifeq (${HYPERROGUE_USE_ROGUEVIZ},1)
  # Enable RogueViz. RogueViz requires C++17.
  CXXFLAGS_STD = -std=c++17
  CXXFLAGS_EARLY += -DCAP_ROGUEVIZ=1
endif


## We have now finished HYPERROGUE_USE_XXX-specific computations.
## Begin the Makefile proper.


override CXXFLAGS := $(CXXFLAGS_STD) $(CXXFLAGS_EARLY) $(CXXFLAGS) ${EXTRA_CXXFLAGS}
override LDFLAGS := $(LDFLAGS_EARLY) $(LDFLAGS) ${EXTRA_LDFLAGS}

hyperrogue$(EXE_EXTENSION): $(hyper_OBJS) $(hyper_RES)
	$(CXX) $(CXXFLAGS) $(hyper_OBJS) $(hyper_RES) $(LDFLAGS) $(hyper_LDFLAGS) -o $@

hyper$(OBJ_EXTENSION): *.cpp language-data.cpp autohdr.h
	$(CXX) -O2 $(CXXFLAGS) $(hyper_CXXFLAGS) -c hyper.cpp -o $@

hyper.res: hyper.rc hr-icon.ico
	windres hyper.rc -O coff -o hyper.res

langen$(EXE_EXTENSION): langen.cpp language-??.cpp language-ptbr.cpp all-string-literals.ipp
	$(CXX) -O0 $(CXXFLAGS) $(langen_CXXFLAGS) langen.cpp $(LDFLAGS) -o $@

makeh$(EXE_EXTENSION): makeh.cpp
	$(CXX) -O2 makeh.cpp -o $@

autohdr.h: makeh$(EXE_EXTENSION) language-data.cpp *.cpp
	./makeh classes.cpp locations.cpp colors.cpp hyperpoint.cpp geometry.cpp embeddings.cpp goldberg.cpp init.cpp floorshapes.cpp cell.cpp multi.cpp shmup.cpp pattern2.cpp mapeditor.cpp graph.cpp textures.cpp hprint.cpp language.cpp util.cpp complex.cpp multigame.cpp arbitrile.cpp rulegen.cpp *.cpp > autohdr.h

all-string-literals.ipp: lanlint.py *.cpp
	python lanlint.py > all-string-literals.ipp

language-data.cpp: langen$(EXE_EXTENSION)
	./langen > language-data.cpp

savepng$(OBJ_EXTENSION): savepng.cpp
	$(CXX) -O2 $(CXXFLAGS) -c savepng.cpp -o $@

mymake$(EXE_EXTENSION): mymake.cpp
	$(CXX) -O2 $(CXXFLAGS) mymake.cpp -pthread -o $@

emscripten: hyper.html

%.html %.js %.wasm: %.emscripten-sources
	emcc -std=c++14 -O3 -s USE_ZLIB=1 -s LEGACY_GL_EMULATION=1 -s TOTAL_MEMORY=128MB hyperweb.cpp -o hyper.html

hyper.emscripten-sources: *.cpp autohdr.h

.PHONY: clean

clean:
	rm -f langen$(EXE_EXTENSION) language-data.cpp
	rm -f makeh$(EXE_EXTENSION) autohdr.h
	rm -rf mymake$(EXE_EXTENSION) mymake_files/
	rm -f hyperrogue$(EXE_EXTENSION) hyper$(OBJ_EXTENSION) $(hyper_RES) savepng$(OBJ_EXTENSION)
	rm -f hyper.html hyper.js hyper.wasm
	rm -f all-string-literals.ipp
