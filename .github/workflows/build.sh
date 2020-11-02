set -x -e o pipefail


GH_MYMAKE_ARGS="-fPIC"
GH_AUTOTOOLS_CXXFLAGS="-W -Wall -Wextra -Wno-unused-parameter -Wno-maybe-uninitialized"

HYPERROGUE_USE_GLEW=$GH_HYP_GLEW
export HYPERROGUE_USE_GLEW=${HYPERROGUE_USE_GLEW: -1}

HYPERROGUE_USE_PNG=$GH_HYP_PNG
export HYPERROGUE_USE_PNG=${HYPERROGUE_USE_PNG: -1}

HYPERROGUE_USE_ROGUEVIZ=$GH_HYP_RVIZ
export HYPERROGUE_USE_ROGUEVIZ=${HYPERROGUE_USE_ROGUEVIZ: -1}
if [[ "$GH_HYP_RVIZ" == "rviz_1" ]]; then
  GH_MYMAKE_ARGS+=" -std=c++17 -rv"
  GH_AUTOTOOLS_CXXFLAGS+=" -std=c++17 -DCAP_ROGUEVIZ=1"
fi

if [[ "$GH_HYP_BRINGRIS" == "bringris_1" ]]; then
  GH_MYMAKE_ARGS+=" -bringris"
fi

export CC=$GH_COMPILER
export CXX=${CC%cc}++

if [[ "$GH_BUILDSYS" == "makefile" ]]; then
  if [[ "$GH_HYP_BRINGRIS" == "bringris_1" ]]; then
    make -f Makefile.simple bringris
    mv bringris hyperrogue
  else
    make -f Makefile.simple
  fi
elif [[ "$GH_BUILDSYS" == "autotools" ]]; then
  autoreconf -vfi
  ./configure CXXFLAGS="${GH_AUTOTOOLS_CXXFLAGS}"
  make
elif [[ "$GH_BUILDSYS" == "mymake" ]]; then
  make -f Makefile.simple mymake
  ./mymake $GH_MYMAKE_ARGS
  mv hyper hyperrogue
else
  echo 'unknown build system'
  exit 1
fi
