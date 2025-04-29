#! /bin/bash
rm -r ~/flair/flair-install/bin/demos/core2-64/uavcoop
rm -r ~/flair/flair-build/build/demos/uavcoop
mkdir -p $FLAIR_ROOT/flair-build/build/demos/uavcoop
cd $FLAIR_ROOT/flair-build/build/demos/uavcoop
$FLAIR_ROOT/flair-src/scripts/cmake_codelite_outofsource.sh $FLAIR_ROOT/my_src/uavcoop
cd build
make install -j15

rm -r ~/flair/flair-install/bin/demos/armv7a-neon/uavcoop
rm -r ~/flair/flair-build/build_armv7a_neon/demos/uavcoop
mkdir -p $FLAIR_ROOT/flair-build/build_armv7a_neon/demos/uavcoop
cd $FLAIR_ROOT/flair-build/build_armv7a_neon/demos/uavcoop
$FLAIR_ROOT/flair-src/scripts/cmake_codelite_outofsource.sh $FLAIR_ROOT/my_src/uavcoop
cd build_armv7a_neon
make install -j15

cd ~/flair/my_src/uavcoop
flairrun uavcoop
