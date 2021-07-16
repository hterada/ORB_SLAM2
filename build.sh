echo "Configuring and building Thirdparty/g2o ..."

pushd Thirdparty/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
popd


echo "Uncompress vocabulary ..."

pushd Vocabulary
tar -xf ORBvoc.txt.tar.gz
popd

echo "Configuring and building ORB_SLAM2 ..."

pushd build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
popd

