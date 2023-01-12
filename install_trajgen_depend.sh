sudo apt-get install unzip
sudo apt-get install gfortran
git clone https://github.com/arplaboratory/traj_plan_lib_dep.git
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/nlopt.git
cd traj_plan_lib_dep
unzip ma27.zip
cd ma27
./configure --build=aarch64-unknown-linux-gnu FFLAGS="-O -fPIC"
make clean
export CXXFLAGS="-O -fPIC"
make -j4
sudo make install
export MA27LIB=/usr/local/lib/libma27.a 
sudo apt-get install libblas-dev liblapack-dev
cd ..
./configure --build=aarch64-unknown-linux-gnu FFLAGS="-O -fPIC"
make clean
export CXXFLAGS="-O -fPIC"
make -j4
sudo make install
cd traj_plan_lib_dep
tar -xvzf nlopt-2.6.2.tar.gz
cd nlopt-2.6.2
mkdir build
cd build
cmake ..
make
sudo make install
cd ..
cd ..
catkin build traj_gen ros_traj_gen_utils
