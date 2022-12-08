# mirBSA

Follow the steps to compile and execute 

```bash
  git clone git@github.com:HacB45/mirBSA.git
  cd mirBSA/src/
  git clone git@github.com:iris-ua/iris_lama.git
  git clone git@github.com:iris-ua/iris_lama_ros.git
  git clone git@github.com:nobleo/full_coverage_path_planner.git
  git clone git@github.com:nobleo/tracking_pid.git
  cd ../
  catkin_make
  source devel/setup.bash
  roslaunch mir100_navigation disinfection.launch
```
