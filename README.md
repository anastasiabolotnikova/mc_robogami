Robogami robot module
================

This package contains the robot module for the Robogami robot.

It is an essential software component to use Robogami with [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/) control framework.

## Requirements

[mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)

[robogami_description](https://github.com/anastasiabolotnikova/robogami_description)

## Clone

```
git clone https://github.com/anastasiabolotnikova/mc_robogami
```

## Build-install

```
mkdir -p build
cd build
cmake
make
sudo make install
```

## Use
Once installed, robot module can be used in an mc_rtc controller.

Refer to [robogami_controller](https://github.com/anastasiabolotnikova/robogami_controller) for an example.