# Vulkan Minimal Hybrid Rendering

A minimal hybrid rendering sample using ray query

![](https://user-images.githubusercontent.com/30839669/127963548-6fe937f0-3739-402e-ac3c-5e3fe6468266.png)

## Features

- Rasterization
- Raytraced shadow

## Environment

- Vulkan SDK 1.2.162.0 or later
- GPU / Driver that support Vulkan Ray Query
- C++ 17

## Clone

```sh
git clone --recursive https://github.com/yknishidate/vulkan-minimal-hybrid-rendering
cd vulkan-minimal-hybrid-rendering
```

## Build

```sh
mkdir build
cd build
cmake ..
cmake --build .
```

## Run

```sh
# on Windows
Debug\vulkan_minimal_hybrid_rendering.exe
```
