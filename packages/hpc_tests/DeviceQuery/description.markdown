---
title: Device Query
lab: 1
---

## Objective

The purpose of this lab is to introduce you to the CUDA hardware resources along with their capabilities. You are not expected to understand the details of the code, but should understand the process of compiling and running code that will be used in subsequent modules.

## Instructions

The provided code queries the GPU hardware on the system. Do not concentrate on the `cuda` API calls, but on functions starting with `htk`. The `htkLog` function logs results.

Specifically, the following hardware features are logged:

- GPU device's name
- GPU computation capabilities
- Maximum global memory size
- Maximum constant memory size
- Maximum shared memory size per block
- Maximum number of block dimensions
- Maximum number of grid dimensions
- Warp size

The executable generated as a result of compiling the lab can be run using the following command:

```
./solution
```

Alternatively, just type `make run`.

## Install and Run (some) CUDA Samples

In a separate directory outside of the hpc-labs, download and extract the following archive file.

cuda-samples.tgz

```
tar -xzf cuda-samples.tgz
```

Change directory to cuda-samples and type the following to build all the examples.

```
make SMS=86
```

After the build completes, the executable files can be found in the "cuda-samples/bin/x86_64/linux/release" directory. Change to this directory and run some of them to see what they do. Report which ones are your favorite in the lab questions.

The source code is available for each of the examples and is a great resource when you are writing your own code for GPUs.

To reclaim disk space after building all the applications (~1.5 GB), type the following from the cuda-samples directory.

```
make clean
```

If you are only interested in a few of the examples, you can build them individually. Change to the source directory of an example, and type `make SMS=86`.
