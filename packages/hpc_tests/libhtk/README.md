# LibHTK

<!-- TOC -->
- [LibHTK](#libhtk)
  - [Introduction](#introduction)
  - [Compilation](#compilation)
  - [Command line arguments and IO types](#command-line-arguments-and-io-types)
  - [Docs](#docs)
    - [htkLog](#htklog)
      - [Example](#example)
    - [htkTime_start](#htktime_start)
      - [Example](#example-1)
    - [htkTime_stop](#htktime_stop)
      - [Example](#example-2)
    - [htkArg_read](#htkarg_read)
      - [Example](#example-3)
<!-- /TOC -->

## Introduction
LibHTK is a library required by the HPC labs. This library allows the labs to be self contained in the sense that no external libraries are needed to run.

The main features of the library are:

  - Logging capabilities for debugging with details of a lab run
  - Time measurement functions
  - Import and export PPM images
  - Import and export raw vectors and matrices

## Compilation
The library will be built automatically as a dependency when building a lab. Alternatively, the library can be built separately by typing `make` from within the `libhtk` directory.

## Command line arguments and IO types
Programs using LibHTK, like the HPC labs, have the ability to read command line arguments. A typical command line run is:
``` sh
./program -e <expected output file> -i <input file 1>,<input file 2> -o <output> -t <type>
```
Where:<br/>
&nbsp;&nbsp;`-e <expected output file>` is the expected output produced by the data generator.<br/>
&nbsp;&nbsp;`-i <input file 1>,<input file 2>` are the input files.<br/>
&nbsp;&nbsp;`-o <output>` is the file to store the output.<br/>
&nbsp;&nbsp;`-t <type>` type of the output file, it can be any of:<br/>
&nbsp;&nbsp;&nbsp;&nbsp;`vector`<br/>
&nbsp;&nbsp;&nbsp;&nbsp;`matrix`<br/>
&nbsp;&nbsp;&nbsp;&nbsp;`image`

**Observe that arguments of the same option are specified without spaces and separated by commas:**
``` sh
./program -i file1, file2  // Wrong usage, will result in an error or segfault
./program -i file1,file2   // Correct usage
```

## Docs

### htkLog
The function **htkLog** outputs a message to stdout and logs the message with a TAG.
```
htkLog(TAG, T msg0, ... ):
  TAG : indicates the type of the message, useful for debugging purposes
  msg0: message to output to stdout and log on the TAG logs,
        where T is a printable type like a string or a number
  ... : additional messages to output to stdout and log on the TAG logs
```

Available TAG kinds are:
```
  FATAL    // For reporting fatal errors
  ERROR    // For reporting errors
  WARN     // For reporting warnings
  INFO     // For providing information
  DEBUG    // For debugging
  TRACE    // For traditional stdout
```

#### Example
``` cpp
   int a = 0;
   int error = 1;
   htkLog(TRACE, "Print value of a ", a);
   if (error != 0)
      htkLog(ERROR, "An error has occurred, the error code is ", error);
```

### htkTime_start
The function **htkTime_start** starts a timer identified by a TAG and a message.
```
htkTime_start(TAG, string identifier):
  TAG: indicates the kind of the timer
  identifier: message to identify the timer
```

**Important:** In order to stop a timer, _htkTime_stop_ must be called with same TAG and identifier as _htkTime_start_.

Available TAG kinds are:
```
  Generic          // Tag for generic time measurements
  IO               // Tag for input output time measurements
  GPU              // Tag for general GPU time measurements
  Copy             // Tag for synchronic memory transfer time measurements
  Driver           // Tag for driver function time measurements
  CopyAsync        // Tag for a-synchronic memory transfer time measurements
  Compute          // Tag for kernel time measurements
  CPUGPUOverlap    // Tag for CPU GPU overlapping time measurements
```

#### Example
``` cpp
  htkTime_start(Generic, "Initializing memory"); // Start generic timer with
                                                 // identifier "Initializing memory"
  // Initialize memory
  htkTime_stop(Generic, "Initializing memory");  // Stop generic timer
```

### htkTime_stop
The function **htkTime_stop** stops a timer identified by a TAG and a message.
```
htkTime_stop(TAG, string identifier):
  TAG: indicates the kind of the timer
  identifier: message to identify the timer
```

**Important:** In order to stop a timer, _htkTime_stop_ must be called with the same TAG and identifier as _htkTime_start_.

Available TAG kinds are:
```
  Generic          // Tag for generic time measurements
  IO               // Tag for input output time measurements
  GPU              // Tag for general GPU time measurements
  Copy             // Tag for synchronic memory transfer time measurements
  Driver           // Tag for driver function time measurements
  CopyAsync        // Tag for a-synchronic memory transfer time measurements
  Compute          // Tag for kernel time measurements
  CPUGPUOverlap    // Tag for CPU GPU overlapping time measurements
```

#### Example
``` cpp
  htkTime_start(Compute, "Performing vector add"); // Start compute timer with
                                                   // identifier "Performing vector add"
  // Launch kernel performing vector add
  htkTime_stop(Compute, "Performing vector add");  // Stop compute timer
```

### htkArg_read
The function **htkArg_read** reads and processes the command line arguments passed to the program.
```
  htkArg_read(int argc, char **argv):
    argc: argument count
    argv: string array with the arguments
```

#### Example
``` cpp
  // Call program with: ./main -i file1,file2,file3
  int main(int argc, char **argv) {
    htkArg_t args;
    args = htkArg_read(argc, argv);
    std::string arg0 = htkArg_getInputFile(args, 0); // arg0 now contains "file1"
  }
```
