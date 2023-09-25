
# Program with Astro

## Contents

- [1. Introdução](#introducao)

2. [CARMEN Style Guide](#carmen-style-guide)

    2.1. [The Prime Directive](the-prime-directive)

    2.2. [Units and Coordinates](#units-and-coordinates)

    2.3. [Naming Conventions](#naming-conventions)

    2.3. [Memory Management, System Calls, & Functions with Side Effects](#Memory-Management-System-Calls-Functions-with-Side-Effects)

    2.3. [IPC](#ipc)

    2.3. [Graphics](#graphics)

3. [Getting Data from Carmen](#getting-data-from-carmen)

    3.1. [Sensor Data from the Base](#sensor-data-from-the-base)

    3.2. [Map-based Navigation Messages](#map-based-navigation-messages)

4. [Commanding the Robot](#commanding-the-robot)

    4.1. [Moving the Robot](#moving-the-robot)

    4.2. [Initializing Localize](#initializing-localize)

    4.3. [Autonomous Motion](#autonomous-motion)

5. [Getting Maps](#Getting-maps)

    5.1. [Grid Maps](#grid-maps)

    5.2. [Map Placelists](#map-placelists)

    5.3. [Off Limits Regions](#off-limits-regions)

    5.3. [Navigator Maps](#navigator-maps)

6. [Getting Parameters](#getting-parameters)

    6.1. [Subscribing to Changes](#subscribing-to-changes)

    6.2. [Map Placelists](#map-placelists)

    6.3. [The Parameter Factory](#the-parameter-factory)

    6.3. [Specifying Parameters from the Command Line](#specifying-parameters-from-the-command-line)



## [1. Introdução](#introducao)

This document is designed to get CARMEN users started in writing new programs for integration into the program set. Because of the diverse skills and habits of programmers, the first section is a Style Guide to ensure compatability and clarity of new programs. The next secions describe the commands for getting information and issuing commands within CARMEN.

## CARMEN Style Guide

### 2.1  The Prime Directive

You are not the only person who will ever have to read, understand and modify your code.

### 2.2  Units and Coordinates

Always represent all units in MKS. All distances are always in metres. All angles are always, always in radians.
All floating point numbers should be doubles, not floats, and all fixed point numbers should be ints, not chars or shorts. The only known exceptions are large, low-precision data chunks, i.e. laser data and maps.
All co-ordinate frames, internal and external, are right-handed. This means that q always increases counter-clockwise, from positive x to positive y. This is the opposite of screen graphics.
q = 0 always points along the positive x axis.
There are exactly three allowable co-ordinate frames.
The robot's frame of reference. Distances are in metres, and the robot always faces along the positive x axis.
The global frame of reference. Distances are in metres, and q = 0 is with respect to a map. This is a meaningless frame of reference without a map.
The map frame of reference. Distances are in grid cells, and q = 0 is with respect to a map. This is a meaningless frame of reference without a map.
Never convert between radians and degrees yourself. Always use
carmen_radians_to_degrees and carmen_degrees_to_radians.
Angles are always between -p and p. Never normalize angles yourself. Always use carmen_normalize_theta.
Never use asin, acos or atan to recover angles distances. Always use atan2 (3).
           theta = atan2(y, x);

should always be used instead of
           theta = atan(y/x);

If you need the hypotenuse of something, use hypot (3) - do not take the sum of squares and find the square root. hypot (3) should be used for code clarity.
Try not to invent your own data structures. Use one of
carmen_point_t
carmen_traj_point_t
carmen_map_point_t
carmen_world_point_t
making sure that you use the right data structure to store the right kind of data.
When converting between coordinate frames, use the helper functions in map_interface.h and global.h.
When drawing to the screen, do not maintain internal representations of data in screen co-ordinates. Convert to screen co-ordinates only immediately before calling extern graphics functions. Use the helper functions in global_graphics.h.

### 2.3  Naming Conventions

The most important naming convention is to expect that your code could be converted into a library one day. Therefore, it is important to think about global name spaces.
As many functions and global variables should be static as possible. In general, try to avoid global variables.
Static global variables with accessor functions are preferable to non-static globals.
Any non-static functions and global variables must have the prepend carmen_{module-name}. e.g., carmen_base_subscribe_odometry.

### 2.4  Memory Management, System Calls, & Functions with Side Effects

Never use a system call without checking the return value. This includes any memory allocation. We have provided a function
carmen_test_alloc that facilitates memory checking. CVS will not allow code to be committed to core CARMEN that contains a call to malloc/calloc/realloc and does not have a call to carmen_test_alloc on the subsequent line.
When using statically allocated arrays, especially strings, never make the array ``just big enough''.

Wrong: char buffer[11] for a string of length 10.

Right: char buffer[1024] for a string of length 10.

Why? You minimize the probability of off-by-one errors writing into memory you don't own. Memory is cheap. If you really need to create ``just-big-enough'' memory arrays because you're running out of memory, you're solving the wrong problem.

Never use fscanf, gets, etc to read into buffers without limit.
Never make a system call, and check its side effect in one step. For example,

           fd = open(filename, O_RDONLY);
           if (fd < 0)
             return -1;

should always be used instead of

           if ((fd = open(filename, O_RDONLY)) < 0)
             return -1;

This is for two reasons:

Code clarity : it is easier to read the former than the latter.
Debugging : it is easier to use a debugger with the former than the latter.

Avoid macros. If you must have a macro, write an inline function instead. Macros can hurt you in the following manner:

           #define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
           ...
                  max_x = MAX(X++, Y++);

The larger of the two fields will be incremented twice. Instead, use:

           inline int max_int(int x, int y) {
             return (x > y ? x : y);
           }

Under compiler flags -O, these two code fragments are compiled identically under gcc, but the macro has unexpected side-effects.

Never generate random numbers yourself. You will do it wrong. Always use one of carmen_int_random, carmen_uniform_random or carmen_gaussian_random. Consult the documentation for these functions to find out which random algorithm they use.

Never create random number seeds yourself. You will do it wrong. Always use carmen_randomize. This function randomizes by reading a seed from /dev/random.

### 2.5  IPC

Do not initialize IPC yourself. Use carmen_ipc_initialize.
If you write a stand-alone module, there should be three separate files with your module:

{module}_messages.h - this contains the IPC message definitions.

{module}_interface.h - this contains the function prototypes for communicating with your module.

{module}_interface.c - this contains the actual functions for communicating with your module. This file should be compiled into a library.

We expect that all modules will have well-defined interface libraries that relieve the user of having to worry about marshalling data.

Every IPC message should have a timestamp and hostname field. The hostname should be 10 chars long, and should be a canonical representation of the machine on which the process is running that created the message. Use the helper function carmen_get_tenchar_host_name() to generate the hostname.
The timestamp should also reflect the time of creation of the data, not the time the message was published. For instance, the laser message timestamp is the time the data was read from the serial port. Use the helper function carmen_get_time_ms() to generate the timestamp as a double.

### 2.6  Graphics

Keep graphics and robot functionality in separate processes.
Notice that none of the core robot functionality (base_services, robot, navigator, localize) link against graphics libraries. This is for multiple reasons:

It increases the likelihood that your code will work on a standalone robot in the absence of a network. If your process is displaying output to a remote X window, and the network drops out, your process will wedge until the network comes back. 

If, however, your process is sending IPC messages to an external display process, then only the display process will wedge, not your (presumably more important) robot process.
You are removed from the temptation of displaying internal information not accessible to external code. 

If you want to see the state of your process, the odds are very high that eventually, someone else will want to use the state of your process. By forcing you to write an IPC message and interface handler for that information just to get it into the display, you make life much easier for people who come after you.

Allowing your code to compile without graphics makes it more portable, and makes it more distributable, in the sense that it can run anywhere - it is not constrained by the speed of network connections between machines.

For obvious reasons, programs like map_editor and mapper have displays compiled in - these programs are not intended for autonomous use on the robot.

