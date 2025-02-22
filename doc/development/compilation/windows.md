---
Author: Howard Butler
Contact: howard at hobu.co
Date: 11/20/2020
---

(building-windows)=

# Building Under Windows

```{note}
{ref}`conda` contains a pre-built up-to-date 64 bit Windows binary. It
is fully-featured, and if you do not need anything custom, it is likely
the fastest way to get going.
```

## Introduction

Pre-built binary packages for Windows are available via {ref}`conda` (64-bit version),
and all of the prerequisites required for compilation of a fully featured build
are also available via that packaging system. This document assumes you
will be using Conda Forge as your base, and anything more advanced is beyond
the scope of the document.

```{note}
The GitHub Action build system uses the PDAL project's configuration on the Conda Forge
system. It contains a rich resource of known working examples. See
<https://github.com/PDAL/PDAL/blob/master/.github/workflows/win.yml> and
<https://github.com/PDAL/PDAL/tree/master/scripts/ci/win> for inspiration.
```

## Required Compiler

PDAL is known to compile on [Visual Studio 2015], and 2013 *might* work with
some source tree adjustments. PDAL makes heavy use of C++11, and a compiler
with good support for those features is required.

## Prerequisite Libraries

PDAL uses the [GitHub Actions] continuous integration platform for building and
testing itself on Windows. The configuration that PDAL uses is valuable
raw materials for configuring your own environment because the PDAL
team must keep it up to date with both the {ref}`conda` environment and
the Microsoft compiler situation.

You can see the current configuration at
<https://github.com/PDAL/PDAL/blob/master/.github/workflows/win.yml> The most interesting bits
are the `Setup` step, the `CMake` step, and the `Compile` scripts.
The configuration already has Miniconda installed, and the
`setup.sh` script installs all of PDAL's prerequisites via the command
line.

```
conda install geotiff laszip nitro curl ^
   gdal pcl cmake eigen ninja libgdal ^
   zstd numpy xz libxml2 laz-perf qhull ^
   sqlite hdf5 tiledb conda-build ninja -y
```

```{note}
The package list here might change over time. The canonnical location
to learn the  prerequisite list for PDAL is the `scripts/ci/win`
directory in PDAL's source tree.
```

## Fetching the Source

Get the source code for PDAL. Presumably you have [GitHub for Windows] or
something like it. Run a "git shell" and clone the repository into the
directory of your choice.

> ```
> c:\dev> git clone https://github.com/PDAL/PDAL.git
> ```

Switch to the `-maintenance` branch.

> ```
> c:\dev> git checkout 1.9-maintenance
> ```
>
> ```{note}
> PDAL's active development branch is `master`, and you are welcome to
> build it, but is not as stable as the major-versioned release
> branches are likely to be.
> ```

## Configuration

PDAL uses [CMake] for its build configuration. You will need to install CMake
and have it available on your path to configure PDAL.

Invoke your `cmake` command to configure the PDAL.

```
cmake -G "NMake Makefiles" .
```

A fully-featured build will require more specification of libraries, enabled
features, and their locations. For more information on this, users can refer to the `examples.sh` step in the Action.

```{note}
Placing your command in a `.bat` file will make for easy reuse.
```

## Building

If you chose `NMake Makefiles` as your CMake generator, you can
invoke the build by calling **nmake**:

```
nmake /f Makefile
```

If you chose "Visual Studio 14 Win64" as your CMake generator, open `PDAL.sln`
and chose your configuration to build.

## Running

After you've built the tree, you can run `pdal.exe` by issuing it

```
c:\dev\pdal\bin\pdal.exe
```

```{note}
You may need to have your Conda environment active to enable access to
PDAL's dependencies.
```

[cmake]: http://www.cmake.org
[github actions]: https://github.com/PDAL/PDAL/actions
[github for windows]: https://desktop.github.com/
[visual studio 2015]: https://www.visualstudio.com/vs/older-downloads/
