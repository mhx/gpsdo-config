# gpsdo-config

Find GPS reference clock configuration from set of frequencies

## Overview

This tool is intended to fill a gap in configuring the Precision GPS Reference
Clock from [Leo Bodnar Electronics](http://www.leobodnar.com/) on Linux. There
is already the [lb-gps-linux project](https://github.com/simontheu/lb-gps-linux)
that allows you to set the "raw" parameters of the hardware, but in order to
determine those parameters, you have to rely on the Windows GUI application.

The `gpsdo-config` tool provided here allows you to find these parameters by
specifying the desired output frequencies. The parameters can optionally be
printed as command-line arguments for the `lb-gps-linux` tool, so setting new
output frequencies can be done with a simple one-liner:

```
lb-gps-linux /dev/hidraw3 $(gpsdo-config 10M 120M --cmdline)
```

## Building and Installing

```
cmake .
make -j
sudo make install
```

## Usage

```
Usage: ./gpsdo-config f1 [f2] [options...]

Options:
  --f1 arg              frequency 1
  --f2 arg              frequency 2
  --all                 find all possible solutions
  --any                 find any possible solution
  --best                find best possible solution
  -v [ --verbose ]      print more information
  --cmdline             print command line config
  --json                print solutions as json objects
  -h [ --help ]         produce help message

If only one frequency is specified, both outputs will be set to the
same frequency. Frequencies will be processed accurately as rational
numbers internally, and can also be specified as such. An integral
part can be separated from a fraction by either a single space or an
underscore. Suffixes `M` and `k` are supported for MHz and kHz.

`--all` and `--best` can be really slow as there may be millions
of possible solutions. By default, the code will look for a "good"
solution, which shouldn't be significantly slower than `--any`.
The "quality" of a solution is measured purely by means of the
phase detector comparison frequency (f3), which directly impacts
jitter/phase noise. `--best` will always search for the solution
with the highest possible f3. The default behaviour will accept
any f3 that is higher than 50% of the maximum value.

Output for `--json` and `--cmdline` will always be exclusively
written to stdout, suitable for processing by other commands.
All other output will be written to stderr.

Examples:
  ./gpsdo-config 1000
  ./gpsdo-config 10M 96k
  ./gpsdo-config 1000.31 2345.61 --best
  ./gpsdo-config 10_1/7k 500/9k --all --verbose
  lb-gps-linux /dev/hidraw3 $(./gpsdo-config 10M 120M --cmdline)

Exit status:
  0: successful completion
  1: could not find any solution for the specified frequencies
  2: input processing error
```
