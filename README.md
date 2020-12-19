# I²C

## Contents of Readme

1. About
2. Modules
3. IOs of Modules
4. Inter-Integrated Circuit (Brief information)
5. Test
6. Status Information
7. Issues

[![Repo on GitLab](https://img.shields.io/badge/repo-GitLab-6C488A.svg)](https://gitlab.com/suoglu/i2c)
[![Repo on GitHub](https://img.shields.io/badge/repo-GitHub-3D76C2.svg)](https://github.com/suoglu/Simple-I2C)

---

## About

Set of simple modules to communicate via I²C Bus.

## Modules

I²C communication modules and clock generator modules are included in [i2c.v](Sources/i2c.v).

**`i2c_master`**

* I²C Master module
* Only supports single master
* Can read/write up to 4 Bytes of data in single transaction.
* Works with 3.125MHz, 781.25kHz, 390.625kHz and 97.656kHz clock frequencies

**`i2c_slave`**

* I²C Slave module
* Currently does not work

**`clockGen_i2c`**

* I²C Clock generator
* Contains only clock dividers
* Output frequency can be set to 3.125MHz, 781.25kHz, 390.625kHz or 97.656kHz
* Input clock should be 100 MHz for above values, for other fin values scale with fin/100e6

## IOs of Modules

|   Port   | Module | Type | Width |  Description |
| :------: | :----: | :----: | :----: |  ------    |
| `clk` | M/S |  I  | 1 | System Clock |
| `rst` | M/S  |  I  | 1 | System Reset |
| `freqSLCT` | M |  I  | 2 | Frequency Select |
| `busy` | M/S  |  O  | 1 | System in transaction |
| `newData` | M/S |  O  | 1 | Pulse to show new received |
| `dataReq` | M |  O  | 1 | New data should be entered |
| `data_valid` | M |  I  | 1 | `data_i` is valid, required to be high to enter `WRITE` state |
| `start` | M |  I  | 1 | Start a new transaction |
| `data_byte_size` | M |  I  | 2 | (Byte count - 1) for transaction |
| `read_nwrite` | M |  I  | 1 | Read/Not Write |
| `addr` | M/S |  I  | 7 | I²C device address |
| `data_i` | M/S |  I  | 8 | Data input |
| `data_o` | M/S |  O  | 8 | Data Output |
| `SCL` | M/S |  O/I  | 1 | I²C clock signal |
| `SDA` | M/S |  IO  | 1 | I²C data signal |

M: Master S: Slave I: Input  O: Output

## Inter-Integrated Circuit (Brief information)

From [Wikipedia](https://en.wikipedia.org/wiki/I%C2%B2C): I²C uses only two bidirectional open collector or open drain lines, Serial Data Line (SDA) and Serial Clock Line (SCL), pulled up with resistors. The I²C reference design has a 7-bit address space, with a rarely used 10-bit extension.

## Test

### Master

Master module tested on [Basys 3](https://reference.digilentinc.com/reference/programmable-logic/basys-3/reference-manual) with [master_board.v](Test/master_board.v) and [Basys3_master.xdc](Test/Basys3_master.xdc). [Pmod HYGRO](https://reference.digilentinc.com/reference/pmod/pmodhygro/start) is connected to upper JB header. `SDA`, `SCL`, `newData` and `busy` signals monitored using logic analyzer and protocol spying of [Digital Discovery](https://reference.digilentinc.com/reference/instrumentation/digital-discovery/start). Read values connected to seven segment displays. Right button used to initiate new transaction. Switches used to get user inputs. Writing one byte data and reading two byte data is tested.

## Status Information

**Last test of master module:** 19 December 2020, on [Digilent Basys 3](https://reference.digilentinc.com/reference/programmable-logic/basys-3/reference-manual).

## Issues

* Slave module does not work
