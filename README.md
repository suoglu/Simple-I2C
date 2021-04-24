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

I²C communication modules and clock generator modules are included in [i2c.v](Sources/i2c.v). All information in here refers to most recent version unless a version specified.

### I²C Master (`i2c_master`)

* I²C Master module
* Multi master
* Can read/write up to 7 Bytes of data in single transaction.

**(Synthesized) Utilization on Artix-7 XC7A35T-1CPG236C:**

* Slice LUTs: 36 (as Logic)
* Slice Registers: 30 (as Flip Flop)

### I²C Slave (`i2c_slave`)

* I²C Slave module
* Not finished yet

### I²C Clock Generator (`clockGen_i2c`)

* I²C Clock generator
* Output frequency can be set to 3.125MHz, 781.25kHz, 390.625kHz or 97.656kHz
* Input clock should be 100 MHz for above values, for other fin values scale with fin/100e6

## IOs of Modules

### I²C Master IO (`i2c_master`)

|   Port   | Type | Width |  Description |
| :------:| :----: | :----: |  ------    |
| `clk` |  I  | 1 | System Clock |
| `rst` |  I  | 1 | System Reset |
| `clkI2Cx2` | I  | 1 | I²C Clock Source (2 x f<sub>SCL</sub>) |
| `start` | I  | 1 | Start a new transaction |
| `ready` | O  | 1 | System is ready for another transaction |
| `i2cBusy` | O  | 1 | I²C bus is in use by another master |
| `data_available` | O  | 1 | Pulse to show new data Byte should be read |
| `data_request` | O  | 1 | Next data Byte should be entered |
| `data_valid` |  I  | 1 | `data_i` is valid, required to be high to enter `WRITE` state |
| `read_nwrite` |  I  | 1 | Read/Not Write |
| `addr` | I  | 7 | I²C device address |
| `data_i` |   I  | 8 | Data input |
| `data_o` |   O  | 8 | Data Output |
| `data_size` | I  | 3 | Byte size of the transaction |
| `SCL` |   IO  | 1 | I²C clock signal |
| `SDA` |  IO  | 1 | I²C data signal |

I: Input  O: Output

### I²C Clock Generator IO (`clockGen_i2c`)

|   Port   | Type | Width |  Description |
| :------:| :----: | :----: |  ------    |
| `clk_i` |  I  | 1 | System Clock (100 MHz) |
| `rst` |  I  | 1 | System Reset |
| `freqSLCT` | I | 2 | Frequency select |
| `clk_o` | O | 1 | Output Clock |

I: Input  O: Output

|   `freqSLCT`   | f<sub>out</sub> |
| :------:| :----: |
|  *00*   | 195,312 kHz |
|  *01*   |  781,25 kHz |
|  *10*   | 1,5625 MHz |
|  *11*   | 6,25 MHz |

## Inter-Integrated Circuit (Brief information)

From [Wikipedia](https://en.wikipedia.org/wiki/I%C2%B2C): I²C uses only two bidirectional open collector or open drain lines, Serial Data Line (SDA) and Serial Clock Line (SCL), pulled up with resistors. The I²C reference design has a 7-bit address space, with a rarely used 10-bit extension.

## Test

Information about the test of I²C master module can be found below.

### Master Test 1 (v1.x) on 01 January 2021

Master module v1 tested on [Basys 3](https://reference.digilentinc.com/reference/programmable-logic/basys-3/reference-manual) with [master_board.v](Test/master_board.v) and [Basys3_master.xdc](Test/Basys3_master.xdc). [Pmod HYGRO](https://reference.digilentinc.com/reference/pmod/pmodhygro/start) is connected to upper JB header. `SDA`, `SCL`, `newData` and `busy` signals monitored using logic analyzer and protocol spying of [Digital Discovery](https://reference.digilentinc.com/reference/instrumentation/digital-discovery/start). Read values connected to seven segment displays. Right button used to initiate new transaction. Switches used to get user inputs. Writing one byte data and reading two byte data is tested.

### Master Test 2 (v2) on 01 January 2021

Master module v2 tested on [Basys 3](https://reference.digilentinc.com/reference/programmable-logic/basys-3/reference-manual) with tester module in [test_module.v](Test/test_module.v) and constrains [master2test.xdc](Test/master2test.xdc). [Pmod HYGRO](https://reference.digilentinc.com/reference/pmod/pmodhygro/start) is connected to upper JB header. `SDA` and `SCL` signals monitored using logic analyzer of [Digital Discovery](https://reference.digilentinc.com/reference/instrumentation/digital-discovery/start). Read values connected to seven segment displays. Push buttons are used to initiate diffrent transactions. Seven segment display shows two last read Bytes. Switches used to get custom data and LEDs are used for debuging. Waveforms can be found in Wiki.

## Status Information

**Last test of master module:** 24 April 2021, on [Digilent Basys 3](https://reference.digilentinc.com/reference/programmable-logic/basys-3/reference-manual).

**Last test of slave module:** -

## Issues

* Slave module does not work
* Some glitches on SDA while SCL is low in Master module. But it doesn't seem to cause any problems.
