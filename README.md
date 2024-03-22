# Sigfox End-Point library (EP_LIB)

## Description

The **Sigfox End-Point Library** is a public example of the [Sigfox radio protocol](https://build.sigfox.com/sigfox-device-radio-specifications) implementation. This is the **official open source successor** of the old library delivered by Sigfox (versions up to `v2.11.0`). It provides the middleware to build Sigfox frames and schedule the transmission or reception appropriately.

Most of Sigfox features are implemented:

* **Application messages** (uplink and optional downlink procedures).
* **Control keep alive message**.
* Single or multi **radio configurations** support (individually selectable RCx).
* Single or multi frames per message (**N=1**, **2** or **3**).
* **100 bps** and/or **600 bps** uplink bitrates support.
* Fixed or dynamic **uplink payload size**.
* **Public key** switch.
* Configurable **message counter rollover**.

This new library has a clear differentiator in term of **memory optimization**: thanks to a very **modular architecture** and multiple **compilation flags**, the code can be **optimized** to match any device application. The library is designed to operate either in **blocking** or **asynchronous** mode, and supports most of **MCUs** and **radio chipsets**.

## Technical links

* [Sigfox End-Point library wiki](https://github.com/sigfox-tech-radio/sigfox-ep-lib/wiki) : please visit the **wiki** to read more **documentation** and get started with **examples**.
* [Sigfox 0G technology discussion space](https://github.com/orgs/sigfox-tech-radio/discussions) : do not hesitate to use this space if you find any **bug** or want to share a **common interest improvement**.
* [Sigfox device radio specification](https://build.sigfox.com/sigfox-device-radio-specifications) : this is the input specification of the Sigfox End-Point library, implemented in C langage for embedded device projects. For a better comprehension the code uses the **same names and acronyms** as the document.

## Stack architecture

<p align="center">
<img src="https://github.com/sigfox-tech-radio/sigfox-ep-lib/wiki/images/sigfox_ep_lib_architecture.drawio.png" width="600"/>
</p>

## Compilation flags for optimization

Most of Sigfox radio parameters and features are **conditioned to a dedicated flag**, so that the stack can be configured to perfectly match your application, without dead code and thus with a **minimum memory footprint**. The flags are located in the `sigfox_ep_flags.h` file, but can also be set through the `cmake` command when building the project (see section [How to add Sigfox library to your project](https://github.com/sigfox-tech-radio/sigfox-ep-lib/wiki/how-to-add-sigfox-library-to-your-project)).

To have such a flexibility, the stack uses a lot of preprocessor directives, which makes the source code less readable. If you plan to look or modify the source files, we advise you to run the cmake pre-compilation command, that will **remove all preprocessor directives** according to your flags selection (see section [Precompiled source code](https://github.com/sigfox-tech-radio/sigfox-ep-lib/wiki/how-to-add-sigfox-library-to-your-project#precompiled-source-code)).

Below is the list of available flags.

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `RCx_ZONE` | `undefined` / `defined` | Support the RCx radio configuration if defined. |
| `APPLICATION_MESSAGES` | `undefined` / `defined` | Support uplink application messages if defined. |
| `CONTROL_KEEP_ALIVE_MESSAGE` | `undefined` / `defined` | Support uplink control keep alive message if defined. |
| `BIDIRECTIONAL` | `undefined` / `defined` | Support bidirectional procedure (downlink) if defined. Only applicable to application messages. Otherwise all messages will be uplink only. |
| `ASYNCHRONOUS` | `undefined` / `defined` | Asynchronous mode if defined, blocking mode otherwise. |
| `LOW_LEVEL_OPEN_CLOSE` | `undefined` / `defined` | Enable MCU and RF open/close functions if defined. |
| `REGULATORY` | `undefined` / `defined` | Enable radio regulatory control (DC, FH or LBT check) if defined. |
| `LATENCY_COMPENSATION` | `undefined` / `defined` | Enable radio latency compensation to improve MCU timers accuracy. |
| `SINGLE_FRAME` | `undefined` / `defined` | Send 1 frame per message (N=1) if defined. Otherwise number of frames per message is dynamically given when sending a message (N=1, 2 or 3). |
| `UL_BIT_RATE_BPS` | `undefined` / `100` / `600` | If defined, give the only uplink bit rate supported (100bps or 600bps depending on the RC). Otherwise, value is dynamically given when sending a message. |
| `TX_POWER_DBM_EIRP` | `undefined` / `<tx_power_dbm_eirp>` | If defined, give the only TX power supported by the radio. Otherwise the value is dynamically given when sending a message. |
| `T_IFU_MS` | `undefined` / `<t_ifu_ms>` | If defined, give the fixed inter-frame delay used between uplink frames of a same message (0 to 2000ms). Value 0 disables the delay and associated timers to optimize memory space. Otherwise value is dynamically given when sending a message. |
| `T_CONF_MS` | `undefined` / `<t_conf_ms>` | If defined, give the fixed delay between downlink frame reception and uplink confirmation message (1400 to 4000ms). Otherwise value is dynamically given when sending a message. |
| `UL_PAYLOAD_SIZE` | `undefined` / `<ul_payload_size>` | If defined, give the only uplink payload length supported (0 to 12). Value 0 enables the bit 0, bit 1 and empty messages. Otherwise, all uplink payload lengths are dynamically supported. |
| `CRC_HW` | `undefined` / `defined` | If defined, enable hardware CRC through MCU API functions. Otherwise the embedded driver is used. |
| `MESSAGE_COUNTER_ROLLOVER` | `undefined` / `128` / `256` / `512` / `1024` / `2048` / `4096` | If defined, give the only message counter rollover value supported. Otherwise, value is dynamically given when opening the library. |
| `PARAMETERS_CHECK` | `undefined` / `defined` | Enable parameters check if defined. |
| `CERTIFICATION` | `undefined` / `defined` | Enable certification features if defined. |
| `PUBLIC_KEY_CAPABLE` | `undefined` / `defined` | Enable public key switch feature if defined. |
| `VERBOSE` | `undefined` / `defined` | Enable credentials (ID / PAC) API access and version control functions if defined. |
| `ERROR_CODES` | `undefined` / `defined` | Use return codes if defined, otherwise all functions return void. |
| `ERROR_STACK` | `undefined` / `<error_stack_depth>` | If defined, store low level errors in a stack (the macro gives the depth). Errors can be read with the `SIGFOX_EP_API_unstack_error()` function. |

## How to add Sigfox library to your project

### Submodule

The best way to embed the Sigfox End-Point library into your project is to use a [Git submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules). The library will be seen as a sub-repository with independant history. It will be much easier to **upgrade the library** or to **switch between versions** when necessary, by using the common `git pull` and `git checkout` commands within the `sigfox-ep-lib` folder.

In order to keep the repository clean, it is recommended to **select the compilation flags in your project settings**, without modifying the `sigfox_ep_flags.h` file (which is only included if the `USE_SIGFOX_EP_FLAGS_H` flag is defined). This way, you can change the compilation flags when you want, in order to add features or save code memory. The source code will be **set up automatically according to your flags selection**, without any action or diff in the submodule folder. You will also need to **exclude** the `src/manuf` folder from build (where the manufacturer templates are defined) and implement the functions in another location of your project, so that the repository is kept clean.

To add the Sigfox library submodule, go to your project location and run the following commands:

```bash
mkdir lib
cd lib/
git submodule add https://github.com/sigfox-tech-radio/sigfox-ep-lib.git
```

This will clone the Sigfox End-Point library repository. At project level, you can commit the submodule creation with the following commands:

```bash
git commit --message "Add Sigfox End Point library submodule."
git push
```

With the submodule, you can easily:

* Update the library to the **latest version**:

```bash
cd lib/sigfox-ep-lib/
git pull
git checkout master
```

* Use a **specific release**:

```bash
cd lib/sigfox-ep-lib/
git pull
git checkout <tag>
```

### Raw source code

You can [download](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases) or clone any release of the Sigfox End-Point library and copy all files into your project. Then you can either define the `USE_SIGFOX_EP_FLAGS_H` flag and select your compilation flags in the `sigfox_ep_flags.h` file, or define them in your project settings.

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-lib.git
```

### Precompiled source code

You can [download](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases) or clone any release of the Sigfox End-Point library and copy all files into your project. If you do not plan to change your compilation flags in the future, you can perform a **precompilation step** before copying the file in your project. The precompilation will **remove all preprocessor directives** according to your flags selection, in order to produce a more **readable code**. Then you can copy the new files into your project.

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-lib.git
```

To perform the precompilation, you have to install `cmake` and `unifdef` tools, and run the following commands:

```bash
cd sigfox-ep-lib/
mkdir build
cd build/
```

* Precompiling by reading the `sigfox_ep_flags.h` file:

```bash
cmake -DUSE_SIGFOX_EP_FLAGS_H=ON ..
make precompil
```

* Precompiling by entering the flags selection on command line:

```bash
cmake -DUSE_SIGFOX_EP_FLAGS_H=OFF \
      -DRC1_ZONE=ON \
      -DRC2_ZONE=ON \
      -DRC3C_ZONE=ON \
      -DRC3D_ZONE=ON \
      -DRC4_ZONE=ON \
      -DRC5_ZONE=ON \
      -DRC6_ZONE=ON \
      -DRC7_ZONE=ON \
      -DAPPLICATION_MESSAGES=ON \
      -DCONTROL_KEEP_ALIVE_MESSAGE=ON \
      -DBIDIRECTIONAL=ON \
      -DASYNCHRONOUS=ON \
      -DLOW_LEVEL_OPEN_CLOSE=ON \
      -DREGULATORY=ON \
      -DLATENCY_COMPENSATION=ON \
      -DSINGLE_FRAME=ON \
      -DPARAMETERS_CHECK=ON \
      -DCERTIFICATION=ON \
      -DPUBLIC_KEY_CAPABLE=ON \
      -DVERBOSE=ON \
      -DCRC_HW=OFF \
      -DERROR_CODES=ON \
      -DUL_BIT_RATE_BPS=OFF \
      -DT_IFU_MS=OFF \
      -DT_CONF_MS=OFF \
      -DUL_PAYLOAD_SIZE=OFF \
      -DMESSAGE_COUNTER_ROLLOVER=OFF \
      -DERROR_STACK=12 ..
make precompil
```

The new files will be generated in the `build/precompil` folder.

### Static library

You can also [download](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases) or clone any release of the Sigfox End-Point library and build a **static library**.

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-lib.git
```

To build a static library, you have to install `cmake` tool and run the following commands:

```bash
cd sigfox-ep-lib/
mkdir build
cd build/
```

* Building by reading the `sigfox_ep_flags.h` file:

```bash
cmake -DUSE_SIGFOX_EP_FLAGS_H=ON ..
make sigfox_ep_lib
```

* Building by entering the flags selection on command line:

```bash
cmake -DUSE_SIGFOX_EP_FLAGS_H=OFF \
      -DRC1_ZONE=ON \
      -DRC2_ZONE=ON \
      -DRC3C_ZONE=ON \
      -DRC3D_ZONE=ON \
      -DRC4_ZONE=ON \
      -DRC5_ZONE=ON \
      -DRC6_ZONE=ON \
      -DRC7_ZONE=ON \
      -DAPPLICATION_MESSAGES=ON \
      -DCONTROL_KEEP_ALIVE_MESSAGE=ON \
      -DBIDIRECTIONAL=ON \
      -DASYNCHRONOUS=ON \
      -DLOW_LEVEL_OPEN_CLOSE=ON \
      -DREGULATORY=ON \
      -DLATENCY_COMPENSATION=ON \
      -DSINGLE_FRAME=ON \
      -DPARAMETERS_CHECK=ON \
      -DCERTIFICATION=ON \
      -DPUBLIC_KEY_CAPABLE=ON \
      -DVERBOSE=ON \
      -DCRC_HW=OFF \
      -DERROR_CODES=ON \
      -DUL_BIT_RATE_BPS=OFF \
      -DT_IFU_MS=OFF \
      -DT_CONF_MS=OFF \
      -DUL_PAYLOAD_SIZE=OFF \
      -DMESSAGE_COUNTER_ROLLOVER=OFF \
      -DERROR_STACK=12 ..
make sigfox_ep_lib
```

The archive will be generated in the `build/lib` folder.

## Addons

### RF & Protocol

In order to test your implementation against **Sigfox specifications**, you can use the [Sigfox End-Point RF & Protocol addon](https://github.com/sigfox-tech-radio/sigfox-ep-addon-rfp) which will drive the library to perform Sigfox test modes.

The addon can be directly generated from the Sigfox End-Point library **cmake** by using the `ADDON_RFP` option:

```bash
cmake <all previous flags> -DADDON_RFP=ON ..
make precompil_sigfox_ep_addon_rfp
make sigfox_ep_addon_rfp
```

### Type Approval

In order to pass **radio regulatory certifications** on your device, you can use the [Sigfox End-Point Type Approval addon](https://github.com/sigfox-tech-radio/sigfox-ep-addon-ta) which will drive the library to perform regulatory test modes.

The addon can be directly generated from the Sigfox End-Point library **cmake** by using the `ADDON_TA` option:

```bash
cmake <all previous flags> -DADDON_TA=ON ..
make precompil_sigfox_ep_addon_ta
make sigfox_ep_addon_ta
```

## RF API implementation examples

In this organization you will find some example of `RF_API`implementation

### ST S2LP

The [S2LP RF API example code](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-s2lp) can be directly generated from the Sigfox End-Point library **cmake** by using the `S2LP_RF_API` option:

```bash
cmake <all previous flags> -DS2LP_RF_API=ON ..
make precompil_s2lp_rf_api
make s2lp_rf_api
```

### Semtech LR11XX

The [LR11XX RF API example code](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-semtech-lr11xx) can be directly generated from the Sigfox End-Point library **cmake** by using the `LR11XX_RF_API` option:

```bash
cmake <all previous flags> -DLR11XX_RF_API=ON ..
make precompil_lr11xx_rf_api
make lr11xx_rf_api
```

### Semtech SX126X

The [SX126X RF API example code](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-semtech-sx126x) can be directly generated from the Sigfox End-Point library **cmake** by using the `SX126X_RF_API` option:

```bash
cmake <all previous flags> -DSX126X_RF_API=ON ..
make precompil_sx126x_rf_api
make sx126x_rf_api
```
