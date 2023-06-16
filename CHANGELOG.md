# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [v3.2](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.2) - 16 Jun 2023

### Added

* New `LATENCY_COMPENSATION` flag which enables **radio latency compensation** to improve MCU timers accuracy. Delay computation is now performed by the core library.
* Add `SIGFOX_ERROR_SOURCE_HW_API` item to handle **board drivers errors** in RF API or MCU API implementation.

### Changed

* Remove **anonymous structures** to improve compilers compatibility.
* Reduce **RAM usage of AES function** with a 16 bytes array whatever the UL payload length.
* Improve **error naming** and TX control **flags naming**.
* Reduce **stack usage** by using global context directly when calling core functions.
* Add **cast operations** to reduce warnings.
* Change **default output power** to 14dBm in flags.

### Fixed

* Fix **bistream size** macro issue when defining `UL_PAYLOAD_SIZE` and `CONTROL_KEEP_ALIVE_MESSAGE` or `BIDIRECTIONAL`.
* Wrong **status assignment** in TX control driver.

### Known limitations

* **Payload encryption** not supported.
* **Secure element** not supported.

## [v3.1](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.1) - 01 Mar 2023

### General

* Add `CHANGELOG` file.

### Added

* Add `MCU_API_timer_status()` function to facilitate **timeout management in RF API** functions.
* Add **logical names for timers** (`MCU_API_TIMER_INSTANCE_xxx` macros) in MCU API to define instances mapping.
* Add `reason` field in timer structure.
* Add `RF_API_get_latency()` function to compensate **radio delays** in MCU API timers duration.
* Add new **bypass flags** in test API for RFP addon (`ul_enable`, `dl_enable`, `dl_decoding_enable`, `dl_conf_enable`, `ldc_check_enable` bits, `rx_frequency_hz`, `cs_max_duration_first_frame_ms`, and `dl_t_w_ms` / `dl_t_rx_ms` timing values).
* Add **Low Duty Cycle (LDC) check** in TX control.
* Add `MCU_API_print_dl_payload()` function to **print RFP addon downlink results**.

### Changed

* Rename fields of message status and split error flag into `execution_error` and `network_error` to **differentiate internal execution errors from external errors** such as TX control failure or downlink reception timeout.
* Move `SIGFOX_UL_BITSTREAM_SIZE_BYTES` **macro** in `inc/sigfox_types.h` to avoid core files inclusion in manufacturer drivers.
* Improve **manufacturer functions call sequence** (`Tw` timer and downlink ending).
* **Random frequency generator**: optimize RAM footprint and increase allowed bandwidth of N=1 and N=2 modes.
* Remove all **function pointers** in blocking mode for specific compilers compatibility.
* Change default 32 bits type to `long`.
* Rename `sigfox_ep_version.h` to `sigfox_ep_lib_version.h`.
* Use `RETURN` macro in manufacturer functions **template**.

### Fixed

* **Buffer overflow** in downlink frame decoding functions (`src/core/sigfox_ep_bitstream.c`).
* Wrong application message **pointer access** in blocking mode (`src/sigfox_ep_api.c`).
* Wrong **LBT carrier sense timeout** computation for second and third frames (`src/core/sigfox_tx_control.c`).

### Removed

* Remove `dl_phy_content_size` field in **RX data structure**, as it is fixed to `SIGFOX_DL_PHY_CONTENT_SIZE_BYTES`.

### Known limitations

* **Payload encryption** not supported.
* **Secure element** not supported.

## [v3.0](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v3.0) - 12 Dec 2022

### General

* First version of the new Sigfox EP library.

### Added

* **Asynchronous** operation mode in addition to the legacy blocking mode.
* **Memory footprint optimization** thanks to multiple **compilation flags** (`inc/sigfox_ep_flags.h`)
* **Precompiled source** files and **library** generation with `cmake`.
* Sigfox **empty frame** support.
* **Message status** and **error stack**.
* New **options in user API**: dynamic TX power, bit rate and inter-frame delay.
* New **random frequency algorithm** with lower memory footprint and better distribution over device ID ranges.
* Using **structures** as functions parameters to improve compatibility between compilation flags and versions.

### Known limitations

* **Payload encryption** not supported.
* **Secure element** not supported.
