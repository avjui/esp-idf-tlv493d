# ESP-IDF-TLV493D Library

<img src="doc/_static/logo.png" width="200" height="200">

![platformio build](https://github.com/avjui/esp-idf-tlv493d/actions/workflows/build.yml/badge.svg)  ![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/avjui/esp-idf-tlv493d)  [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

## Table of Contents

- [ESP-IDF-TLV493D Library](#esp-idf-tlv493d-library)
  - [Table of Contents](#table-of-contents)
  - [About ](#about-)
  - [Getting Started ](#getting-started-)
    - [Installation ](#installation-)
      - [ESP-IDF ](#esp-idf-)
      - [Platformio ](#platformio-)
  - [Usage ](#usage-)
    - [Constructor with default config](#constructor-with-default-config)
    - [Add custom config](#add-custom-config)
    - [Perform update of values](#perform-update-of-values)
    - [Read magnetics values](#read-magnetics-values)
  - [Documentation ](#documentation-)
  - [Todo ](#todo-)

## About <a name = "about"></a>

This libaray is for the tlv493d ic to read out magnetic data.

:warning: This repository is still in development!

## Getting Started <a name = "getting_started"></a>

Goal of this project is to provide an easy to use libary for the tlv493d written in C++!

The main features are:
  - Easy to use
  - Reading values in background task
  - Manual mode

Here you can find a short example how to use the library in your project.

:file_folder: You can also find an example in [example](https://github.com/avjui/esp-idf-tlv493d/tree/master/example) folder.

:warning: The library was only tested with esp-idf version `5.1` and `5.2`.

### Installation <a name = "installing"></a>

#### ESP-IDF <a name = "esp-idf"></a>

The library contains a `idf_component.yml` file. So you can install it with the esp-idf packagemanager to be accessible in your project.

To integrate it to your project create a file named `idf_component.yml` and put following lines in it.

``` yaml
dependencies:
    idf: ">=5.1"

    tlv493d:
        git: https://github.com/avjui/esp-idf-tlv493d.git
```

The file was also available in the [example folder main directory](./example/base/main/).

After this installation is complete and you can build your project with the esp-idf-tlv493d library.

#### Platformio <a name = "platformio"></a>

To use this library in platformio you can add `lib_deps` to your configuration file `platformio.ini`

``` ini
lib_deps = 
    git+https://github.com/avjui/esp-idf-tlv493d
```

:warning: Because platformio not register the `Kconfig` by him self you have to add `Kconfig`with editing `CMakeLists.txt`, or add the config variables to `platformio.ini`.

-   First option is to add following lines to `CMakeLists.txt` in the root directory.

``` cmake
get_filename_component(configName "${CMAKE_BINARY_DIR}" NAME)
list(APPEND kconfigs "${CMAKE_SOURCE_DIR}/.pio/libdeps/${configName}/esp-idf-tlv493d/Kconfig")
```

-   Second option is to add the config to `platformio.ini`.

``` ini
[tlv493d]
; Build flags for tlv493d library
build_flags=
        -D CONFIG_TLV493D_PIN_SDA=18     ; i2c sda pin
        -D CONFIG_TLV493D_PIN_SCL=19     ; i2c scl pin
        -D CONFIG_TLV493D_TEMP_ENABLE=1  ; enable temp read out
        -D CONFIG_TLV493D_INT_ENABLE=1   ; interrupt enabled

[env:...]
....

build_flags = 
    ${tlv493d.build_flags}
```

## Usage <a name = "usage"></a>

At first we must include the header files

``` cpp
#include <tlv493d.h>
```

### Constructor with default config

After that we can create a Object with the standard config.
This means the configuration made by `menuconfig` well be used.


:warning: There you can find a `TLV493D Configuration` section.


``` cpp
/* Init with default config 
*  or config make with 'menuconfig' 
*/
TLV493D magnetometer;
tlv493d_conf_t conf_magnetometer = DEFAULT_TLV493D_CONFIG();
```

### Add custom config

If you want more control over the configuration you can use the overloaded constructor.
There you can use the `tlv493d_conf_t` struct.

``` cpp
/* Create custom config */
conf_magnetometer = {
        .tlv493d_conf = {  
            .pin_sda = 50,         /* custom pin number for sda */                      
            .pin_scl = 51,         /* custom pin number for scl */  
            .mode = FASTMODE,      /* change mode */
            .use_temp = true,      /* enable temperature read out */
            .intr_enable = false,  /* disable interrupt */
            },
        };

/* Init with custom config */
magnetometer.init(&conf_magnetometer);
```

### Perform update of values

Here we have two options to update the `conf_magnetometer` values.

One option can be to start the background task.

``` cpp
/* Start background task to periodically read out value from tlv493d modul */
magnetometer.startBackgroundRead();
```

The second option is to perform a manual update of the date.

:warning: This function blocks until it get data or 10 retries fails.

``` cpp
/* Read out data manual. Note this is a blocking function */
esp_err_t err = magnetometer.update();
if (err != ESP_OK)
{
    ESP_LOGI("MAIN", "Read out new data failed");
}
```

### Read magnetics values

To read the load you can use the variables from struct `conf_magnetometer`. If you have started the background task you can read the values, without doing anything else.

``` cpp
/* Read value and print */
ESP_LOGI("MAIN", "X: %f, Y: %f, Y: %f", conf_magnetometer.dataX, conf_magnetometer.dataY, conf_magnetometer.dataY);
```

## Documentation <a name = "documentation"></a>

For more information you can have a look at the [documentation](https://avjui.gihub.io/esp-idf-tlv493d).

## Todo <a name = "todo"></a>

  - find a way to connect interrupt handler to i2c sda pin (maybe over gpio matrix)