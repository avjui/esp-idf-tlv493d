Quickstart
**********

| Here you can find a short example how to use the library in your project. 
| You can also find an example in `example`_ folder.

.. _example: https://github.com/avjui/esp-idf-tlv493d/tree/master/example

Installation
#############

ESP-IDF
^^^^^^^^

The library contains a `idf_component.yml` file. So you can install it with the esp-idf packagemanager to be accessible in your project.

To integrate it to your project create a file named `idf_component.yml` and put following lines in it.

.. code-block:: yaml

    dependencies:
        idf: ">=5.1"

        tlv493d:
            git: https://github.com/avjui/esp-idf-tlv493d.git


The file was also available in the [example folder main directory](./example/base/main/).

After this installation is complete and you can build your project with the esp-idf-tlv493d library.

Platformio
^^^^^^^^^^^^

To use this library in platformio you can add `lib_deps` to your configuration file `platformio.ini`

.. code-block:: ini

    lib_deps = 
        git+https://github.com/avjui/esp-idf-tlv493d

.. warning::
    Because platformio not register the `Kconfig` by him self you have to add `Kconfig` with editing `CMakeLists.txt`, or add the config variables to `platformio.ini`. 

- First option is to add following lines to `CMakeLists.txt` in the root directory.

.. code-block:: cmake

    get_filename_component(configName "${CMAKE_BINARY_DIR}" NAME)
    list(APPEND kconfigs "${CMAKE_SOURCE_DIR}/.pio/libdeps/${configName}/esp-idf-tlv493d/Kconfig")

- Second option is to add the config to `platformio.ini`.

.. code-block:: ini

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


Initialization and setup
#########################

At first we must include the header files

.. code-block:: cpp

    #include <tlv493d.h>

Constructor with default config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| After that we can create a Object with the standard config. 
| This means the configuration made by :code:`menuconfig` well be used. 

.. note:: There you can find a :code:`TLV493D Configuration` section.

.. code-block:: cpp

    /* Init with default config 
    *  or config make with 'menuconfig' 
    */
    TLV493D magnetometer;
    tlv493d_conf_t conf_magnetometer = DEFAULT_TLV493D_CONFIG();


Add custom config
^^^^^^^^^^^^^^^^^^
| If you want more control over the configuration you can use the overloaded constructor.
| There you can use the :code:`tlv493d_conf_t` struct.

.. code-block:: cpp

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


Usage
######

Perform update of values
^^^^^^^^^^^^^^^^^^^^^^^^^

Here we have two options to update the :code:`conf_magnetometer` values.

One option can be to start the background task.

.. code-block:: cpp

    /* Start background task to periodically read out value from tlv493d modul */
    magnetometer.startBackgroundRead();


The second option is to perform a manual update of the date. 

.. note:: This function blocks until it get data or 10 retries fails.


.. code-block:: cpp

    /* Read out data manual. Note this is a blocking function */
    esp_err_t err = magnetometer.update();
    if (err != ESP_OK)
    {
        ESP_LOGI("MAIN", "Read out new data failed");
    }


Read magnetics values
^^^^^^^^^^^^^^^^^^^^^^

To read the load you can use the variables from struct :code:`conf_magnetometer`. If you have started
the background task you can read the value, without doing anything else.

.. code-block:: cpp

    /* Read value and print */
    ESP_LOGI("MAIN", "X: %f, Y: %f, Y: %f", conf_magnetometer.dataX, conf_magnetometer.dataY, conf_magnetometer.dataY);

