## v0.2.0 (2025-02-16)

### Feat

- **tlv493d**: add support for idf > 5.2
- **idf_component.yml**: add support for esp-idf packetmanager
- **.cz.json**: add commitzen configuration file
- **tlv493d.cpp**: change readout handling based on timer

### Fix

- **tlv93d**: some changes in interrupt implementation and reformat some log messages and variables
- **tlv493d.cpp**: fix registry write for fast and mastercontroller mode
- **tlv493d.cpp**: change to the right register to check power down flag
- **idf_component.yml**: resolve 'unknown keys' error
- **tlv493d.cpp**: resolve error in reset() function
- fix hanging after after some time
- **tlv493d.cpp**: restart timer to get new data from sensor
- **conf.py**: change version to semver style

### Refactor

- **tlv493d.cpp**: change logmode for read register to debug
- **tlv493d.cpp**: remove unnecessary printf´s
