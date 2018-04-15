# Release Notes

## BLE Mesh v1.0.1
This is a hotfix release with documentation/bug fixes.

### New features
- None

### Bugfixes
- Provisionee support added to Beacon example
- Beacon Example: Fixed beacon enable serial command
- Provisioner:  Fixed OOB authentication procedure
- Device Reset: Fixed 1 dropped packet on startup

### Other / Documentation
- SDK coexistence guide updated for better explanation of coexistence with other Nordic SDKs for concurrent GATT/GAP (and other usage)
- DFU quick start guide fixed along with device_page_generator.py script
- SDK patch file updated
- Serial Example documentation updated
- Mesh Assert cleanup

### Verification / Test Errata
- nRF51 platform testing has been put on hold


## BLE Mesh v1.0.0

This is the first production release of Nordic's nRF5 SDK for Mesh. This release implements mandatory features for the Mesh Profile 1.0 specification and also some proprietary features (PB-remote and Nordic Advertiser Extensions) in experimental state.

### New features
- Key refresh has been implemented
- Heartbeat support has been added
- DFU bootloader source code added
- Support for nRF52840_xxAA
- Proprietary Nordic Advertiser Extensions ("InstaBurst!") feature for improved throughput
- AD-listener module for simplified subscription to advertisement data
- IRQ levels are now aligned with the nRF5 SDK
    - All API-functions are expected to be called from `NRF_MESH_IRQ_PRIORITY_LOWEST`
- Node de-provisioning supported through the Config Server Node Reset message

### Bugfixes
- Fixed beacon advertiser not enabled in example
- Fixed trailing garbage data in Composition Data
- Build fails if the path to the repo has spaces
- Fixed issue where the serial interface gets corrupted memory or hangs on an incoming serial packet that is too long
- Fixed issue where the Config Server used the application key index and not the DSM handle when deleting key
- Removed duplication of Company ID
- Fixed assertion on invalid length for unprovisioned beacons
- Removed warnings and errors from documentation build

### Other
- Provisioning API changed slightly to de-couple the upper and lower layers more cleanly

### Known limitations
- Optional features of the Mesh Profile 1.0 Specification are not part of this release.

### Test Errata
- DFU : replacing Softdevice live while running app is not tested.


## BLE Mesh v0.10.1-Alpha

This is a hotfix release with no new features.

### Bugfixes

- Segger Embedded Studio Projects have devkit BOARD defines now instead of dongles.
- Standard BLE Access Address now used for all binary artifacts (.hex/.lib)


## BLE Mesh v0.10.0-Alpha

This is a minor feature release for the experimental nRF5 SDK for Mesh

### New features

- Health Model

- Reworked build system
    - Support for nRF52840 (but not part of the integration testing)
    - Support for new SoftDevices S132 versions 4.0.4 and 5.0.0 and S140 version 5.0.0-3.alpha
    - Aligned with the nRF5 SDK version 14. Note: A subset of the nRF5 SDK is included in the nRF5 SDK for Mesh with some minor changes. See `external/nRF5_SDK_14.0.0_3bcc1f7/nRF5_SDK_14.0.0_3bcc1f7.patch` for the changes made.

- Refactored Bearer Layer and Core for increased bandwidth and robustness
    - Any number of concurrent advertisers
    - Custom radio implementations possible
    - Improved SAR reliability and throughput
    - TX Complete events fully supported
    - Significant reduction in packet processing time
    - More efficient radio code, major throughput improvements in noisy conditions

- Access Layer loopback, messages sent between models on the same device will now be short-circuited through the access layer
- RSSI and timestamp information is now available in the `access_message_rx_meta_t` and used in the Light switch example
- New filter engine for scanner
- Scanner filters: AD type, advertisement packet type, GAP address, RSSI
- Support for user-defined packet filters
- Each example project now has its own Segger Embedded Studio project file

### Bugfixes

- SAR receiving segments with TTL=0 should reply with SegAck TTL=0
- Bug in validation of command line options in device_page.py
- "family" undefined in reset_device() in bootloader_verify.py
- Access application key bitfield needs to hold device keys also
- DSM clears regular address when deleting virtual address
- Reliable parameter unset in access:packet_tx()
- Various bugfixes

### Document updates
- Revised documentation for better distinction between implementation-specific information and concepts defined by the Bluetooth Mesh Profile Specification, updated references to the latter
- Updated DFU Quick Start guide
- Added more detailed installation instructions

### Other
- "Light control example" has been renamed "Light switch example" to resolve similarity with the Light Control model in the Mesh Model Bluetooth Specification
- Step-by-step howto for setting up Keil projects for Mesh now available on DevZone: https://devzone.nordicsemi.com/blogs/1180/creating-a-keil-project-for-a-bluetooth-mesh-examp/

### Known limitations
- Heartbeat feature is not supported




## BLE Mesh v0.9.2-Alpha

This is a hotfix release, providing critical bug fixes and improvements.

### New features

- nrf_mesh_packet_send() now supports the reliable feature. I.e., it is possible to send single segments messages using the transport layer SAR.
- Interactive PyACI has support for an interactive provisioner and provisionee
- New serial interface event "Prov Failed"

### Bug fixes

- Provisionee not handling invalid provisioning data properly
- Problems using "Release" configuration in SES examples
- Incorrect usage of hal_led_blink_ms() in light control server
- Serial buffers must be word aligned
- Number of elements not handled in Serial interface's "Capabilities set" command
- S110 build failure
- Default build type is set in CMake
- Word alignment problems caused by high optimization levels when using Segger Embedded Studio
- PB-remote opcodes overlapping with Configuration model opcodes
- Advertisement bearer used timer_scheduler contexts dangerously, potentially corrupting its internal linked list
- PB-remote server would get confused about out-of-order ACKs from the client
- Documentation has been updated

### Document updates

- Rename "Bluetooth Mesh SDK" to "nRF5 SDK for Bluetooth Mesh"
- How to create a model document updated to current API
- Minor fixes to cryptography section in the "Bluetooth Mesh basic concepts" document
- Added installation instructions for CMake, Ninja and ARM toolchain
- Added information on how to select compile target

### Other

- Serial handlers split into separate source file
- Fix unit tests to work with the public CMock
- Fix packet formats and application nonce for compliance
    - Update unit tests with new sample data
- Several minor bugfixes
- Serial interface "Init Context" command removed. Provisioning context initialization is managed automatically.

### Known limitations of this release

- Heartbeat and Health model not implemented
- Source for bootloader must be downloaded from OpenMesh gitHub repo
- No power down state storage




## BLE Mesh v0.9.1-Alpha

This is an experimental release for exploration of the BLE Mesh stack on the nRF5 device family. It is not intended for commercial use.

### Key Features

- Bluetooth Mesh software core stack
    - Based on Bluetooth 4.0 PHY
    - Bearer, network, transport, and access layers
    - Foundation models
- Support for Node and Relay Node roles
    - Configurable scanning interval and duty cycle (from 3ms-10240ms)
    - Configurable advertisement interval (from 20ms-10240ms
- Example applications and proprietary models
- Broadcast flooding mesh
    - Theoretically up to 32,000 nodes
    - No routing tables
    - No single point of failure
    - Node-to-node and node-to-group communication
- Two-layer 128-bit AES-CCM network and node-to-node security
- Provisioning support
    - Standard "local" provisioning over advertisement bearer
    - Proprietary "remote" provisioning via relaying nodes (implemented as proprietary model)
- Persistent (flash) storage of configuration data
- Support for concurrent beaconing (separate API, Eddystone, iBeacon)
- Python shell-based test and demo framework for PC
- Support for over-the-air secure background DFU
    - Application and/or stacks
- Cross-platform toolchain support
    - Segger Embedded Studio
    - GCC and armcc
    - Windows, Linux and macOS

### Bugfixes in this release
- AD types updated to use Bluetooth SIG allocated numbers
- Proprietary On/Off model example renamed from "Generic On/Off" to "Simple On/Off"
- Minor DFU bugfixes and improvements

### Other

- Revised and reorganized documentation.

### Compatibility

- nRF51 and nRF52
- Recommended >= 200 kB Flash and >= 32 kB RAM
- nRF5 SDK (v10.0.0 for S110, v12.1.0 for S130 and S132)
- Segger Embedded Studio v3.22

### Known limitations of this release

- Packet format in Alpha is not entirely BLE Mesh compatible
- Heartbeat and Health model not implemented
- ASZMIC field is missing from application/device nonce for segmented access messages
- Source for bootloader must be downloaded from OpenMesh GitHub repo
- No power down state storage


## BLE Mesh v0.9.0

### Bugfixes

- DSM for-loop overflow bug. Looped over number of application keys rather than network keys. Caused fault on reception of secure mesh network beacons.
- IRQ level for nRF52 set too low in light control example. Caused hardfault on button press.
- Debounce algorithm for button presses in light control example used `TIMER_OLDER_THAN()` when it should have used `TIMER_DIFF()`. Made buttons unresponsive after ~1h.
- Replay protection cache checking wrong address
- Network cache not caching messages if transport layer fails
- Serial provisioning interface not setting all parameters of the provisioning struct

### New features
- Configuration server
- Configuration client
- light control example uses configuration client+server to configure the newly provisioned devices.
    - Also supports group addressing
- New host scripts for Serial ACI. Auto-generated the same way as documentation is.
- Persistent storage manager
    - Integrates with access layer and the Device State Manager
- Persistent storage integration with examples
- Node configuration module for provisionee applications
- Use asynchronous event flags with bearer event module


### Other

- Updated documentation for PB-remote example
- Added README for light control example
- Added Howto for creating proprietary models

### Known limitations

- No generated documentation or host scripts for model specific commands in the Serial ACI. This affects mostly the PB-remote client.
- In bootloader mode all DFU transfers will be accepted regardless of the FWID.
- BLE Mesh non-compliance (will be fixed in upcoming releases):
    - "Generic OnOff" model example is not complete nor spec compliant
    - Opcodes do not follow specification
    - Bluetooth SIG assigned numbers for AD_Types, Service UUIDs and Characteristic UUIDs are not up-to-date
    - Health Model and Heartbeat features are not implemented
    - Packet format changes related to removal of MD bit are not implemented

## BLE Mesh v0.8.1

### Release notes
- PB-remote server and client updated for new access layer
- Bugfixes for new access layer and device state manager
- Asynchronous event flags
  - Timer scheduler
  - Serial
- Serial integration
  - Device state manager
  - Access layer
- Transport layer device key decryption bug fix
- Adds support for PB-remote server disabling (issue #66)
- Adds a Simple OnOff demo model
- Network beacon initialization added (was missing in v 0.8.0)

### Known issues
- Per-beacon TX power settings are not yet functional for the serial interface
- The transport layer still defaults to +malloc()+ to allocate SAR buffers
- IV update will trigger without checking the value of the received iv index
- Interactive_pyaci is not up-to-date with the latest serial interface

## BLE Mesh v0.8.0

This release features a preview of the refactored nRF Mesh API and new key modules.

### Release highlights

- Device State Manager
  - Manages all mesh-related keys and addresses for the user
  - Reduces key-storage memory footprint
- Access layer
  - Reworked API to fit more usage scenarios
  - Reduced memory footprint
  - Now takes ownership of the model database, in preparation for persistent storage support
- New application configuration module for the Device State Manager and access layer pool sizes
  - Application specified compile-time configuration of memory usage for the upper layers
- Reworked serial interface for use with the Device State Manager
  - Added optional SLIP encoding for serial UART packets
  - Support for setting custom beacon data over serial
  - Improved documentation generator, now including command responses
  - Implemented GAP address get command (issue #38)
  - Reduced memory footprint
- Improved IV update procedure
  - Improved quality of service for mesh messages
  - Fully spec compliant
- DFU procedure improvements
  - Improved reliability
  - Packet loss recovery procedure has lower impact on the rest of the transfer
  - Fix for GitHub issue #77: Timeout in DFU ready phase

### Known issues and limitations

- Serial opcodes are not backwards compatible
- PB-Remote is not integrated with the new access layer
  - It is not supported by the serial interface either
- Per-beacon TX power settings are not yet functional for the serial interface
- The transport layer still defaults to +malloc()+ to allocate SAR buffers



## BLE Mesh v0.7.7

- Documentation improvements
- Renamed some modules to prevent using Bluetooth SIG's confidential identifiers
- Bug fixes
  - A corner-case bug where the provisioning complete ACK is lost but a successful close packet
    is sent is now fixed
  - Fixed incorrect endianness in address check (issue #74)

### Notes
The transport SAR uses +malloc()+ to allocate buffers for SAR transactions. This behaviour
can be overridden using +transport_sar_mem_funcs_set()+, otherwise +__HEAPSIZE+ needs to be
defined.

**WARNING:** SoftDevice needs to be Flashed without memory protection

