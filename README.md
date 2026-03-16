# WS281xReceiver
A library to decode and forward the WS281x protocol.

## Overview

This library provides classes for handling WS281x LED signals on RP2040 microcontrollers, enabling reception, processing, and transmission of LED data.

### Classes

* **LEDReceiver**: This class handles the reception and processing of LED data from WS281x signals. It manages the connection state (Online, Offline, Error, DataMissing, Unknown), detects data changes, and provides methods to access the current state and check for data updates. The class uses a WS281xProcessor instance for low-level signal processing and includes mechanisms for reconnection and error handling.

* **WS281xProcessor**: Class for processing WS281x LED signals on RP2040 microcontrollers. It handles signal reception and repetition using PIO state machines, manages DMA transfers for efficient data gathering, and provides callback mechanisms for data reception and error handling.

* **WS2812Sender**: Class for sending WS2812 LED data using PIO on RP2040 microcontrollers. It initializes a PIO state machine to output LED signals on a specified pin and provides methods to set the color of a status LED in RGB or HSV formats.

# License / Lizenz #
EN: This module is licensed under the Business Source License 1.1 (BSL). Non-commercial use is permitted. Commercial use requires prior authorization. On December 31 2035 this code will convert to LGPL-2.1. See LICENSE.txt for details.

DE: Dieses Modul ist unter der Business Source License 1.1 (BSL) lizenziert. Eine Nutzung für nicht-kommerzielle Zwecke ist gestattet. Für kommerzielle Nutzungen ist eine Genehmigung erforderlich. Am 31. Dezember 2035 wird dieser Code automatisch unter die LGPL-2.1 gestellt. Details finden Sie in der LICENSE.txt Datei.
