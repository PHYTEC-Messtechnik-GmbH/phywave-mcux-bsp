# BLE ePaper Example

This project is located in `examples/phynode_ble/`.

This example demonstrates the usage of Bluetooth Low Energy in conjunction with
the onboard ePaper display. It is highly recommended to download the [official
phyNODE Android
app](https://play.google.com/store/apps/details?id=de.phytec.iot.phynode) to use
the phyNODE board with this example.

## Usage

To make the phyNODE advertising and thus discoverable by other devices press the
user button S2. Then you can connect to it by using the [phyNODE Android
app](https://play.google.com/store/apps/details?id=de.phytec.iot.phynode) or a
Bluetooth Low Energy tool such as `gatttool` or `bluetoothctl`.

The phyNODE's display is interfaced via the following BLE characteristics:

- `f000aa21-0451-4000-b000-000000000000` Initiate the display for a write. One
  packet should be 1 byte long containing the value `0x00`.
- `f000aa22-0451-4000-b000-000000000000` This UUID points to the display buffer.
  One packet should be 16 bytes long containing data of one display column
  (16 bytes = 128 bit = 128 pixels).
- `f000aa23-0451-4000-b000-000000000000` Update the display and show the written
  data. One packet should be 1 byte long containg the value `0x00`.

## FAQ for developing your own BLE application

### NXP Documentation on Bluetooth Low Energy
NXP provides extensive documentation for writing your own Bluetooth Low Energy
applications inside their [MCUXpresso SDK](https://mcuxpresso.nxp.com) in
`docs/wireless/Bluetooth`.

### Setting the BLE Address
To change the MAC address open `app_preinclude.h` and edit BD_ADDR macro:
```c
#define BD_ADDR  0x00,0x53,0x00,0x5E,0x00,0x00
```
Note that the address is entered in LSB to MSB order.

### Enabling Notifaction Callbacks for Custom Services
In order to get a callback when a custom service and its characteristics are
written or read one has to register their handles to be notifiable:
```c
/* value_epaper is the name of the handle created with VALUE() in your
 * gatt_db.h */
uint16_t handleArray[] = { value_epaper };
uint8_t handleCount = sizeof(handleArray) / sizeof(uint16_t);

/* call the GATT server callback function when an ATT WRITE occurs */
GattServer_RegisterHandlesForWriteNotifications(handleCount,
                (uint16_t *) &handleArray[0]);

/* call the GATT server callback function when an ATT READ occurs */
GattServer_RegisterHandlesForReadNotifications(handleCount,
                (uint16_t *) &handleArray[0]);
```
This also requires a custom GATT server callback function which should be
registered with:
```c
App_RegisterGattServerCallback(my_custom_gatt_server_callback);
```
