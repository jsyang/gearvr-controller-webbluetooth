class ControllerBluetoothInterface {
    constructor(onControllerDataReceived, onDeviceDisconnected) {
        this.gattServer               = null;
        this.batteryService           = null;
        this.deviceInformationService = null;
        this.customService            = null;
        this.customServiceNotify      = null;
        this.customServiceWrite       = null;

        this.pair       = this.pair.bind(this);
        this.disconnect = this.disconnect.bind(this);
        this.runCommand = this.runCommand.bind(this);

        this.onDeviceConnected      = this.onDeviceConnected.bind(this);
        this.onNotificationReceived = this.onNotificationReceived.bind(this);

        if (onDeviceDisconnected) {
            this.onDeviceDisconnected = onDeviceDisconnected.bind(this);
        }

        if (onControllerDataReceived) {
            this.onControllerDataReceived = onControllerDataReceived.bind(this);
        }
    }

    onDeviceConnected(device) {
        if (this.onDeviceDisconnected) {
            device.addEventListener('gattserverdisconnected', onDeviceDisconnected);
        }

        return device.gatt.connect();
    }

    pair() {
        return navigator.bluetooth.requestDevice({
            acceptAllDevices: true,
            optionalServices: [
                ControllerBluetoothInterface.UUID_CUSTOM_SERVICE
            ]
        })
            .then(this.onDeviceConnected)
            .then(gattServer => this.gattServer = gattServer)

            // Get custom service
            .then(() => this.gattServer.getPrimaryService(ControllerBluetoothInterface.UUID_CUSTOM_SERVICE))
            .then(customService => this.customService = customService)

            //todo: battery service, device information service

            .then(() => this.customService
                .getCharacteristic(ControllerBluetoothInterface.UUID_CUSTOM_SERVICE_WRITE)
                .then(characteristic => this.customServiceWrite = characteristic))

            .then(() => this.customService
                .getCharacteristic(ControllerBluetoothInterface.UUID_CUSTOM_SERVICE_NOTIFY)
                .then(characteristic => this.customServiceNotify = characteristic))

            .then(() => this.customServiceNotify
                .startNotifications()
                .then(() => this.customServiceNotify.addEventListener('characteristicvaluechanged', this.onNotificationReceived)))
            ;
    }

    disconnect() {
        this.gattServer && this.gattServer.disconnect();
    }

    onNotificationReceived(e) {
        const {buffer}  = e.target.value;
        const eventData = new Uint8Array(buffer);

        // Max observed value = 315
        // (corresponds to touchpad sensitive dimension in mm)
        const axisX = (
            ((eventData[54] & 0xF) << 6) +
            ((eventData[55] & 0xFC) >> 2)
        ) & 0x3FF;

        // Max observed value = 315
        const axisY = (
            ((eventData[55] & 0x3) << 8) +
            ((eventData[56] & 0xFF) >> 0)
        ) & 0x3FF;

        // com.samsung.android.app.vr.input.service/ui/c.class:L222
        const timestamp = ((new Int32Array(buffer.slice(0, 3))[0]) & 0xFFFFFFFF) / 1000 * ControllerBluetoothInterface.TIMESTAMP_FACTOR;

        // com.samsung.android.app.vr.input.service/ui/c.class:L222
        const temperature = eventData[57];

        const {
                  getAccelerometerFloatWithOffsetFromArrayBufferAtIndex,
                  getGyroscopeFloatWithOffsetFromArrayBufferAtIndex,
                  getMagnetometerFloatWithOffsetFromArrayBufferAtIndex
              } = ControllerBluetoothInterface;

        // 3 x accelerometer and gyroscope x,y,z values per data event
        const accel = [
            getAccelerometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 4, 0),
            getAccelerometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 6, 0),
            getAccelerometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 8, 0),
            // getAccelerometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 4, 1),
            // getAccelerometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 6, 1),
            // getAccelerometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 8, 1),
            // getAccelerometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 4, 2),
            // getAccelerometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 6, 2),
            // getAccelerometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 8, 2)
        ].map(v => v * ControllerBluetoothInterface.ACCEL_FACTOR);

        const gyro = [
            getGyroscopeFloatWithOffsetFromArrayBufferAtIndex(buffer, 10, 0),
            getGyroscopeFloatWithOffsetFromArrayBufferAtIndex(buffer, 12, 0),
            getGyroscopeFloatWithOffsetFromArrayBufferAtIndex(buffer, 14, 0),
            // getGyroscopeFloatWithOffsetFromArrayBufferAtIndex(buffer, 10, 1),
            // getGyroscopeFloatWithOffsetFromArrayBufferAtIndex(buffer, 12, 1),
            // getGyroscopeFloatWithOffsetFromArrayBufferAtIndex(buffer, 14, 1),
            // getGyroscopeFloatWithOffsetFromArrayBufferAtIndex(buffer, 10, 2),
            // getGyroscopeFloatWithOffsetFromArrayBufferAtIndex(buffer, 12, 2),
            // getGyroscopeFloatWithOffsetFromArrayBufferAtIndex(buffer, 14, 2)
        ].map(v => v * ControllerBluetoothInterface.GYRO_FACTOR);

        const magX = getMagnetometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 0);
        const magY = getMagnetometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 2);
        const magZ = getMagnetometerFloatWithOffsetFromArrayBufferAtIndex(buffer, 4);

        const triggerButton    = Boolean(eventData[58] & (1 << 0));
        const homeButton       = Boolean(eventData[58] & (1 << 1));
        const backButton       = Boolean(eventData[58] & (1 << 2));
        const touchpadButton   = Boolean(eventData[58] & (1 << 3));
        const volumeUpButton   = Boolean(eventData[58] & (1 << 4));
        const volumeDownButton = Boolean(eventData[58] & (1 << 5));

        this.onControllerDataReceived({
            accel,
            gyro,

            magX, magY, magZ,

            timestamp,
            temperature,
            axisX, axisY,
            triggerButton,
            homeButton,
            backButton,
            touchpadButton,
            volumeUpButton,
            volumeDownButton
        });
    }

    runCommand(commandValue) {
        const {getLittleEndianUint8Array, onBluetoothError} = ControllerBluetoothInterface;

        return this.customServiceWrite.writeValue(getLittleEndianUint8Array(commandValue))
            .catch(onBluetoothError);
    }
}

ControllerBluetoothInterface.onBluetoothError = e => {
    console.warn('Error: ' + e);
};

ControllerBluetoothInterface.UUID_CUSTOM_SERVICE        = "4f63756c-7573-2054-6872-65656d6f7465";
ControllerBluetoothInterface.UUID_CUSTOM_SERVICE_WRITE  = "c8c51726-81bc-483b-a052-f7a14ea3d282";
ControllerBluetoothInterface.UUID_CUSTOM_SERVICE_NOTIFY = "c8c51726-81bc-483b-a052-f7a14ea3d281";

ControllerBluetoothInterface.CMD_OFF                          = '0000';
ControllerBluetoothInterface.CMD_SENSOR                       = '0100';
ControllerBluetoothInterface.CMD_UNKNOWN_FIRMWARE_UPDATE_FUNC = '0200';
ControllerBluetoothInterface.CMD_CALIBRATE                    = '0300';
ControllerBluetoothInterface.CMD_KEEP_ALIVE                   = '0400';
ControllerBluetoothInterface.CMD_UNKNOWN_SETTING              = '0500';
ControllerBluetoothInterface.CMD_LPM_ENABLE                   = '0600';
ControllerBluetoothInterface.CMD_LPM_DISABLE                  = '0700';
ControllerBluetoothInterface.CMD_VR_MODE                      = '0800';

ControllerBluetoothInterface.GYRO_FACTOR      = 0.0001; // to radians / s
ControllerBluetoothInterface.ACCEL_FACTOR     = 0.00001; // to g (9.81 m/s**2)
ControllerBluetoothInterface.TIMESTAMP_FACTOR = 0.001; // to seconds

ControllerBluetoothInterface.getAccelerometerFloatWithOffsetFromArrayBufferAtIndex = (arrayBuffer, offset, index) => {
    const arrayOfShort = new Int16Array(arrayBuffer.slice(16 * index + offset, 16 * index + offset + 2));
    return (new Float32Array([arrayOfShort[0] * 10000.0 * 9.80665 / 2048.0]))[0];
};

ControllerBluetoothInterface.getGyroscopeFloatWithOffsetFromArrayBufferAtIndex = (arrayBuffer, offset, index) => {
    const arrayOfShort = new Int16Array(arrayBuffer.slice(16 * index + offset, 16 * index + offset + 2));
    return (new Float32Array([arrayOfShort[0] * 10000.0 * 0.017453292 / 14.285]))[0];
};

ControllerBluetoothInterface.getMagnetometerFloatWithOffsetFromArrayBufferAtIndex = (arrayBuffer, offset) => {
    const arrayOfShort = new Int16Array(arrayBuffer.slice(32 + offset, 32 + offset + 2));
    return (new Float32Array([arrayOfShort[0] * 0.06]))[0];
};

ControllerBluetoothInterface.getLength = (f1, f2, f3) => Math.sqrt(f1 ** 2 + f2 ** 2 + f3 ** 2);

ControllerBluetoothInterface.getLittleEndianUint8Array = hexString => {
    const leAB = new Uint8Array(hexString.length >> 1);

    for (let i = 0, j = 0; i + 2 <= hexString.length; i += 2, j++) {
        leAB[j] = parseInt(hexString.substr(i, 2), 16);
    }

    return leAB;
};
