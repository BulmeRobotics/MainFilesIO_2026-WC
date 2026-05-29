#include "BLE.h"
//@author Vincent Rohkamm
//@date 07.07.2025
//@brief 

#ifdef _MSC_VER
#pragma region PRIVATE FUNCTIONS
#endif

void BLE_UART::pollBLE(){
    if ((!_CONNECTED) || (!peripheral.connected())) {
        _CONNECTED = false;
        return;
    }

    if (rxCharacteristic.valueUpdated()) {
        const uint8_t* data = rxCharacteristic.value();
        int len = rxCharacteristic.valueLength();
        
        for (int i = 0; i < len; i++) {
            uint16_t next = (_rx_head + 1) % RX_BUFFER_SIZE;
            if (next != _rx_tail) {
                _rx_buffer[_rx_head] = data[i];
                _rx_head = next;
            }
        }
    }
}


#ifdef _MSC_VER
#pragma endregion PRIVATE FUNCTIONS
#pragma region PUBLIC FUNCTIONS
#endif

ErrorCodes BLE_UART::init(UserInterface* ui){
    _ui = ui;
    if(!BLE.begin()){   //Begin BLE
        if(_debug_ifc != nullptr) _debug_ifc->println("BLE init failed");
        return ErrorCodes::no_connection;
    }
    return ErrorCodes::OK;
}

ErrorCodes BLE_UART::connect(){
    if(_ui != nullptr) _ui->ShowCalibrationScreen(PoI_Type::ble);

    uint32_t startTime = millis();

    BLE.scanForUuid(serviceUUID);

    

    while (startTime + timeoutTime > millis()){
        if(_ui != nullptr) _ui->UpdateCalibrationProgress(1,6);
        //--- SCAN --- 
        peripheral = BLE.available();   //Check if BLE peripheral is available

        if(!peripheral) continue;

        //Debug Info
        if(_debug_ifc != nullptr){
                _debug_ifc->print("Found BLE device: ");
                _debug_ifc->print(peripheral.address());
                _debug_ifc->print(" '");
                _debug_ifc->print(peripheral.localName());
                _debug_ifc->print("' ");
                _debug_ifc->print(peripheral.advertisedServiceUuid());
                _debug_ifc->println();
            }
        
        if(_ui != nullptr) _ui->UpdateCalibrationProgress(2,6);

        if(peripheral.localName() != LOCAL_NAME){
            if(_debug_ifc != nullptr) _debug_ifc->println("Wrong device name");
            if(_ui != nullptr) _ui->FinishCalibration(false);
            return ErrorCodes::invalid;
        }

        BLE.stopScan();

        // --- Connect ---
        if(peripheral.connect()){
            if(_debug_ifc != nullptr) _debug_ifc->println("connected to BLE device");
            if(_ui != nullptr) _ui->UpdateCalibrationProgress(3,6);
        } else {
            if(_debug_ifc != nullptr) _debug_ifc->println("connection failed!");
            if(_ui != nullptr) _ui->FinishCalibration(false);
            return ErrorCodes::no_connection;
        }

        // --- Discover Attributes ---
        if(peripheral.discoverAttributes()){
            if(_ui != nullptr) _ui->UpdateCalibrationProgress(4,6);
        } else {
            if(_debug_ifc != nullptr) _debug_ifc->println("Attribute Error");
            if(_ui != nullptr) _ui->FinishCalibration(false);
            return ErrorCodes::no_connection;
        }

        // --- Check Characteristics ---
        rxCharacteristic = peripheral.characteristic(rxUUID);
        txCharacteristic = peripheral.characteristic(txUUID);

            //Check RX
        if(!rxCharacteristic){
            if(_debug_ifc != nullptr) _debug_ifc->println("No rx characteristic!");
            peripheral.disconnect();
            if(_ui != nullptr) _ui->FinishCalibration(false);
            return ErrorCodes::invalid;
        } else if(!rxCharacteristic.canSubscribe()){
            if(_debug_ifc != nullptr) _debug_ifc->println("RX cannot Subscribe");
            peripheral.disconnect();
            if(_ui != nullptr) _ui->FinishCalibration(false);
            return ErrorCodes::invalid;
        } else if(!rxCharacteristic.subscribe()){
            if(_debug_ifc != nullptr) _debug_ifc->println("RX Subscribe failed");
            peripheral.disconnect();
            if(_ui != nullptr) _ui->FinishCalibration(false);
            return ErrorCodes::invalid;
        }

        if(_ui != nullptr) _ui->UpdateCalibrationProgress(5,6);
        if(_debug_ifc != nullptr) _debug_ifc->println("rx connected");

            //Check TX
        if(!txCharacteristic){
            if(_debug_ifc != nullptr) _debug_ifc->println("No tx characteristic!");
            peripheral.disconnect();
            if(_ui != nullptr)_ui->FinishCalibration(false);
            return ErrorCodes::invalid;
        } else if(!txCharacteristic.canWrite()){
            if(_debug_ifc != nullptr) _debug_ifc->println("No Write tx!");
            peripheral.disconnect();
            if(_ui != nullptr)_ui->FinishCalibration(false);
            return ErrorCodes::invalid;
        }

        //Finish
        if(_ui != nullptr)_ui->UpdateCalibrationProgress(6,6);
        if(_debug_ifc != nullptr) {_debug_ifc->println("tx connected"); _debug_ifc->println("Finished connect!");}

        _rx_head = 0;
        _rx_tail = 0;

        _CONNECTED = true;
        if(_ui != nullptr)_ui->FinishCalibration(true);
        return ErrorCodes::OK;
    }
    if(_ui != nullptr)_ui->FinishCalibration(false);
    return ErrorCodes::TIMEOUT;
}

// --- STREAM ---
int BLE_UART::available() {
    pollBLE(); // check for new data
    return (_rx_head - _rx_tail + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;
}

int BLE_UART::read() {
    pollBLE();
    if (_rx_head == _rx_tail) return -1; // Puffer leer
    uint8_t c = _rx_buffer[_rx_tail];
    _rx_tail = (_rx_tail + 1) % RX_BUFFER_SIZE;

    if(_debug_ifc != nullptr) _debug_ifc->println(c);
    return c;
}

int BLE_UART::peek() {
    pollBLE();
    if (_rx_head == _rx_tail) return -1;
    return _rx_buffer[_rx_tail];
}

size_t BLE_UART::write(uint8_t c) {
    return write(&c, 1);
}

size_t BLE_UART::write(const uint8_t *buffer, size_t size) {
    if ((!_CONNECTED) || (!peripheral.connected())) return 0;
    
    size_t sent = 0;
    while(size > 0){
        size_t chunk = (size > 20) ? 20 : size;
        if(txCharacteristic.writeValue(buffer + sent, chunk)) {
            sent += chunk;
            size -= chunk;
        } else {
            break; // send failed
        }
    }
    return sent;
}

void BLE_UART::flush() {}

#ifdef _MSC_VER
#pragma endregion PUBLIC FUNCTIONS
#endif