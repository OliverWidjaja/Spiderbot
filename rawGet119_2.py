import asyncio
import threading
from bleak import BleakClient
from pyvesc.protocol.interface import encode_request, decode
from pyvesc.VESC.messages import GetValues

# Bluetooth configuration
ADDRESS = "CF:9D:22:EF:60:F9"
RX_CHARACTERISTIC = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_CHARACTERISTIC = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# CAN ID
SECONDARY_CAN_ID = 119  # Only query this VESC

class BluetoothVESC:    
    def __init__(self, client, rx_characteristic, tx_characteristic):
        self.client = client
        self.rx_characteristic = rx_characteristic
        self.tx_characteristic = tx_characteristic
        self.secondary_values = None
        self._stop_requested = False
        self._values_lock = threading.Lock()
        self._buffer = bytearray()
        
        # Store message info for getting values from secondary VESC only
        self._get_secondary_values_msg = encode_request(GetValues(can_id=SECONDARY_CAN_ID))
        print(f"CAN ID 119 request message: {self._get_secondary_values_msg.hex()}")
    
    async def start_continuous_read(self):
        """Start continuously reading VESC values from CAN ID 119 only"""
        self._stop_requested = False
        asyncio.create_task(self._continuous_read_task())
    
    async def stop_continuous_read(self):
        """Stop the continuous reading task"""
        self._stop_requested = True
    
    async def _continuous_read_task(self):
        """Background task to continuously read VESC values from CAN ID 119"""
        while not self._stop_requested:
            try:
                # Only request secondary VESC values (CAN ID 119)
                await self.client.write_gatt_char(self.rx_characteristic, self._get_secondary_values_msg)
                print("Sent request to VESC CAN ID 119")
                
                await asyncio.sleep(0.1)  # 100ms between requests
                
            except Exception as e:
                print(f"Error in continuous read: {e}")
                await asyncio.sleep(0.1)
    
    async def notification_handler(self, sender, data):
        """Handle incoming notifications (responses from VESCs)"""
        try:
            print(f"Raw data received ({len(data)} bytes): {data.hex()}")
            
            # Add new data to buffer
            self._buffer.extend(data)
            
            # Try to decode complete messages from buffer
            while len(self._buffer) > 0:
                response, consumed = decode(bytes(self._buffer))
                
                if consumed > 0:
                    print(f"Decoded message, consumed {consumed} bytes")
                    print(f"Response type: {type(response)}")
                    
                    if response is not None:
                        # Print ALL attributes of the response for debugging
                        print("=== FULL RESPONSE OBJECT ===")
                        attrs = [attr for attr in dir(response) if not attr.startswith('_') and not callable(getattr(response, attr))]
                        for attr in sorted(attrs):
                            try:
                                value = getattr(response, attr)
                                print(f"  {attr}: {value} (type: {type(value)})")
                            except Exception as e:
                                print(f"  {attr}: <error reading: {e}>")
                        
                        # Check for CAN ID in multiple possible fields
                        can_id = getattr(response, 'can_id', None)
                        app_controller_id = getattr(response, 'app_controller_id', None)
                        
                        print(f"CAN ID from can_id field: {can_id}")
                        print(f"app_controller_id: {app_controller_id}")
                        
                        # Try to extract CAN ID from app_controller_id
                        detected_can_id = None
                        if app_controller_id and len(app_controller_id) > 0:
                            # Convert the first byte of app_controller_id to integer
                            detected_can_id = app_controller_id[0]
                            print(f"CAN ID detected from app_controller_id: {detected_can_id}")
                        
                        with self._values_lock:
                            if detected_can_id == SECONDARY_CAN_ID:
                                self.secondary_values = response
                                print(f"✓ SUCCESS: Updated VESC CAN ID {detected_can_id} values")
                            elif can_id == SECONDARY_CAN_ID:
                                self.secondary_values = response
                                print(f"✓ SUCCESS: Updated VESC CAN ID {can_id} values")
                            else:
                                print(f"✗ UNEXPECTED: Received response from CAN ID {detected_can_id or can_id}, expected {SECONDARY_CAN_ID}")
                    
                    else:
                        print("✗ Decoded response is None")
                    
                    # Remove consumed bytes from buffer
                    self._buffer = self._buffer[consumed:]
                    print(f"Buffer remaining: {len(self._buffer)} bytes")
                    
                else:
                    # Not enough data for a complete message
                    print(f"Not enough data for complete message. Buffer: {len(self._buffer)} bytes")
                    break
                    
        except Exception as e:
            print(f"Error in notification handler: {e}")
            import traceback
            traceback.print_exc()
            self._buffer.clear()  # Clear buffer on error
    
    def get_secondary_position(self):
        """Get the current motor position from secondary VESC"""
        with self._values_lock:
            if self.secondary_values is not None and hasattr(self.secondary_values, 'pid_pos_now'):
                return self.secondary_values.pid_pos_now
        return None
    
    def get_secondary_rpm(self):
        """Get the current RPM from secondary VESC"""
        with self._values_lock:
            if self.secondary_values is not None and hasattr(self.secondary_values, 'rpm'):
                return self.secondary_values.rpm
        return None
    
    def get_secondary_voltage(self):
        """Get the input voltage from secondary VESC"""
        with self._values_lock:
            if self.secondary_values is not None and hasattr(self.secondary_values, 'v_in'):
                return self.secondary_values.v_in
        return None
    
    def get_all_secondary_values(self):
        """Get all current measurement values from secondary VESC"""
        with self._values_lock:
            return self.secondary_values

async def main():
    try:
        async with BleakClient(ADDRESS) as client:
            print(f"Connected to primary VESC")
            print(f"Querying ONLY VESC with CAN ID: {SECONDARY_CAN_ID}")
            
            # Create motor object
            motor = BluetoothVESC(client, RX_CHARACTERISTIC, TX_CHARACTERISTIC)
            
            # Set up notification handler
            await client.start_notify(TX_CHARACTERISTIC, motor.notification_handler)
            
            # Start continuous reading
            await motor.start_continuous_read()
            print("Continuous reading started for CAN ID 119 only")
            
            try:
                # Read values for 30 seconds to get more data
                for i in range(30):
                    # Secondary VESC values
                    secondary_pos = motor.get_secondary_position()
                    secondary_rpm = motor.get_secondary_rpm()
                    secondary_voltage = motor.get_secondary_voltage()
                    
                    # Handle None values safely
                    def format_value(value, format_str=None):
                        if value is None:
                            return "N/A"
                        elif format_str:
                            return format_str.format(value)
                        else:
                            return str(value)
                    
                    print(f"\n--- Iteration {i+1} ---")
                    print(f"VESC CAN ID {SECONDARY_CAN_ID}:")
                    print(f"  Position: {format_value(secondary_pos, '{:.3f}')}")
                    print(f"  RPM: {format_value(secondary_rpm)}")
                    print(f"  Voltage: {format_value(secondary_voltage)}V")
                    
                    # Print buffer status
                    print(f"Buffer size: {len(motor._buffer)} bytes")
                    
                    await asyncio.sleep(1)
                    
            except KeyboardInterrupt:
                print("Stopping...")
            
            finally:
                # Clean up
                await motor.stop_continuous_read()
                await client.stop_notify(TX_CHARACTERISTIC)
                
    except Exception as e:
        print(f"Connection failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())