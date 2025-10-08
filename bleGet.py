import asyncio
import threading
from bleak import BleakClient
from pyvesc.protocol.interface import encode_request, decode
from pyvesc.VESC.messages import GetValues

# Bluetooth configuration
ADDRESS = "CF:9D:22:EF:60:F9"
RX_CHARACTERISTIC = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_CHARACTERISTIC = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# CAN IDs
PRIMARY_CAN_ID = 12  # Primary VESC (no CAN ID)
SECONDARY_CAN_ID = 119  # Secondary VESC

class BluetoothVESC:    
    def __init__(self, client, rx_characteristic, tx_characteristic):
        self.client = client
        self.rx_characteristic = rx_characteristic
        self.tx_characteristic = tx_characteristic
        
        # Data storage with thread safety
        self.primary_values = None
        self.secondary_values = None
        self._values_lock = threading.Lock()
        self._buffer = bytearray()
        
        # Control flags
        self._stop_requested = False
        self._is_reading = False
        
        # Pre-encode messages for efficiency
        self._primary_request_msg = encode_request(GetValues())
        self._secondary_request_msg = encode_request(GetValues(can_id=SECONDARY_CAN_ID))
        
        print(f"Primary request message: {self._primary_request_msg.hex()}")
        print(f"Secondary request message: {self._secondary_request_msg.hex()}")
    
    async def start_continuous_read(self, read_primary=True, read_secondary=True):
        """Start continuously reading VESC values"""
        self._stop_requested = False
        self._is_reading = True
        
        if read_primary or read_secondary:
            asyncio.create_task(self._continuous_read_task(read_primary, read_secondary))
    
    async def stop_continuous_read(self):
        """Stop the continuous reading task"""
        self._stop_requested = True
        self._is_reading = False
    
    async def _continuous_read_task(self, read_primary, read_secondary):
        """Background task to continuously read VESC values"""
        request_interval = 0.05  # 50ms between requests
        
        while not self._stop_requested and self._is_reading:
            try:
                if read_primary:
                    await self.client.write_gatt_char(self.rx_characteristic, self._primary_request_msg)
                
                if read_secondary:
                    await asyncio.sleep(request_interval / 2)  # Small delay between requests
                    await self.client.write_gatt_char(self.rx_characteristic, self._secondary_request_msg)
                
                await asyncio.sleep(request_interval)
                
            except Exception as e:
                print(f"Error in continuous read: {e}")
                await asyncio.sleep(0.1)
    
    def _extract_can_id(self, response):
        """Extract CAN ID from response object"""
        # Try direct can_id field first
        can_id = getattr(response, 'can_id', None)
        if can_id is not None:
            return can_id
        
        # Try to extract from app_controller_id
        app_controller_id = getattr(response, 'app_controller_id', None)
        if app_controller_id and len(app_controller_id) > 0:
            return app_controller_id[0]
        
        return PRIMARY_CAN_ID  # Assume primary if no CAN ID found
    
    async def notification_handler(self, sender, data):
        """Handle incoming notifications (responses from VESCs)"""
        try:
            # Add new data to buffer
            self._buffer.extend(data)
            
            # Process all complete messages in buffer
            while len(self._buffer) > 0:
                response, consumed = decode(bytes(self._buffer))
                
                if consumed > 0:
                    if response is not None:
                        can_id = self._extract_can_id(response)
                        
                        with self._values_lock:
                            if can_id == PRIMARY_CAN_ID:
                                self.primary_values = response
                                # print(f"✓ Updated primary VESC values")
                            elif can_id == SECONDARY_CAN_ID:
                                self.secondary_values = response
                                # print(f"✓ Updated secondary VESC (CAN {can_id}) values")
                            else:
                                print(f"✗ Unexpected CAN ID: {can_id}")
                    
                    # Remove consumed bytes from buffer
                    self._buffer = self._buffer[consumed:]
                    
                else:
                    # Not enough data for complete message
                    break
                    
        except Exception as e:
            print(f"Error in notification handler: {e}")
            import traceback
            traceback.print_exc()
            self._buffer.clear()
    
    # Primary VESC getters
    def get_primary_position(self):
        """Get position from primary VESC"""
        with self._values_lock:
            if self.primary_values is not None:
                for attr in ['pid_pos_now', 'position', 'pos']:
                    if hasattr(self.primary_values, attr):
                        return getattr(self.primary_values, attr)
        return None
    
    def get_primary_rpm(self):
        """Get RPM from primary VESC"""
        with self._values_lock:
            if self.primary_values is not None and hasattr(self.primary_values, 'rpm'):
                return self.primary_values.rpm
        return None
    
    def get_primary_voltage(self):
        """Get voltage from primary VESC"""
        with self._values_lock:
            if self.primary_values is not None and hasattr(self.primary_values, 'v_in'):
                return self.primary_values.v_in
        return None
    
    def get_all_primary_values(self):
        """Get all values from primary VESC"""
        with self._values_lock:
            return self.primary_values
    
    # Secondary VESC getters
    def get_secondary_position(self):
        """Get position from secondary VESC"""
        with self._values_lock:
            if self.secondary_values is not None and hasattr(self.secondary_values, 'pid_pos_now'):
                return self.secondary_values.pid_pos_now
        return None
    
    def get_secondary_rpm(self):
        """Get RPM from secondary VESC"""
        with self._values_lock:
            if self.secondary_values is not None and hasattr(self.secondary_values, 'rpm'):
                return self.secondary_values.rpm
        return None
    
    def get_secondary_voltage(self):
        """Get voltage from secondary VESC"""
        with self._values_lock:
            if self.secondary_values is not None and hasattr(self.secondary_values, 'v_in'):
                return self.secondary_values.v_in
        return None
    
    def get_all_secondary_values(self):
        """Get all values from secondary VESC"""
        with self._values_lock:
            return self.secondary_values
    
    def print_detailed_values(self, vesc_type="primary"):
        """Print all available values for debugging"""
        values = self.get_all_primary_values() if vesc_type == "primary" else self.get_all_secondary_values()
        
        if values is None:
            print(f"No {vesc_type} VESC data available")
            return
        
        print(f"=== {vesc_type.upper()} VESC DETAILED VALUES ===")
        attrs = [attr for attr in dir(values) if not attr.startswith('_') and not callable(getattr(values, attr))]
        
        for attr in sorted(attrs):
            try:
                value = getattr(values, attr)
                print(f"  {attr}: {value}")
            except Exception as e:
                print(f"  {attr}: <error: {e}>")

async def main():
    try:
        async with BleakClient(ADDRESS) as client:
            print(f"Connected to primary VESC")
            print(f"Will query both primary and secondary (CAN {SECONDARY_CAN_ID}) VESCs")
            
            # Create motor object
            motor = BluetoothVESC(client, RX_CHARACTERISTIC, TX_CHARACTERISTIC)
            
            # Set up notification handler
            await client.start_notify(TX_CHARACTERISTIC, motor.notification_handler)
            
            # Start continuous reading for both VESCs
            await motor.start_continuous_read(read_primary=True, read_secondary=True)
            print("Continuous reading started for both VESCs")
            
            try:
                # Read values for specified duration
                for i in range(30):
                    # Get values from both VESCs
                    primary_pos = motor.get_primary_position()
                    primary_rpm = motor.get_primary_rpm()
                    primary_voltage = motor.get_primary_voltage()
                    
                    secondary_pos = motor.get_secondary_position()
                    secondary_rpm = motor.get_secondary_rpm()
                    secondary_voltage = motor.get_secondary_voltage()
                    
                    # Helper function to format values safely
                    def format_value(value, format_str=None):
                        if value is None:
                            return "N/A"
                        elif format_str:
                            return format_str.format(value)
                        return str(value)
                    
                    print(f"\n--- Iteration {i+1} ---")
                    print("Primary VESC:")
                    print(f"  Position: {format_value(primary_pos, '{:.3f}')}")
                    print(f"  RPM: {format_value(primary_rpm)}")
                    print(f"  Voltage: {format_value(primary_voltage)}V")
                    
                    print(f"Secondary VESC (CAN {SECONDARY_CAN_ID}):")
                    print(f"  Position: {format_value(secondary_pos, '{:.3f}')}")
                    print(f"  RPM: {format_value(secondary_rpm)}")
                    print(f"  Voltage: {format_value(secondary_voltage)}V")
                    
                    # Print detailed values every 5 iterations for debugging
                    # if (i + 1) % 5 == 0:
                    #     motor.print_detailed_values("primary")
                    #     motor.print_detailed_values("secondary")
                    
                    # print(f"Buffer size: {len(motor._buffer)} bytes")
                    
                    await asyncio.sleep(1)
                    
            except KeyboardInterrupt:
                print("Stopping...")
            
            finally:
                # Clean up
                await motor.stop_continuous_read()
                await client.stop_notify(TX_CHARACTERISTIC)
                print("Disconnected cleanly")
                
    except Exception as e:
        print(f"Connection failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())