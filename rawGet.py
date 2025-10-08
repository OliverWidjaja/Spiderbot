import numpy as np
import asyncio
import threading
from bleak import BleakClient
from pyvesc.protocol.interface import encode_request, decode
from pyvesc.VESC.messages import GetValues

# Bluetooth configuration
ADDRESS = "CF:9D:22:EF:60:F9"
RX_CHARACTERISTIC = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_CHARACTERISTIC = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

class BluetoothVESC:    
    def __init__(self, client, rx_characteristic, tx_characteristic):
        self.client = client
        self.rx_characteristic = rx_characteristic
        self.tx_characteristic = tx_characteristic
        self.current_values = None
        self._stop_requested = False
        self._values_lock = threading.Lock()
        self._buffer = bytearray()
        
        # Store message info for getting values
        msg = GetValues()
        self._get_values_msg = encode_request(msg)
        self._get_values_msg_expected_length = msg._full_msg_size
    
    async def start_continuous_read(self):
        """Start continuously reading VESC values"""
        self._stop_requested = False
        asyncio.create_task(self._continuous_read_task())
    
    async def stop_continuous_read(self):
        """Stop the continuous reading task"""
        self._stop_requested = True
    
    async def _continuous_read_task(self):
        """Background task to continuously read VESC values"""
        while not self._stop_requested:
            try:
                # Send get values request
                await self.client.write_gatt_char(self.rx_characteristic, self._get_values_msg)
                
                # Wait a bit for response
                await asyncio.sleep(0.05)  # Increased to 50ms for better response handling
                
            except Exception as e:
                print(f"Error in continuous read: {e}")
                await asyncio.sleep(0.1)  # Wait longer on error
    
    async def notification_handler(self, sender, data):
        """Handle incoming notifications (responses from VESC)"""
        try:
            # Add new data to buffer
            self._buffer.extend(data)
            
            # Try to decode complete messages from buffer
            while len(self._buffer) > 0:
                # Try to decode from current buffer
                response, consumed = decode(bytes(self._buffer))
                
                if consumed > 0:
                    # Successfully decoded a message
                    print(f"Successfully decoded message, consumed {consumed} bytes")
                    
                    if response is not None:
                        print(f"Decoded response type: {type(response)}")
                        
                        # Check if this is a GetValues response
                        if hasattr(response, 'rpm') or hasattr(response, 'v_in') or hasattr(response, 'pid_pos_now'):
                            # Update current values with thread safety
                            with self._values_lock:
                                self.current_values = response
                                print(f"Successfully updated VESC values")
                                
                                # Print some key values for debugging
                                if hasattr(response, 'rpm'):
                                    print(f"  RPM: {response.rpm}")
                                if hasattr(response, 'v_in'):
                                    print(f"  Input Voltage: {response.v_in}")
                                if hasattr(response, 'pid_pos_now'):
                                    print(f"  Position: {response.pid_pos_now}")
                                if hasattr(response, 'temp_mos'):
                                    print(f"  MOSFET Temp: {response.temp_mos}")
                        else:
                            print(f"Unexpected response type: {type(response)}")
                    
                    # Remove consumed bytes from buffer
                    self._buffer = self._buffer[consumed:]
                    
                else:
                    # Not enough data for a complete message, wait for more
                    print(f"Buffer has {len(self._buffer)} bytes, waiting for more data...")
                    break
                    
        except Exception as e:
            print(f"Error in notification handler: {e}")
            import traceback
            traceback.print_exc()
            
            # Clear buffer on error to prevent corruption
            self._buffer.clear()
    
    def get_position(self):
        """
        Get the current position value from the latest measurements
        
        :return: Current motor position, or None if no data available
        """
        with self._values_lock:
            if self.current_values is not None:
                # Try different possible attribute names for position
                for attr in ['pid_pos_now', 'position', 'pos']:
                    if hasattr(self.current_values, attr):
                        return getattr(self.current_values, attr)
        return None
    
    def get_all_values(self):
        """
        Get all current measurement values
        
        :return: Current GetValues message object, or None if no data available
        """
        with self._values_lock:
            return self.current_values

async def main():
    try:
        async with BleakClient(ADDRESS) as client:
            print(f"Connected: {client.is_connected}")

            # Try pairing (might not be necessary)
            try:
                paired = await client.pair(protection_level=2)
                print(f"Paired: {paired}")
            except Exception as e:
                print(f"Pairing failed or not required: {e}")

            # Create motor object with both RX and TX characteristics
            motor = BluetoothVESC(client, RX_CHARACTERISTIC, TX_CHARACTERISTIC)
            print("Motor object created")
            
            # Set up notification handler for incoming data
            await client.start_notify(TX_CHARACTERISTIC, motor.notification_handler)
            print("Notification handler set up")
            
            # Start continuous reading
            await motor.start_continuous_read()
            print("Continuous reading started")
            
            try:
                # Example usage: print position every second for 10 seconds
                for i in range(10):
                    position = motor.get_position()
                    all_values = motor.get_all_values()
                    
                    if all_values is not None:
                        print(f"\n--- Iteration {i+1} ---")
                        print(f"Position: {position}")
                        
                        # Print all available attributes from the response
                        print("Available values:")
                        attrs = [attr for attr in dir(all_values) if not attr.startswith('_') and not callable(getattr(all_values, attr))]
                        for attr in sorted(attrs):
                            try:
                                value = getattr(all_values, attr)
                                print(f"  {attr}: {value}")
                            except Exception as e:
                                print(f"  {attr}: <error reading>")
                    else:
                        print(f"\nIteration {i+1}: No valid VESC data received yet")
                        print(f"Current buffer size: {len(motor._buffer)} bytes")
                    
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