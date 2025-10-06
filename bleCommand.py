import asyncio
from bleak import BleakClient
from pyvesc.protocol.interface import encode
from pyvesc.VESC.messages import SetPosition

ADDRESS = "CF:9D:22:EF:60:F9"
RX_CHARACTERISTIC = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_CHARACTERISTIC = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


class BluetoothVESC:    
    def __init__(self, client, rx_characteristic):
        self.client = client
        self.rx_characteristic = rx_characteristic
    
    async def set_pos(self, new_pos, can_id=None):
        if can_id is not None:
            buffer = encode(SetPosition(new_pos, can_id=can_id))
        else:
            buffer = encode(SetPosition(new_pos))
        
        print(f"Sending position command: {new_pos}° (CAN ID: {can_id})")
        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)


async def run_traj(motor: BluetoothVESC):
    print("Executing trajectory...")
    await motor.set_pos(300)
    await motor.set_pos(300, can_id=0x77)


async def main(address):
    async with BleakClient(address) as client:
        print(f"Connected: {client.is_connected}")

        paired = await client.pair(protection_level=2)
        print(f"Paired: {paired}")

        motor = BluetoothVESC(client, RX_CHARACTERISTIC)
        print("Motor object created")

        await run_traj(motor)


if __name__ == "__main__":
    asyncio.run(main(ADDRESS))