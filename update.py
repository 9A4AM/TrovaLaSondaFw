#!/usr/bin/python
import asyncio,struct,sys
from bleak import BleakScanner, BleakClient

SERVICE_UUID="79ee1705-f663-4674-8774-55042fc215f5"
OTA_SERVICE_UUID="0410c8a6-2c9c-4d6a-9f0e-4bc0ff7e0f7e"
BATT_UUID='66bf4d7f-2b21-468d-8dce-b241c7447cc6'
OTA_CHR="63fa4cbe-3a81-463f-aa84-049dea77a209"

with open('TrovaLaSondaFw.ino.bin','rb') as f:
    data=f.read()

async def main():
    devices = await BleakScanner.discover()

    for device in devices:
        print(device)
        if device.name!=None and device.name.startswith("TrovaLaSonda"):
            async with BleakClient(device.address) as client:
                print(client.is_connected)
                n=0
                await client.write_gatt_char(OTA_CHR,struct.pack('hl',0x4853,len(data)),True)
                while n<len(data):
                    l=min(512,len(data)-n)
                    await client.write_gatt_char(OTA_CHR,data[n:n+l],True)
                    n+=512
                    print(n,end='\r')
                await client.disconnect()
            break

asyncio.run(main())
