import asyncio
from bleak import BleakClient, BleakScanner

MAG_SERVICE_UUID = "180A"
MAG_CHAR_UUID = "2A58"
OUTPUT_FILE = "magneto_data.txt"

async def find_device():
    print("🔍 Suche nach BLE-Geräten...")
    while True:
        devices = await BleakScanner.discover()
        for d in devices:
            name = d.name or "Unbekannt"
            print(f"   → {name} ({d.address})")
            # Nimm alle, die nach Arduino aussehen oder keinen Namen haben
            if any(keyword in name for keyword in ["Arduino", "Nano", "Sense", "Mag", "Unbekannt"]):
                print(f"➡️  Teste Gerät: {name} ({d.address})")
                try:
                    async with BleakClient(d.address) as client:
                        services = await client.get_services()
                        if any(MAG_SERVICE_UUID.lower() in s.uuid.lower() for s in services):
                            print(f"✅ Gefunden: {name} ({d.address}) mit Service 180A")
                            return d.address
                except Exception:
                    pass
        print("Kein passendes Gerät gefunden – neuer Scan in 3 s…")
        await asyncio.sleep(3)

async def connect_and_listen(address):
    """Verbindet sich und empfängt Daten mit automatischem Reconnect."""
    while True:
        try:
            print(f"🔗 Verbinde mit {address}…")
            async with BleakClient(address) as client:
                print("✅ Verbunden – warte auf Daten…")
                with open(OUTPUT_FILE, "a") as f:
                    def handle_data(_, data):
                        try:
                            decoded = data.decode().strip()
                            x, y, z = map(float, decoded.split(","))
                            f.write(f"{x} {y} {z}\n")
                            f.flush()
                            print(f"{x:.2f}, {y:.2f}, {z:.2f}")
                        except Exception as e:
                            print("⚠️ Fehler beim Parsen:", e)

                    await client.start_notify(MAG_CHAR_UUID, handle_data)
                    while True:
                        await asyncio.sleep(1)

        except KeyboardInterrupt:
            print("\n🛑 Beende Programm…")
            return
        except Exception as e:
            print(f"⚠️ Verbindung verloren: {e}")
            print("⏳ Versuche in 3 s erneut zu verbinden…")
            await asyncio.sleep(3)

async def main():
    print("🚀 Starte Magnetometer-Logger…")
    address = await find_device()
    await connect_and_listen(address)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Programm beendet.")
