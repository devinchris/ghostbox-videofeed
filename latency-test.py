import asyncio
from bleak import BleakClient

# UUIDs müssen mit dem Arduino übereinstimmen
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHARACTERISTIC_UUID = "12345678-1234-5678-1234-56789abcdef1"
DEVICE_ADDRESS = "XX:XX:XX:XX:XX:XX"  # MAC-Adresse deines ESP32 eintragen

latencies = []

async def notification_handler(sender, data):
    """Wird aufgerufen, wenn eine Notification empfangen wird."""
    from datetime import datetime
    recv_time = datetime.now().timestamp() * 1000  # ms
    try:
        sent_time = int(data.decode("utf-8"))
        latency = recv_time - sent_time
        latencies.append(latency)
        print(f"Empfangen: {sent_time} ms | Latenz: {latency:.1f} ms")
    except Exception as e:
        print("Fehler beim Dekodieren:", e)

async def main():
    print("Verbinde mit ESP32...")
    async with BleakClient(DEVICE_ADDRESS) as client:
        connected = await client.is_connected()
        print("Verbunden:", connected)

        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        print("Warte auf Benachrichtigungen...\n")
        await asyncio.sleep(10)  # 10 Sekunden lang messen
        await client.stop_notify(CHARACTERISTIC_UUID)

    if latencies:
        avg = sum(latencies) / len(latencies)
        print(f"\n⏱ Durchschnittliche Latenz: {avg:.2f} ms über {len(latencies)} Messungen")
    else:
        print("Keine Daten empfangen – prüfe MAC-Adresse oder UUID.")

if __name__ == "__main__":
    asyncio.run(main())
