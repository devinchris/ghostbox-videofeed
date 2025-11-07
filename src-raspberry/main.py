import os                       # Für Socket cleanup
import json                     # Daten werden im JSON Format ausgegeben
from bleak import BleakClient   # Bibliothek für BLE connections
import asyncio                  # Kennt man
import yaml                     # Auslesen der config Datei
from datetime import datetime   # Erklärt sich von selbst
import logging                  # Logging verwenden wir beim Debuggen -> Später entfernen
import socket                   # Daten werden via Unix Domain Sockets übertagen
# import smooth_data              Wird auf dem Jet passieren

# LOGGING
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# CONFIG
with open("config.yml", "r", encoding="utf-8") as _f:
    config = yaml.safe_load(_f)

# CONSTANTS
SOCKET_PATH: str = "/tmp/jet_data.sock"
DEVICE_UID: str = config.get("device_uid", "ARDUINO-001")     # DEBUG ARDUINO bis PCB Board kommt
JET_MAC_ADDR: str = config.get("mac_address_jet")
CHAR_UID: str = config.get("characteristic_uuid")
FREQUENCY: int = config.get("frequency", 30)  # Hz

if not JET_MAC_ADDR:
    raise RuntimeError("Mac Adresse vom Jet nicht in config.yml gefunden. Bitte eintragen!")
elif not CHAR_UID:
    raise RuntimeError("Characteristics UUID nicht in config.yml gefunden. Bitte eintragen!")

# VARIABLES
running: bool = True
connected: bool = False
client: BleakClient = None  #type: ignore
latest_data: dict = {"timestamp": None, "data": None}
err_count: int = 0

# SOCKET
conn = None
sock = None
start_time = datetime.now()

"""
CONNECTIONS

BLE Verbindung managen & Daten empfangen
"""
async def connect() -> bool:
    global connected, client
    try:
        client = BleakClient(JET_MAC_ADDR, timeout=15.0)
        await client.connect()
        connected = await client.is_connected() #type: ignore
        logger.info(f"[{datetime.now().isoformat()}] Connected: {connected}")
        return connected
    except Exception as e:
        logger.error(f"Verbindungsfehler: {e}")
        connected = False
        return False

async def notification_handler(sender: int, data: bytearray) -> None:   # Daten werden per BLE Notifications empfangen
    """Callback-Funktion für BLE Benachrichtigungen"""
    global latest_data
    try:
        # Daten via BLE empfangen & parsen
        decoded = data.decode("utf-8")
        raw_data = json.loads(decoded)

        timestamp = datetime.now().isoformat()
        uptime = int((datetime.now() - start_time).total_seconds())

        # Daten formatieren
        formatted_data = {
            "timestamp": timestamp,
            "device_uid": DEVICE_UID,
            "sensor_data": {
                "temperature": raw_data.get("temperature", 0.0),
                "pressure": raw_data.get("pressure", 0.0),
                "gyro": {
                    "x": raw_data.get("gyro_x", 0.0),
                    "y": raw_data.get("gyro_y", 0.0),
                    "z": raw_data.get("gyro_z", 0.0)
                },
                "acceleration": {
                    "x": raw_data.get("accel_x", 0.0),
                    "y": raw_data.get("accel_y", 0.0),
                    "z": raw_data.get("accel_z", 0.0)
                },
                "magnetometer": {
                    "x": raw_data.get("mag_x", 0.0),
                    "y": raw_data.get("mag_y", 0.0),
                    "z": raw_data.get("mag_z", 0.0)
                },
                "compass": calculate_compass([
                    raw_data.get("mag_x", 0.0),
                    raw_data.get("mag_y", 0.0),
                    raw_data.get("mag_z", 0.0)
                ]) 
            },
            "metadata": {
                "battery": raw_data.get("battery", 0),
                "uptime": uptime
            }
        }

        latest_data = {"timestamp": timestamp, "data": formatted_data}
        await send_data(formatted_data)
    
    except json.JSONDecodeError as e:
        logger.error(f"JSON Decode Fehler: {e}")
    except Exception as a:
        logger.error(f"Fehler beim Verarbeiten der Benachrichtigung: \n {a}")

async def update_file(timestamp, data) -> None:     # ALT: Daten werden NICHT mehr in einer JSON gespeichert
    with open("data.json", "a") as f:
        json.dump({"timestamp": timestamp, "data": data}, f)
        f.write("\n")

"""
DATA

Verarbeitung der erhaltenen Daten
"""
#def smooth_data(rot: list[float]) -> list[float]:     # TODO: Rotation smoothen
#    return rot

def calculate_compass(magnet_data: list[float]) -> list[float]:  # TODO: Himmelsrichtung berechnen (wenn möglich) in Gradzahl ausgeben
    return magnet_data

"""
SOCKET

Kommunikation mit dem Rendering
"""    
async def send_data(data: dict) -> None:
    global conn
    try:
        if conn:
            conn.sendall((json.dumps(data) + "\n").encode())
    except BrokenPipeError:
        logger.warning(f"Socket Verbindung unterbrochen. {datetime.now()}")
    except Exception as e:
        logger.error(f"Fehler beim Senden der Daten: {e}")
    # Daten per Socket versenden

async def setup_socket():
    global sock, conn

    # Falls vorhanden: alten Socket path entfernen
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)    # AF_UNIX läuft nur auf Linux (Unix basierten Systemen) # type: ignore
    sock.bind(SOCKET_PATH)
    sock.listen(1)
    sock.setblocking(False)

    logger.info(f"Waiting for Renderer (UNIX SOCKET): {datetime.now().isoformat()}")

    # Warte auf Verbindung (-> Nicht blockierend)
    loop = asyncio.get_event_loop()
    conn, _ = await loop.sock_accept(sock)
    logger.info("Renderer Verbunden!")

async def main():
    global connected, running, err_count

    # Socket Setup
    await setup_socket()

    #BLE Verbindung herstellen
    while not connected:
        if await connect():
            logger.info(f"Verbindung hergestellt mit: {JET_MAC_ADDR}, um: {datetime.now().isoformat()}")
            break
        else:
            logger.warning(f"Verbindung fehlgeschlagen! ")
            err_count += 1  
            if err_count == 10:
                logger.debug("Verbindung konnte seit einiger Zeit nicht hergestellt werden. \n"
                             "Mögliche Fixes: \n"
                            "-MAC Adresse neu eintragen \n"
                            "-UUID neu eintragen \n"
                            "-Ghostbox überprüfen (Eingeschalten? In Reichweite?)")
                break
            await asyncio.sleep(5)

    # BLE Notifications aktivieren
    try:
        await client.start_notify(CHAR_UID, notification_handler) #type: ignore
        logger.info(f"BLE Notifications aktiviert für {CHAR_UID}")
    except Exception as e:
        logger.error(f"Fehler beim aktivieren der Notifications: {e}")
        return
    
    # HAUPTSCHLEIFE
    # Läuft bis running False wird
    #
    try: 
        while running:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("Programm beendet durch Keyboard Interrupt")
    finally:
        # Aufräumen
        if client and client.is_connected:
            try:
                await client.stop_notify(CHAR_UID)
            except: logger.debug("stop_notify fehlgeschlagen")
            try:
                await client.disconnect()
            except Exception as e: 
                logger.debug(f"Trennen fehlgeschlagen: {e}")
            logger.info("BLE Verbindung getrennt")

        if conn and connected:
            conn.close()
        if sock:
            sock.close()
        if os.path.exists(SOCKET_PATH):
            os.remove(SOCKET_PATH)

        logger.info(f"Programm beendet: {datetime.now().isoformat()}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info(f"Programm beendet: {0}", datetime.now().isoformat())
        RuntimeError("Programm durch Benutzer beendet.")
        import sys 
        sys.exit(0)
    except Exception as e:
        logger.exception(f"Unerwarteter Fehler: {e}")
        raise