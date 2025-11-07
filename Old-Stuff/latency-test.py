#!/usr/bin/env python3
"""
Raspberry Pi BLE Debug Script

Verbindet sich mit Arduino, empfängt Daten und misst Latenzen
"""

import asyncio
import json
import time
from datetime import datetime
from bleak import BleakClient, BleakScanner
from statistics import mean, stdev

# KONFIGURATION
DEVICE_NAME = "JET-DEBUG"  # Name des Arduino
CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
SCAN_TIMEOUT = 10.0  # Sekunden
CONNECTION_TIMEOUT = 15.0

# STATISTIKEN
latencies = []
messages_received = 0
messages_lost = 0
last_msg_id = 0
start_time = None

# ANSI Farben für schönere Ausgabe
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'

def log(msg, color=Colors.RESET):
    """Farbige Log-Ausgabe"""
    print(f"{color}{msg}{Colors.RESET}")

async def scan_for_device():
    """Sucht nach dem Arduino BLE Device"""
    log("\n🔍 Scanne nach BLE Devices...", Colors.BLUE)
    
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
    
    log(f"\n📡 {len(devices)} Device(s) gefunden:", Colors.BLUE)
    for i, d in enumerate(devices, 1):
        marker = "✅" if d.name == DEVICE_NAME else "  "
        print(f"{marker} {i}. {d.name or 'Unbekannt'} - {d.address}")
    
    # Nach Namen suchen
    target = None
    for d in devices:
        if d.name == DEVICE_NAME:
            target = d
            break
    
    if target:
        log(f"\n✅ Ziel-Device gefunden: {target.name} ({target.address})", Colors.GREEN)
        return target.address
    else:
        log(f"\n❌ Device '{DEVICE_NAME}' nicht gefunden!", Colors.RED)
        return None

def notification_handler(sender, data):
    """Callback für empfangene BLE Notifications"""
    global messages_received, messages_lost, last_msg_id, start_time
    
    receive_time = time.time() * 1000  # in ms
    
    try:
        # JSON parsen
        json_str = data.decode('utf-8')
        msg = json.loads(json_str)
        
        messages_received += 1
        msg_id = msg.get('msg_id', 0)
        arduino_time = msg.get('arduino_time', 0)
        
        # Latenz berechnen (grobe Schätzung)
        if start_time:
            elapsed = (time.time() - start_time) * 1000
            estimated_latency = elapsed - arduino_time
            latencies.append(estimated_latency)
        
        # Verlorene Nachrichten erkennen
        if last_msg_id > 0 and msg_id > last_msg_id + 1:
            lost = msg_id - last_msg_id - 1
            messages_lost += lost
            log(f"⚠️  {lost} Nachricht(en) verloren!", Colors.YELLOW)
        
        last_msg_id = msg_id
        
        # Ausgabe
        temp = msg.get('temperature', 0)
        battery = msg.get('battery', 0)
        
        if len(latencies) > 0:
            avg_lat = mean(latencies[-10:])  # Durchschnitt der letzten 10
            log(f"📥 Msg #{msg_id} | Temp: {temp}°C | Bat: {battery}% | "
                f"Est. Latenz: {estimated_latency:.1f}ms (Ø {avg_lat:.1f}ms)", 
                Colors.GREEN)
        else:
            log(f"📥 Msg #{msg_id} | Temp: {temp}°C | Bat: {battery}%", Colors.GREEN)
        
    except json.JSONDecodeError as e:
        log(f"❌ JSON Fehler: {e}", Colors.RED)
    except Exception as e:
        log(f"❌ Error: {e}", Colors.RED)

async def main():
    global start_time
    
    log("=" * 50, Colors.BLUE)
    log("  Raspberry Pi BLE Debug Tool", Colors.BLUE)
    log("=" * 50, Colors.BLUE)
    
    # Device suchen
    mac_address = await scan_for_device()
    if not mac_address:
        log("\n💡 Tipps:", Colors.YELLOW)
        log("  - Arduino BLE Code hochgeladen?")
        log("  - Arduino eingeschaltet und in Reichweite?")
        log("  - Bluetooth am Raspberry Pi aktiv? (bluetoothctl power on)")
        return
    
    # Verbinden
    log(f"\n🔗 Verbinde mit {mac_address}...", Colors.BLUE)
    
    try:
        async with BleakClient(mac_address, timeout=CONNECTION_TIMEOUT) as client:
            log(f"✅ Verbunden!", Colors.GREEN)
            
            # Services auflisten
            log(f"\n📋 Verfügbare Services:", Colors.BLUE)
            for service in client.services:
                print(f"  Service: {service.uuid}")
                for char in service.characteristics:
                    props = ", ".join(char.properties)
                    marker = "✅" if char.uuid == CHAR_UUID else "  "
                    print(f"  {marker} Char: {char.uuid} ({props})")
            
            # Notifications aktivieren
            log(f"\n🔔 Aktiviere Notifications für {CHAR_UUID}...", Colors.BLUE)
            await client.start_notify(CHAR_UUID, notification_handler)
            log("✅ Notifications aktiv! Empfange Daten...\n", Colors.GREEN)
            
            start_time = time.time()
            
            # Läuft bis Ctrl+C
            try:
                while True:
                    await asyncio.sleep(1)
            except KeyboardInterrupt:
                log("\n\n⏹️  Beende...", Colors.YELLOW)
            
            # Notifications stoppen
            await client.stop_notify(CHAR_UUID)
            
    except asyncio.TimeoutError:
        log(f"\n❌ Verbindungs-Timeout!", Colors.RED)
    except Exception as e:
        log(f"\n❌ Verbindungsfehler: {e}", Colors.RED)
    
    # Statistiken ausgeben
    if messages_received > 0:
        log("\n" + "=" * 50, Colors.BLUE)
        log("  STATISTIKEN", Colors.BLUE)
        log("=" * 50, Colors.BLUE)
        log(f"📨 Empfangene Nachrichten: {messages_received}")
        log(f"❌ Verlorene Nachrichten: {messages_lost}")
        
        if len(latencies) > 0:
            log(f"\n⏱️  Latenz-Analyse:")
            log(f"  Min:    {min(latencies):.2f} ms")
            log(f"  Max:    {max(latencies):.2f} ms")
            log(f"  Mittel: {mean(latencies):.2f} ms")
            if len(latencies) > 1:
                log(f"  StdDev: {stdev(latencies):.2f} ms")
        
        packet_loss = (messages_lost / (messages_received + messages_lost)) * 100
        log(f"\n📊 Paketverlust: {packet_loss:.2f}%")
        
        if packet_loss < 1:
            log("\n✅ Verbindungsqualität: Ausgezeichnet", Colors.GREEN)
        elif packet_loss < 5:
            log("\n⚠️  Verbindungsqualität: Gut", Colors.YELLOW)
        else:
            log("\n❌ Verbindungsqualität: Schlecht", Colors.RED)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log("\n\nProgramm beendet.", Colors.YELLOW)
