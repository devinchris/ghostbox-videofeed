"""
Panda3D BLE Quaternion Visualizer
Empfängt Quaternionen vom Arduino Nano 33 BLE Sense Rev2 via BLE
und visualisiert die Rotation in Echtzeit.
"""

import asyncio
import struct
import numpy as np
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    loadPrcFileData, LVector3f, Quat, TextNode,
    AmbientLight, DirectionalLight, LColor, WindowProperties
)
from direct.task import Task
from bleak import BleakClient, BleakScanner
import threading
import queue
from collections import deque
import math

# Panda3D Konfiguration
loadPrcFileData("", "win-size 1280 720")
loadPrcFileData("", "window-title Quaternion Visualizer")
loadPrcFileData("", "sync-video false")
loadPrcFileData("", "show-frame-rate-meter true")

# BLE Konfiguration
DEVICE_NAME = "Arduino Nano 33"  # Anpassen je nach Arduino Name
SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"  # Anpassen
CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"  # Anpassen


class SensorData:
    """Speichert die Sensordaten vom Arduino"""
    def __init__(self):
        self.timestamp_ms = 0
        self.accel = LVector3f(0, 0, 0)
        self.quaternion = Quat(1, 0, 0, 0)  # w, x, y, z
        self.mag = LVector3f(0, 0, 0)
        self.temp_c = 0.0
        self.euler = LVector3f(0, 0, 0)  # roll, pitch, yaw in Grad


class QuaternionSmoother:
    """Smooth interpolation zwischen Quaternionen mit SLERP"""
    def __init__(self, window_size=5, smooth_factor=0.3):
        self.history = deque(maxlen=window_size)
        self.current = Quat(1, 0, 0, 0)
        self.target = Quat(1, 0, 0, 0)
        self.smooth_factor = smooth_factor
        
    def add_quaternion(self, q):
        """Fügt neues Quaternion hinzu"""
        self.history.append(q)
        # Durchschnitt der letzten Quaternionen (einfache Mittelung)
        if len(self.history) > 0:
            self.target = self._average_quaternions()
    
    def _average_quaternions(self):
        """Berechnet durchschnittliches Quaternion"""
        if len(self.history) == 1:
            return self.history[0]
        
        # Verwende das erste Quaternion als Referenz
        q_avg = Quat(self.history[0])
        
        # Einfache gewichtete Mittelung
        for i, q in enumerate(self.history[1:], 1):
            weight = i / len(self.history)
            q_avg = self._slerp(q_avg, q, weight / (i + 1))
        
        return q_avg
    
    def _slerp(self, q1, q2, t):
        """Spherical Linear Interpolation"""
        # Panda3D's eingebaute SLERP verwenden
        # Quaternion Komponenten: [0]=w, [1]=x, [2]=y, [3]=z
        return Quat(
            q1[0] * (1 - t) + q2[0] * t,  # w
            q1[1] * (1 - t) + q2[1] * t,  # x
            q1[2] * (1 - t) + q2[2] * t,  # y
            q1[3] * (1 - t) + q2[3] * t   # z
        )
    
    def get_smoothed(self, dt):
        """Gibt geglättetes Quaternion zurück"""
        # SLERP zum Ziel
        alpha = min(1.0, self.smooth_factor * dt * 60)  # 60 fps als Basis
        self.current = self._slerp(self.current, self.target, alpha)
        return self.current


class BLEManager:
    """Verwaltet BLE Verbindung und Datenempfang"""
    def __init__(self, data_queue):
        self.data_queue = data_queue
        self.client = None
        self.is_connected = False
        self.should_run = True
        self.loop = None
        
    async def find_device(self):
        """Sucht nach Arduino"""
        print("Suche nach BLE Geräten...")
        devices = await BleakScanner.discover(timeout=10.0)
        
        for device in devices:
            print(f"Gefunden: {device.name} - {device.address}")
            if device.name and DEVICE_NAME in device.name:
                print(f"Arduino gefunden: {device.address}")
                return device.address
        
        return None
    
    def parse_sensor_packet(self, data):
        """Parst 32-Byte Sensorpaket"""
        if len(data) != 32:
            return None
        
        try:
            # Struct: uint32 + 3*float + 4*float + 3*float + float = 32 bytes
            unpacked = struct.unpack('<I10f', data)
            
            sensor_data = SensorData()
            sensor_data.timestamp_ms = unpacked[0]
            sensor_data.accel = LVector3f(unpacked[1], unpacked[2], unpacked[3])
            
            # Quaternion: q0, q1, q2, q3 (w, x, y, z)
            sensor_data.quaternion = Quat(
                unpacked[4],  # w
                unpacked[5],  # x
                unpacked[6],  # y
                unpacked[7]   # z
            )
            
            sensor_data.mag = LVector3f(unpacked[8], unpacked[9], unpacked[10])
            sensor_data.temp_c = unpacked[11]
            
            # Euler Winkel berechnen
            sensor_data.euler = self.quat_to_euler(sensor_data.quaternion)
            
            return sensor_data
        except Exception as e:
            print(f"Fehler beim Parsen: {e}")
            return None
    
    def quat_to_euler(self, q):
        """Konvertiert Quaternion zu Euler Winkeln (Roll, Pitch, Yaw) in Grad"""
        # w, x, y, z
        w, x, y, z = q[0], q[1], q[2], q[3]
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # In Grad umwandeln
        return LVector3f(
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw)
        )
    
    def notification_handler(self, sender, data):
        """Callback für BLE Benachrichtigungen"""
        sensor_data = self.parse_sensor_packet(data)
        if sensor_data:
            self.data_queue.put(sensor_data)
    
    async def connect_and_run(self):
        """Verbindet mit Arduino und empfängt Daten"""
        while self.should_run:
            try:
                device_address = await self.find_device()
                
                if not device_address:
                    print("Arduino nicht gefunden. Erneuter Versuch in 5 Sekunden...")
                    await asyncio.sleep(5)
                    continue
                
                print(f"Verbinde mit {device_address}...")
                async with BleakClient(device_address, timeout=20.0) as client:
                    self.client = client
                    self.is_connected = True
                    print("Verbunden! Empfange Daten...")
                    
                    # Benachrichtigungen aktivieren
                    await client.start_notify(CHARACTERISTIC_UUID, self.notification_handler)
                    
                    # Verbindung aufrechterhalten
                    while self.should_run and client.is_connected:
                        await asyncio.sleep(1)
                    
                    await client.stop_notify(CHARACTERISTIC_UUID)
                    
            except Exception as e:
                print(f"BLE Fehler: {e}")
                self.is_connected = False
                print("Erneuter Verbindungsversuch in 5 Sekunden...")
                await asyncio.sleep(5)
        
        self.is_connected = False
    
    def start(self):
        """Startet BLE Manager in eigenem Thread"""
        def run_async_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self.connect_and_run())
        
        thread = threading.Thread(target=run_async_loop, daemon=True)
        thread.start()
    
    def stop(self):
        """Stoppt BLE Manager"""
        self.should_run = False
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)


class QuaternionVisualizerApp(ShowBase):
    """Hauptanwendung für Panda3D Visualisierung"""
    
    def __init__(self):
        super().__init__()
        
        # Datenstrukturen
        self.data_queue = queue.Queue()
        self.current_data = SensorData()
        self.smoother = QuaternionSmoother(window_size=5, smooth_factor=0.3)
        
        # BLE Manager starten
        self.ble_manager = BLEManager(self.data_queue)
        self.ble_manager.start()
        
        # 3D Szene aufbauen
        self.setup_scene()
        self.setup_lights()
        self.setup_cube()
        self.setup_ui()
        
        # Update Task
        self.taskMgr.add(self.update_task, "update_task")
        
        # Fenster-Events
        self.accept('window-event', self.on_window_event)
        
        print("Visualizer gestartet. Warte auf BLE Verbindung...")
    
    def setup_scene(self):
        """Richtet die 3D Szene ein"""
        self.cam.setPos(0, -8, 3)
        self.cam.lookAt(0, 0, 0)
        
        # Hintergrundfarbe
        self.setBackgroundColor(0.1, 0.1, 0.15)
    
    def setup_lights(self):
        """Fügt Beleuchtung hinzu"""
        # Ambient Light
        ambient = AmbientLight("ambient")
        ambient.setColor(LColor(0.3, 0.3, 0.3, 1))
        ambient_np = self.render.attachNewNode(ambient)
        self.render.setLight(ambient_np)
        
        # Directional Light
        directional = DirectionalLight("directional")
        directional.setColor(LColor(0.8, 0.8, 0.8, 1))
        directional_np = self.render.attachNewNode(directional)
        directional_np.setHpr(-45, -45, 0)
        self.render.setLight(directional_np)
    
    def setup_cube(self):
        """Erstellt den farbigen Quader"""
        # Hauptquader
        self.cube = self.loader.loadModel("models/box")
        self.cube.reparentTo(self.render)
        self.cube.setScale(2, 3, 1)
        
        # Verschiedene Seiten einfärben durch Duplikate mit verschiedenen Farben
        # Front (rot)
        front = self.loader.loadModel("models/box")
        front.reparentTo(self.cube)
        front.setScale(0.51, 1.01, 1.01)
        front.setPos(0, 0, 0)
        front.setColor(1, 0.2, 0.2, 1)
        
        # Back (grün)
        back = self.loader.loadModel("models/box")
        back.reparentTo(self.cube)
        back.setScale(0.51, 1.01, 1.01)
        back.setPos(0, 0, 0)
        back.setH(180)
        back.setColor(0.2, 1, 0.2, 1)
        
        # Right (blau)
        right = self.loader.loadModel("models/box")
        right.reparentTo(self.cube)
        right.setScale(1.01, 0.51, 1.01)
        right.setPos(0, 0, 0)
        right.setH(90)
        right.setColor(0.2, 0.2, 1, 1)
        
        # Left (gelb)
        left = self.loader.loadModel("models/box")
        left.reparentTo(self.cube)
        left.setScale(1.01, 0.51, 1.01)
        left.setPos(0, 0, 0)
        left.setH(-90)
        left.setColor(1, 1, 0.2, 1)
        
        # Top (magenta)
        top = self.loader.loadModel("models/box")
        top.reparentTo(self.cube)
        top.setScale(1.01, 1.01, 0.51)
        top.setPos(0, 0, 0)
        top.setP(90)
        top.setColor(1, 0.2, 1, 1)
        
        # Bottom (cyan)
        bottom = self.loader.loadModel("models/box")
        bottom.reparentTo(self.cube)
        bottom.setScale(1.01, 1.01, 0.51)
        bottom.setPos(0, 0, 0)
        bottom.setP(-90)
        bottom.setColor(0.2, 1, 1, 1)
    
    def setup_ui(self):
        """Erstellt UI für Datenanzeige"""
        # Text Node für Sensor Daten
        self.text_node = TextNode('sensor_data')
        self.text_node.setText("")
        self.text_node.setTextColor(1, 1, 1, 1)
        self.text_node.setAlign(TextNode.ALeft)
        
        text_np = self.aspect2d.attachNewNode(self.text_node)
        text_np.setScale(0.04)
        text_np.setPos(-1.9, 0, 0.9)
        
        # Verbindungsstatus
        self.status_node = TextNode('status')
        self.status_node.setText("Status: Warte auf Verbindung...")
        self.status_node.setTextColor(1, 0.5, 0, 1)
        self.status_node.setAlign(TextNode.ARight)
        
        status_np = self.aspect2d.attachNewNode(self.status_node)
        status_np.setScale(0.05)
        status_np.setPos(1.9, 0, 0.9)
    
    def update_ui(self):
        """Aktualisiert UI Text"""
        data = self.current_data
        
        text = (
            f"Timestamp: {data.timestamp_ms} ms\n"
            f"\n"
            f"Quaternion:\n"
            f"  W: {data.quaternion[0]:7.3f}\n"
            f"  X: {data.quaternion[1]:7.3f}\n"
            f"  Y: {data.quaternion[2]:7.3f}\n"
            f"  Z: {data.quaternion[3]:7.3f}\n"
            f"\n"
            f"Euler (Grad):\n"
            f"  Roll:  {data.euler.x:7.2f} deg\n"
            f"  Pitch: {data.euler.y:7.2f} deg\n"
            f"  Yaw:   {data.euler.z:7.2f} deg\n"
            f"\n"
            f"Accel (m/s^2):\n"
            f"  X: {data.accel.x:7.3f}\n"
            f"  Y: {data.accel.y:7.3f}\n"
            f"  Z: {data.accel.z:7.3f}\n"
            f"\n"
            f"Mag (Tesla):\n"
            f"  X: {data.mag.x:9.6f}\n"
            f"  Y: {data.mag.y:9.6f}\n"
            f"  Z: {data.mag.z:9.6f}\n"
            f"\n"
            f"Temp: {data.temp_c:.1f} C"
        )
        
        self.text_node.setText(text)
        
        # Status aktualisieren
        if self.ble_manager.is_connected:
            self.status_node.setText("Status: Verbunden OK")
            self.status_node.setTextColor(0, 1, 0, 1)
        else:
            self.status_node.setText("Status: Getrennt X")
            self.status_node.setTextColor(1, 0, 0, 1)
    
    def update_task(self, task):
        """Update Task - wird jeden Frame aufgerufen"""
        dt = self.taskMgr.globalClock.getDt()
        
        # Neue Daten aus Queue holen
        try:
            while not self.data_queue.empty():
                self.current_data = self.data_queue.get_nowait()
                # Quaternion zum Smoother hinzufügen
                self.smoother.add_quaternion(self.current_data.quaternion)
        except queue.Empty:
            pass
        
        # Geglättetes Quaternion anwenden
        smoothed_quat = self.smoother.get_smoothed(dt)
        self.cube.setQuat(smoothed_quat)
        
        # UI aktualisieren
        self.update_ui()
        
        return Task.cont
    
    def on_window_event(self, window=None):
        """Handler für Fenster-Events (z.B. Größenänderung)"""
        if window is not None:
            wp = window.getProperties()
            if wp.getOpen():
                # Fenster ist offen, passe Kamera an
                aspect_ratio = wp.getXSize() / max(wp.getYSize(), 1)
                self.cam.node().getLens().setAspectRatio(aspect_ratio)
    
    def cleanup(self):
        """Cleanup beim Beenden"""
        print("Beende Anwendung...")
        self.ble_manager.stop()
        super().destroy()


if __name__ == "__main__":
    app = QuaternionVisualizerApp()
    try:
        app.run()
    except KeyboardInterrupt:
        print("\nAnwendung wird beendet...")
    finally:
        app.cleanup()
