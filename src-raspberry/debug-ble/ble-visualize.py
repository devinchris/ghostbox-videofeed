"""
IMU 3D Visualizer for Raspberry Pi 5 using Panda3D
Real-time visualization of Arduino Nano 33 BLE IMU data.
High-performance 3D rendering with real-time sensor graphs.
"""

import asyncio
import logging
import struct
import numpy as np
from collections import deque
import threading

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import (
    LineSegs, NodePath, TextNode, CardMaker,
    GeomNode, GeomVertexFormat, GeomVertexData, Geom,
    GeomTriangles, GeomVertexWriter, LVector3, LQuaternion,
    AmbientLight, DirectionalLight, AntialiasAttrib
)
from bleak import BleakScanner, BleakClient

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# BLE Configuration
SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
DEVICE_NAME = "Nano"

# Visualization settings
GRAPH_HISTORY = 150
UPDATE_FPS = 10  # Target framerate for visualization


class IMUData:
    """Thread-safe storage for IMU data"""
    def __init__(self):
        self.lock = threading.Lock()
        self.timestamp = 0
        self.accel = np.array([0.0, 0.0, 9.8])
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # q0, q1, q2, q3
        self.mag = np.array([0.0, 0.0, 0.0])
        self.temp = 20.0
        
        # History for graphs
        self.time_history = deque(maxlen=GRAPH_HISTORY)
        self.accel_history = deque(maxlen=GRAPH_HISTORY)
        self.mag_history = deque(maxlen=GRAPH_HISTORY)
        self.temp_history = deque(maxlen=GRAPH_HISTORY)
        
        self.connected = False
        self.status = "Initializing..."
        self.packet_count = 0
    
    def update(self, data):
        """Update with new sensor data"""
        with self.lock:
            (timestamp_ms, accelX, accelY, accelZ, q0, q1, q2, q3,
             magX, magY, magZ, temp) = data
            
            self.timestamp = timestamp_ms
            self.accel = np.array([accelX, accelY, accelZ])
            self.quaternion = np.array([q0, q1, q2, q3])
            self.mag = np.array([magX, magY, magZ])
            self.temp = temp
            self.packet_count += 1
            
            # Add to history
            self.time_history.append(timestamp_ms / 1000.0)
            self.accel_history.append([accelX, accelY, accelZ])
            self.mag_history.append([magX, magY, magZ])
            self.temp_history.append(temp)
    
    def get_snapshot(self):
        """Get a snapshot of current data"""
        with self.lock:
            return {
                'timestamp': self.timestamp,
                'accel': self.accel.copy(),
                'quaternion': self.quaternion.copy(),
                'mag': self.mag.copy(),
                'temp': self.temp,
                'time_history': list(self.time_history),
                'accel_history': list(self.accel_history),
                'mag_history': list(self.mag_history),
                'temp_history': list(self.temp_history),
                'connected': self.connected,
                'status': self.status,
                'packet_count': self.packet_count
            }


class BLEReceiver:
    """BLE receiver running in separate thread"""
    def __init__(self, imu_data):
        self.imu_data = imu_data
        self.device = None
        self.client = None
        self._stopping = False
    
    async def scan_for_device(self, timeout=10.0):
        """Scan for Arduino device"""
        logger.info(f"Scanning for '{DEVICE_NAME}'...")
        self.imu_data.status = "Scanning..."
        
        devices = await BleakScanner.discover(timeout=timeout)
        for device in devices:
            if device.name and DEVICE_NAME in device.name:
                logger.info(f"Found: {device.name} ({device.address})")
                self.device = device
                return True
        
        logger.warning(f"Device '{DEVICE_NAME}' not found")
        return False
    
    def notification_handler(self, sender, data: bytearray):
        """Handle incoming BLE notifications"""
        try:
            unpacked = struct.unpack('<Ifffffffffff', data)
            self.imu_data.update(unpacked)
        except struct.error as e:
            logger.error(f"Unpack error: {e}")

    
    async def connect_and_receive(self):
        """Main BLE connection loop"""
        while not self._stopping:
            if not self.device:
                if not await self.scan_for_device():
                    await asyncio.sleep(5)
                    continue
            
            try:
                self.imu_data.status = "Connecting..."
                async with BleakClient(self.device.address, timeout=30.0) as client:
                    self.client = client
                    logger.info(f"Connected to {DEVICE_NAME}")
                    self.imu_data.connected = True
                    self.imu_data.status = "Connected"
                    
                    await client.start_notify(CHARACTERISTIC_UUID, self.notification_handler)
                    logger.info("Receiving data...")
                    
                    while client.is_connected and not self._stopping:
                        await asyncio.sleep(1)
                    
                    logger.warning("Disconnected")
                    self.imu_data.connected = False
                    self.imu_data.status = "Disconnected"
                    
            except Exception as e:
                logger.error(f"Connection error: {e}")
                self.imu_data.connected = False
                self.imu_data.status = f"Error: {str(e)[:30]}"
                self.device = None
            
            await asyncio.sleep(2)
    
    def stop(self):
        """Stop the receiver"""
        self._stopping = True

    async def disconnect(self):
        """Cleanly disconnect BLE client"""
        self._stopping = True
        try:
            if self.client and self.client.is_connected:
                await self.client.stop_notify(CHARACTERISTIC_UUID)
                await self.client.disconnect()
                logger.info("BLE disconnected cleanly")
        except Exception as e:
            logger.error(f"BLE disconnect error: {e}")


class IMUVisualizer(ShowBase):
    """Panda3D-based IMU visualizer"""
    
    def __init__(self, imu_data):
        ShowBase.__init__(self)
        self.imu_data = imu_data
        
        # Window setup
        self.setBackgroundColor(0.1, 0.1, 0.15, 1)
        self.disableMouse()
        
        # Camera setup
        self.camera.setPos(0, -8, 3)
        self.camera.lookAt(0, 0, 0)
        
        # Enable antialiasing
        self.render.setAntialias(AntialiasAttrib.MAuto)
        
        # Setup lighting
        self.setup_lighting()
        
        # Create 3D objects
        self.setup_scene()
        
        # Create UI elements
        self.setup_ui()
        
        # Create graph canvases
        self.setup_graphs()
        
        # Add update task
        self.taskMgr.add(self.update_scene, "UpdateScene")
        
        logger.info("Panda3D visualizer initialized")
    
    def setup_lighting(self):
        """Setup scene lighting"""
        # Ambient light
        alight = AmbientLight('ambient')
        alight.setColor((0.4, 0.4, 0.4, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)
        
        # Directional light
        dlight = DirectionalLight('directional')
        dlight.setColor((0.8, 0.8, 0.8, 1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(45, -45, 0)
        self.render.setLight(dlnp)
    
    def setup_scene(self):
        """Setup 3D scene objects"""
        # Create coordinate axes
        self.create_axes()
        
        # Create IMU box
        self.imu_box = self.create_imu_box()
        self.imu_box.reparentTo(self.render)
        
        # Create accelerometer arrow
        self.accel_arrow = NodePath("accel_arrow")
        self.accel_arrow.reparentTo(self.render)
        
        # Create magnetometer arrow
        self.mag_arrow = NodePath("mag_arrow")
        self.mag_arrow.reparentTo(self.render)
        
        # Create grid
        self.create_grid()
    
    def create_axes(self):
        """Create XYZ coordinate axes"""
        # X axis (red)
        x_lines = LineSegs()
        x_lines.setThickness(3)
        x_lines.setColor(1, 0, 0, 1)
        x_lines.moveTo(0, 0, 0)
        x_lines.drawTo(2, 0, 0)
        x_node = x_lines.create()
        self.render.attachNewNode(x_node)
        
        # Y axis (green)
        y_lines = LineSegs()
        y_lines.setThickness(3)
        y_lines.setColor(0, 1, 0, 1)
        y_lines.moveTo(0, 0, 0)
        y_lines.drawTo(0, 2, 0)
        y_node = y_lines.create()
        self.render.attachNewNode(y_node)
        
        # Z axis (blue)
        z_lines = LineSegs()
        z_lines.setThickness(3)
        z_lines.setColor(0, 0, 1, 1)
        z_lines.moveTo(0, 0, 0)
        z_lines.drawTo(0, 0, 2)
        z_node = z_lines.create()
        self.render.attachNewNode(z_node)
    
    def create_grid(self):
        """Create ground grid"""
        lines = LineSegs()
        lines.setThickness(1)
        lines.setColor(0.3, 0.3, 0.3, 1)
        
        size = 5
        step = 0.5
        for i in np.arange(-size, size + step, step):
            lines.moveTo(i, -size, 0)
            lines.drawTo(i, size, 0)
            lines.moveTo(-size, i, 0)
            lines.drawTo(size, i, 0)
        
        grid_node = lines.create()
        self.render.attachNewNode(grid_node)
    
    def create_imu_box(self):
        """Create 3D box representing Arduino"""
        format = GeomVertexFormat.getV3n3c4()
        vdata = GeomVertexData('box', format, Geom.UHStatic)
        vdata.setNumRows(24)
        
        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')
        color = GeomVertexWriter(vdata, 'color')
        
        # Box dimensions (Arduino Nano form factor)
        w, h, d = 0.6, 0.15, 1.8
        
        # Define vertices for each face
        faces = [
            # Bottom (blue)
            [(-w/2, -d/2, -h/2), (w/2, -d/2, -h/2), (w/2, d/2, -h/2), (-w/2, d/2, -h/2), (0, 0, -1), (0.3, 0.3, 0.8, 1)],
            # Top (red)
            [(-w/2, -d/2, h/2), (w/2, -d/2, h/2), (w/2, d/2, h/2), (-w/2, d/2, h/2), (0, 0, 1), (0.8, 0.3, 0.3, 1)],
            # Front
            [(-w/2, -d/2, -h/2), (w/2, -d/2, -h/2), (w/2, -d/2, h/2), (-w/2, -d/2, h/2), (0, -1, 0), (0.5, 0.5, 0.5, 1)],
            # Back
            [(-w/2, d/2, -h/2), (w/2, d/2, -h/2), (w/2, d/2, h/2), (-w/2, d/2, h/2), (0, 1, 0), (0.5, 0.5, 0.5, 1)],
            # Left
            [(-w/2, -d/2, -h/2), (-w/2, d/2, -h/2), (-w/2, d/2, h/2), (-w/2, -d/2, h/2), (-1, 0, 0), (0.5, 0.5, 0.5, 1)],
            # Right
            [(w/2, -d/2, -h/2), (w/2, d/2, -h/2), (w/2, d/2, h/2), (w/2, -d/2, h/2), (1, 0, 0), (0.5, 0.5, 0.5, 1)],
        ]
        
        triangles = GeomTriangles(Geom.UHStatic)
        idx = 0
        
        for face in faces:
            v1, v2, v3, v4, norm, col = face
            
            # Add vertices
            for v in [v1, v2, v3, v4]:
                vertex.addData3f(*v)
                normal.addData3f(*norm)
                color.addData4f(*col)
            
            # Add triangles
            triangles.addVertices(idx, idx+1, idx+2)
            triangles.addVertices(idx, idx+2, idx+3)
            idx += 4
        
        geom = Geom(vdata)
        geom.addPrimitive(triangles)
        
        node = GeomNode('box')
        node.addGeom(geom)
        
        return NodePath(node)
    
    def create_arrow(self, color):
        """Create an arrow for vectors"""
        lines = LineSegs()
        lines.setThickness(4)
        lines.setColor(*color, 1)
        lines.moveTo(0, 0, 0)
        lines.drawTo(1, 0, 0)
        return lines.create()
    
    def update_arrow(self, arrow_path, direction, scale, color):
        """Update arrow visualization"""
        # Remove old arrow
        for child in arrow_path.getChildren():
            child.removeNode()
        
        if np.linalg.norm(direction) < 0.001:
            return
        
        # Create new arrow
        lines = LineSegs()
        lines.setThickness(4)
        lines.setColor(*color, 1)
        lines.moveTo(0, 0, 0)
        
        vec = direction * scale
        lines.drawTo(vec[0], vec[1], vec[2])
        
        # Add arrowhead
        arrow_size = 0.1
        dir_norm = direction / np.linalg.norm(direction)
        perp1 = np.array([-dir_norm[1], dir_norm[0], 0])
        if np.linalg.norm(perp1) < 0.001:
            perp1 = np.array([0, -dir_norm[2], dir_norm[1]])
        perp1 = perp1 / np.linalg.norm(perp1) * arrow_size
        perp2 = np.cross(dir_norm, perp1) * arrow_size
        
        tip = vec
        base = vec - dir_norm * arrow_size * 2
        
        for p in [base + perp1, base - perp1, base + perp2, base - perp2]:
            lines.moveTo(*tip)
            lines.drawTo(*p)
        
        arrow_node = lines.create()
        arrow_path.attachNewNode(arrow_node)
    
    def setup_ui(self):
        """Setup UI text elements"""
        self.title_text = OnscreenText(
            text="IMU 3D Visualizer - Arduino Nano 33 BLE",
            pos=(0, 0.95), scale=0.06, fg=(1, 1, 1, 1),
            align=TextNode.ACenter
        )
        
        self.status_text = OnscreenText(
            text="Status: Initializing...",
            pos=(-1.5, 0.9), scale=0.04, fg=(1, 1, 0, 1),
            align=TextNode.ALeft
        )
        
        self.info_text = OnscreenText(
            text="",
            pos=(-1.5, 0.5), scale=0.035, fg=(1, 1, 1, 1),
            align=TextNode.ALeft
        )
        
        self.legend_text = OnscreenText(
            text="Red Top = Component Side\nYellow = Accelerometer\nMagenta = Magnetometer",
            pos=(-1.5, -0.7), scale=0.035, fg=(0.8, 0.8, 0.8, 1),
            align=TextNode.ALeft
        )
    
    def setup_graphs(self):
        """Setup graph visualization areas"""
        # Graph backgrounds
        self.graph_accel = self.create_graph_background((1.3, 0.7), (0.45, 0.25))
        self.graph_mag = self.create_graph_background((1.3, 0.35), (0.45, 0.25))
        self.graph_temp = self.create_graph_background((1.3, 0.0), (0.45, 0.25))
        
        # Graph labels
        OnscreenText(text="Accelerometer (m/s^2)", pos=(1.3, 0.87), scale=0.04,
                    fg=(1, 1, 1, 1), align=TextNode.ACenter)
        OnscreenText(text="Magnetometer (µT)", pos=(1.3, 0.52), scale=0.04,
                    fg=(1, 1, 1, 1), align=TextNode.ACenter)
        OnscreenText(text="Temperature (°C)", pos=(1.3, 0.17), scale=0.04,
                    fg=(1, 1, 1, 1), align=TextNode.ACenter)
        
        # Graph line storage
        self.graph_lines = {
            'accel': NodePath("accel_lines"),
            'mag': NodePath("mag_lines"),
            'temp': NodePath("temp_lines")
        }
        for node in self.graph_lines.values():
            node.reparentTo(self.aspect2d)
    
    def create_graph_background(self, pos, size):
        """Create a background panel for graphs"""
        cm = CardMaker('graph_bg')
        cm.setFrame(-size[0], size[0], -size[1], size[1])
        card = self.aspect2d.attachNewNode(cm.generate())
        card.setPos(pos[0], 0, pos[1])
        card.setColor(0.15, 0.15, 0.2, 0.8)
        return card
    
    def draw_graph(self, graph_node, pos, size, data_history, colors, y_range=None):
        """Draw a line graph"""
        # Clear previous lines
        for child in graph_node.getChildren():
            child.removeNode()
        
        if len(data_history) < 2:
            return
        
        data = np.array(data_history)
        n_points = len(data)
        
        # Auto-scale if no range provided
        if y_range is None:
            y_min, y_max = np.min(data), np.max(data)
            y_range = (y_min - 0.1 * abs(y_max - y_min), y_max + 0.1 * abs(y_max - y_min))
            if y_max == y_min:
                y_range = (y_min - 1, y_max + 1)
        
        # Draw each channel
        for channel in range(data.shape[1] if len(data.shape) > 1 else 1):
            lines = LineSegs()
            lines.setThickness(2)
            lines.setColor(*colors[channel], 1)
            
            for i in range(n_points):
                x = pos[0] - size[0] + (2 * size[0] * i / (n_points - 1))
                y_val = data[i, channel] if len(data.shape) > 1 else data[i]
                y_norm = (y_val - y_range[0]) / (y_range[1] - y_range[0])
                y = pos[1] - size[1] + (2 * size[1] * y_norm)
                
                if i == 0:
                    lines.moveTo(x+0.8, 0, y)
                else:
                    lines.drawTo(x+0.8, 0, y)
            
            line_node = lines.create()
            graph_node.attachNewNode(line_node)
    
    def update_scene(self, task):
        """Main update task"""
        data = self.imu_data.get_snapshot()
        
        # Update IMU box orientation
        q = data['quaternion']
        quat = LQuaternion(q[0], q[1], q[2], q[3])  
        self.imu_box.setQuat(quat)
        
        # Update accelerometer arrow (yellow)
        self.update_arrow(self.accel_arrow, data['accel'], 0.3, (1, 1, 0))
        
        # Update magnetometer arrow (magenta)
        self.update_arrow(self.mag_arrow, data['mag'], 0.01, (1, 0, 1))
        
        # Update status text
        status_color = (0, 1, 0) if data['connected'] else (1, 1, 0)
        self.status_text.setText(f"Status: {data['status']} | Packets: {data['packet_count']}")
        self.status_text.setFg((*status_color, 1))
        
        # Update info text
        a = data['accel']
        q = data['quaternion']
        m = data['mag']
        info = f"""Time: {data['timestamp']} ms
Temp: {data['temp']:.2f}°C

Accel (m/s^2):
  X: {a[0]:7.3f}
  Y: {a[1]:7.3f}
  Z: {a[2]:7.3f}

Quaternion:
  W: {q[0]:6.3f}
  X: {q[1]:6.3f}
  Y: {q[2]:6.3f}
  Z: {q[3]:6.3f}

Mag (µT):
  X: {m[0]:7.2f}
  Y: {m[1]:7.2f}
  Z: {m[2]:7.2f}"""
        self.info_text.setText(info)
        
        # Update graphs
        if len(data['accel_history']) > 1:
            self.draw_graph(self.graph_lines['accel'], (0.5, 0.7), (0.43, 0.23),
                          data['accel_history'], [(1,0,0), (0,1,0), (0,0,1)])
            self.draw_graph(self.graph_lines['mag'], (0.5, 0.35), (0.43, 0.23),
                          data['mag_history'], [(1,0,0), (0,1,0), (0,0,1)])
            self.draw_graph(self.graph_lines['temp'], (0.5, 0.0), (0.43, 0.23),
                          data['temp_history'], [(1,1,0)])
        
        return Task.cont


def ble_thread_func(receiver):
    """Function to run BLE in separate thread"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(receiver.connect_and_receive())
    except Exception as e:
        logger.error(f"BLE thread error: {e}")
    finally:
        loop.run_until_complete(receiver.disconnect())
        loop.stop()
        loop.close()


def main():
    logger.info("Starting Panda3D IMU Visualizer...")

    imu_data = IMUData()

    receiver = BLEReceiver(imu_data)
    ble_thread = threading.Thread(
        target=ble_thread_func,
        args=(receiver,),
        daemon=True
    )
    ble_thread.start()

    app = IMUVisualizer(imu_data)

    try:
        app.run()
    except Exception as e:
        logger.error(f"Visualizer error: {e}")
    finally:
        receiver.stop()
        logger.info("Waiting for BLE shutdown...")
        ble_thread.join(timeout=3)
        logger.info("Shutting down...")



if __name__ == "__main__":
    main()