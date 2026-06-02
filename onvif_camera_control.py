"""
ONVIF camera control implementation for the laser rangefinder desktop application.
Provides PTZ (Pan-Tilt-Zoom) control functionality using ONVIF protocol.
"""

import threading
from onvif import ONVIFService, ONVIFCamera
from urllib.parse import urlparse


class OnvifCameraController:
    """Class for controlling ONVIF-compatible cameras."""
    
    def __init__(self, ip, username, password, port=80):
        self.ip = ip
        self.username = username
        self.password = password
        self.port = port
        self.camera = None
        self.ptz_service = None
        self.media_service = None
        self.profiles = None
        self.active = False
        
    def connect(self):
        """Establish connection to ONVIF camera."""
        try:
            self.camera = ONVIFCamera(self.ip, self.port, self.username, self.password)
            # Create services
            self.media_service = self.camera.create_media_service()
            self.ptz_service = self.camera.create_ptz_service()
            
            # Get profiles
            self.profiles = self.media_service.GetProfiles()
            self.active = True
            return True
        except Exception as e:
            print(f"Failed to connect to ONVIF camera: {e}")
            return False
    
    def disconnect(self):
        """Close connection to ONVIF camera."""
        self.active = False
        if self.camera:
            del self.camera
            self.camera = None
        self.ptz_service = None
        self.media_service = None
        self.profiles = None
    
    def get_ptz_configurations(self):
        """Get available PTZ configurations."""
        if not self.ptz_service or not self.active:
            return None
        try:
            return self.ptz_service.GetConfigurations()
        except:
            return None
    
    def get_presets(self):
        """Get available PTZ presets."""
        if not self.ptz_service or not self.active:
            return None
        try:
            return self.ptz_service.GetPresets({'ProfileToken': self.profiles[0].token})
        except:
            return None
    
    def absolute_move(self, x=None, y=None, z=None):
        """Perform absolute PTZ movement."""
        if not self.ptz_service or not self.active:
            return False
        try:
            req = self.ptz_service.create_type('AbsoluteMove')
            req.ProfileToken = self.profiles[0].token
            if not hasattr(req, 'Position') or req.Position is None:
                from onvif import zeep
                req.Position = {}
            
            if x is not None:
                req.Position.PanTilt = {'x': x, 'y': y, 'space': 'http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace'}
            if z is not None:
                req.Position.Zoom = {'x': z, 'space': 'http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace'}
            
            self.ptz_service.AbsoluteMove(req)
            return True
        except Exception as e:
            print(f"Failed to perform absolute move: {e}")
            return False
    
    def relative_move(self, x=None, y=None, z=None):
        """Perform relative PTZ movement."""
        if not self.ptz_service or not self.active:
            return False
        try:
            req = self.ptz_service.create_type('RelativeMove')
            req.ProfileToken = self.profiles[0].token
            if not hasattr(req, 'Translation') or req.Translation is None:
                req.Translation = {}
            
            if x is not None or y is not None:
                req.Translation.PanTilt = {'x': x or 0, 'y': y or 0, 'space': 'http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace'}
            if z is not None:
                req.Translation.Zoom = {'x': z, 'space': 'http://www.onvif.org/ver10/tptz/ZoomSpaces/VelocityGenericSpace'}
            
            self.ptz_service.RelativeMove(req)
            return True
        except Exception as e:
            print(f"Failed to perform relative move: {e}")
            return False
    
    def continuous_move(self, x=None, y=None, z=None, timeout=1.0):
        """Perform continuous PTZ movement."""
        if not self.ptz_service or not self.active:
            return False
        try:
            req = self.ptz_service.create_type('ContinuousMove')
            req.ProfileToken = self.profiles[0].token
            if not hasattr(req, 'Velocity') or req.Velocity is None:
                req.Velocity = {}
            
            if x is not None or y is not None:
                req.Velocity.PanTilt = {'x': x or 0, 'y': y or 0, 'space': 'http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace'}
            if z is not None:
                req.Velocity.Zoom = {'x': z, 'space': 'http://www.onvif.org/ver10/tptz/ZoomSpaces/VelocityGenericSpace'}
            
            if timeout:
                req.Timeout = timeout
            
            self.ptz_service.ContinuousMove(req)
            return True
        except Exception as e:
            print(f"Failed to perform continuous move: {e}")
            return False
    
    def stop_move(self, pan_tilt=True, zoom=True):
        """Stop PTZ movement."""
        if not self.ptz_service or not self.active:
            return False
        try:
            req = self.ptz_service.create_type('Stop')
            req.ProfileToken = self.profiles[0].token
            req.PanTilt = pan_tilt
            req.Zoom = zoom
            self.ptz_service.Stop(req)
            return True
        except Exception as e:
            print(f"Failed to stop movement: {e}")
            return False
    
    def zoom_in(self, speed=0.5):
        """Zoom in with specified speed."""
        return self.relative_move(z=speed)
    
    def zoom_out(self, speed=0.5):
        """Zoom out with specified speed."""
        return self.relative_move(z=-speed)
    
    def goto_preset(self, preset_token):
        """Go to a specific preset position."""
        if not self.ptz_service or not self.active:
            return False
        try:
            req = self.ptz_service.create_type('GotoPreset')
            req.ProfileToken = self.profiles[0].token
            req.PresetToken = preset_token
            self.ptz_service.GotoPreset(req)
            return True
        except Exception as e:
            print(f"Failed to go to preset: {e}")
            return False


class PelcoDController:
    """Class for controlling cameras using Pelco-D protocol over serial connection."""
    
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.serial_conn = None
        self.active = False
    
    def connect(self):
        """Connect to serial port for Pelco-D communication."""
        try:
            import serial
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=2400,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1
            )
            self.active = True
            return True
        except Exception as e:
            print(f"Failed to connect to serial port for Pelco-D: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        self.active = False
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
    
    def send_pelco_d_command(self, address, command1, command2, data1, data2):
        """Send a generic Pelco-D command."""
        if not self.active:
            return False
        
        try:
            # Pelco-D packet format: [0xFF][address][command1][command2][data1][data2][checksum]
            checksum = (address + command1 + command2 + data1 + data2) & 0xFF
            packet = bytearray([0xFF, address, command1, command2, data1, data2, checksum])
            self.serial_conn.write(packet)
            return True
        except Exception as e:
            print(f"Failed to send Pelco-D command: {e}")
            return False
    
    def zoom_in(self, speed=0x30):
        """Zoom in command."""
        # Command: Zoom Tele (Zoom In)
        return self.send_pelco_d_command(0x01, 0x00, 0x08, speed, 0x00)
    
    def zoom_out(self, speed=0x30):
        """Zoom out command."""
        # Command: Zoom Wide (Zoom Out)
        return self.send_pelco_d_command(0x01, 0x00, 0x10, speed, 0x00)
    
    def zoom_stop(self):
        """Stop zoom command."""
        return self.send_pelco_d_command(0x01, 0x00, 0x00, 0x00, 0x00)
    
    def pan_tilt_move(self, pan_speed, tilt_speed):
        """Move pan/tilt with specified speeds."""
        cmd1 = 0x00
        # Determine direction bits for pan
        if pan_speed > 0:
            cmd1 |= 0x40  # Right
        elif pan_speed < 0:
            cmd1 |= 0x20  # Left
        
        # Determine direction bits for tilt
        if tilt_speed > 0:
            cmd1 |= 0x08  # Up
        elif tilt_speed < 0:
            cmd1 |= 0x10  # Down
        
        abs_pan = min(abs(pan_speed), 0x3F)
        abs_tilt = min(abs(tilt_speed), 0x3F)
        
        return self.send_pelco_d_command(0x01, cmd1, 0x00, abs_pan, abs_tilt)
    
    def pan_tilt_stop(self):
        """Stop pan/tilt movement."""
        return self.send_pelco_d_command(0x01, 0x00, 0x00, 0x00, 0x00)