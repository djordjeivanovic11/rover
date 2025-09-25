#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
from std_msgs.msg import String
from zed_integration.srv import ConvertCoordinates
import math
import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class UTMCoordinate:
    """UTM coordinate representation"""
    easting: float
    northing: float
    altitude: float
    zone: str
    hemisphere: str


@dataclass
class MapCoordinate:
    """Local map coordinate representation"""
    x: float
    y: float
    z: float
    frame_id: str = 'map'


class CoordinateConverter(Node):
    """Coordinate conversion service node"""
    
    def __init__(self):
        super().__init__('coordinate_converter')
        
        # Parameters
        self.declare_parameter('map_origin_lat', 0.0)
        self.declare_parameter('map_origin_lon', 0.0)
        self.declare_parameter('map_origin_alt', 0.0)
        self.declare_parameter('auto_set_origin', True)
        self.declare_parameter('utm_zone', '')
        
        # Get parameters
        self.origin_lat = self.get_parameter('map_origin_lat').value
        self.origin_lon = self.get_parameter('map_origin_lon').value
        self.origin_alt = self.get_parameter('map_origin_alt').value
        self.auto_set_origin = self.get_parameter('auto_set_origin').value
        self.utm_zone = self.get_parameter('utm_zone').value
        
        # State
        self.origin_set = (self.origin_lat != 0.0 or self.origin_lon != 0.0)
        self.utm_origin: Optional[UTMCoordinate] = None
        
        # Subscribers
        self.create_subscription(GeoPoseStamped, '/camera/geo_pose', self.cb_geo_pose, 10)
        
        # Publishers
        self.origin_pub = self.create_publisher(String, '/coordinate/origin_status', 10)
        self.converted_pose_pub = self.create_publisher(PoseStamped, '/coordinate/converted_pose', 10)
        
        # Services
        self.create_service(ConvertCoordinates, '/coordinate/convert', self.convert_coordinates)
        
        # Initialize origin if set
        if self.origin_set:
            self.utm_origin = self.gps_to_utm(self.origin_lat, self.origin_lon, self.origin_alt)
            self.get_logger().info(f"📍 Map origin set: {self.origin_lat:.6f}, {self.origin_lon:.6f}")
        
        self.get_logger().info("🗺️ Coordinate Converter initialized")
    
    def cb_geo_pose(self, msg: GeoPoseStamped):
        """Handle geographic pose updates"""
        if self.auto_set_origin and not self.origin_set:
            self.origin_lat = msg.pose.position.latitude
            self.origin_lon = msg.pose.position.longitude
            self.origin_alt = msg.pose.position.altitude
            self.origin_set = True
            
            self.utm_origin = self.gps_to_utm(self.origin_lat, self.origin_lon, self.origin_alt)
            
            self.get_logger().info(f"🌍 Auto-set map origin: {self.origin_lat:.6f}, {self.origin_lon:.6f}")
            
            origin_msg = String()
            origin_msg.data = f"origin_set:{self.origin_lat:.6f},{self.origin_lon:.6f},{self.origin_alt:.1f}"
            self.origin_pub.publish(origin_msg)
    
    def convert_coordinates(self, request, response):
        """Convert between coordinate systems"""
        try:
            if request.conversion_type == 'gps_to_map':
                if not self.origin_set:
                    response.success = False
                    response.message = "Map origin not set"
                    return response
                
                map_coord = self.gps_to_map(request.latitude, request.longitude, request.altitude)
                
                response.success = True
                response.map_x = map_coord.x
                response.map_y = map_coord.y
                response.map_z = map_coord.z
                response.message = f"Converted GPS({request.latitude:.6f}, {request.longitude:.6f}) to Map({map_coord.x:.2f}, {map_coord.y:.2f})"
                
            elif request.conversion_type == 'map_to_gps':
                if not self.origin_set:
                    response.success = False
                    response.message = "Map origin not set"
                    return response
                
                lat, lon, alt = self.map_to_gps(request.map_x, request.map_y, request.map_z)
                
                response.success = True
                response.latitude = lat
                response.longitude = lon
                response.altitude = alt
                response.message = f"Converted Map({request.map_x:.2f}, {request.map_y:.2f}) to GPS({lat:.6f}, {lon:.6f})"
                
            else:
                response.success = False
                response.message = f"Unknown conversion type: {request.conversion_type}"
            
        except Exception as e:
            response.success = False
            response.message = f"Conversion failed: {str(e)}"
            self.get_logger().error(f"Coordinate conversion error: {e}")
        
        return response
    
    def gps_to_utm(self, lat: float, lon: float, alt: float) -> UTMCoordinate:
        """Convert GPS coordinates to UTM (simplified)"""
        zone_number = int((lon + 180) / 6) + 1
        zone_letter = 'N' if lat >= 0 else 'S'
        
        # Simplified UTM conversion
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        k0 = 0.9996
        e = 0.0818191908
        
        lon_origin = (zone_number - 1) * 6 - 180 + 3
        lon_origin_rad = math.radians(lon_origin)
        
        n = k0 * 6378137.0 / math.sqrt(1 - e * e * math.sin(lat_rad) * math.sin(lat_rad))
        t = math.tan(lat_rad) * math.tan(lat_rad)
        c = (e * e / (1 - e * e)) * math.cos(lat_rad) * math.cos(lat_rad)
        a = math.cos(lat_rad) * (lon_rad - lon_origin_rad)
        
        utm_x = k0 * n * (a + (1 - t + c) * a * a * a / 6) + 500000.0
        utm_y = k0 * (lat_rad + (1 - t + c) * lat_rad * a * a / 2)
        
        if lat < 0:
            utm_y += 10000000.0
        
        return UTMCoordinate(
            easting=utm_x,
            northing=utm_y,
            altitude=alt,
            zone=f"{zone_number}{zone_letter}",
            hemisphere=zone_letter
        )
    
    def gps_to_map(self, lat: float, lon: float, alt: float) -> MapCoordinate:
        """Convert GPS coordinates to local map frame"""
        if not self.utm_origin:
            raise ValueError("UTM origin not set")
        
        target_utm = self.gps_to_utm(lat, lon, alt)
        
        rel_x = target_utm.easting - self.utm_origin.easting
        rel_y = target_utm.northing - self.utm_origin.northing
        rel_z = target_utm.altitude - self.utm_origin.altitude
        
        return MapCoordinate(x=rel_x, y=rel_y, z=rel_z)
    
    def map_to_gps(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Convert local map coordinates to GPS"""
        if not self.utm_origin:
            raise ValueError("UTM origin not set")
        
        # Simplified reverse conversion
        lat = self.origin_lat + (y / 111000.0)
        lon = self.origin_lon + (x / (111000.0 * math.cos(math.radians(self.origin_lat))))
        alt = self.origin_alt + z
        
        return lat, lon, alt


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        converter = CoordinateConverter()
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
