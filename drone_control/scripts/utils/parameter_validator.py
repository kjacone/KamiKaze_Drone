#!/usr/bin/env python3
# drone_control/scripts/utils/parameter_validator.py

import rospy
import yaml
import os
import json
from typing import Any, Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

class ParameterType(Enum):
    """Parameter data types"""
    INT = "int"
    FLOAT = "float"
    BOOL = "bool"
    STRING = "string"
    LIST = "list"
    DICT = "dict"
    FILE = "file"
    PATH = "path"
    POSITIVE_INT = "positive_int"
    POSITIVE_FLOAT = "positive_float"
    PROBABILITY = "probability"
    ANGLE = "angle"
    VELOCITY = "velocity"
    ACCELERATION = "acceleration"

@dataclass
class ParameterSchema:
    """Schema definition for a parameter"""
    name: str
    type: ParameterType
    required: bool = True
    default: Any = None
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    allowed_values: Optional[List[Any]] = None
    description: str = ""
    regex: Optional[str] = None

class ParameterValidator:
    """Validates and manages system parameters"""
    
    def __init__(self):
        rospy.init_node('parameter_validator', anonymous=False)
        
        # Get parameters
        self.config_dir = rospy.get_param('~config_dir', '/drone_control/config')
        self.validate_on_startup = rospy.get_param('~validate_on_startup', True)
        self.generate_schema = rospy.get_param('~generate_schema', False)
        self.schema_output = rospy.get_param('~schema_output', '/tmp/parameter_schema.json')
        
        # Load schemas
        self.schemas = self._load_schemas()
        
        # Validate on startup
        if self.validate_on_startup:
            self.validate_all_parameters()
        
        # Generate schema documentation
        if self.generate_schema:
            self.generate_schema_documentation(self.schema_output)
        
        rospy.loginfo("Parameter validator initialized successfully")
        
    def _load_schemas(self) -> Dict[str, List[ParameterSchema]]:
        """Load parameter schemas"""
        schemas = {}
        
        # Define schemas for each config file
        schema_definitions = {
            "target_params": self._get_target_params_schema(),
            "camera_calib": self._get_camera_calib_schema(),
            "flight_control": self._get_flight_control_schema(),
            "kalman_filter": self._get_kalman_filter_schema(),
            "system_params": self._get_system_params_schema()
        }
        
        return schema_definitions
        
    def _get_target_params_schema(self) -> List[ParameterSchema]:
        """Schema for target_params.yaml"""
        return [
            ParameterSchema(
                "detection/confidence_threshold", 
                ParameterType.PROBABILITY,
                default=0.5,
                description="Minimum confidence for detection"
            ),
            ParameterSchema(
                "detection/nms_threshold",  # Fixed: changed dot to slash
                ParameterType.PROBABILITY,
                default=0.4,
                description="Non-maximum suppression threshold"
            ),
            ParameterSchema(
                "detection/vehicle_classes",  # Fixed: changed dot to slash
                ParameterType.LIST,
                default=[1, 2, 3, 5, 6, 7, 8],
                description="COCO class IDs to track"
            ),
            ParameterSchema(
                "tracking/engagement_distance",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=10.0,
                min_value=1.0,
                max_value=50.0,
                description="Distance to start tracking in meters"
            ),
            ParameterSchema(
                "tracking/attack_distance",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=2.0,
                min_value=0.5,
                max_value=10.0,
                description="Distance to engage target in meters"
            ),
            ParameterSchema(
                "tracking/kalman/process_noise",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=0.1,
                min_value=0.01,
                max_value=1.0,
                description="Kalman filter process noise"
            ),
            ParameterSchema(
                "tracking/kalman/measurement_noise",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=0.05,
                min_value=0.001,
                max_value=0.5,
                description="Kalman filter measurement noise"
            )
        ]
        
    def _get_camera_calib_schema(self) -> List[ParameterSchema]:
        """Schema for camera_calib.yaml"""
        return [
            ParameterSchema(
                "camera/width",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_INT,
                default=640,
                min_value=1,
                description="Camera width in pixels"
            ),
            ParameterSchema(
                "camera/height",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_INT,
                default=480,
                min_value=1,
                description="Camera height in pixels"
            ),
            ParameterSchema(
                "camera/intrinsic/fx",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=500.0,
                min_value=1.0,
                description="Focal length x in pixels"
            ),
            ParameterSchema(
                "camera/intrinsic/fy",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=500.0,
                min_value=1.0,
                description="Focal length y in pixels"
            ),
            ParameterSchema(
                "camera/intrinsic/cx",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=320.0,
                min_value=0,
                description="Principal point x in pixels"
            ),
            ParameterSchema(
                "camera/intrinsic/cy",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=240.0,
                min_value=0,
                description="Principal point y in pixels"
            )
        ]
        
    def _get_flight_control_schema(self) -> List[ParameterSchema]:
        """Schema for flight_control.yaml"""
        return [
            ParameterSchema(
                "dynamics/constraints/max_velocity",  # Fixed: changed dot to slash
                ParameterType.VELOCITY,
                default=5.0,
                min_value=0.5,
                max_value=20.0,
                description="Maximum velocity in m/s"
            ),
            ParameterSchema(
                "dynamics/constraints/max_acceleration",  # Fixed: changed dot to slash
                ParameterType.ACCELERATION,
                default=2.0,
                min_value=0.5,
                max_value=10.0,
                description="Maximum acceleration in m/s^2"
            ),
            ParameterSchema(
                "dynamics/safety_margins/min_altitude",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=2.0,
                min_value=0.5,
                max_value=10.0,
                description="Minimum safe altitude in meters"
            ),
            ParameterSchema(
                "dynamics/safety_margins/max_altitude",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=50.0,
                min_value=10.0,
                max_value=200.0,
                description="Maximum safe altitude in meters"
            ),
            ParameterSchema(
                "mission/collision_avoidance/min_distance",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=1.5,
                min_value=0.5,
                max_value=10.0,
                description="Minimum collision avoidance distance in meters"
            ),
            ParameterSchema(
                "mission/engagement/mode",  # Fixed: changed dot to slash
                ParameterType.STRING,
                default="autonomous",
                allowed_values=["autonomous", "manual", "semi_autonomous"],
                description="Engagement mode"
            )
        ]
        
    def _get_kalman_filter_schema(self) -> List[ParameterSchema]:
        """Schema for kalman_filter.yaml"""
        return [
            ParameterSchema(
                "kalman_filter/state_dim",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_INT,
                default=9,
                min_value=3,
                max_value=12,
                description="State vector dimension"
            ),
            ParameterSchema(
                "kalman_filter/noise/process/position",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=0.1,
                min_value=0.001,
                max_value=1.0,
                description="Position process noise"
            ),
            ParameterSchema(
                "kalman_filter/noise/process/velocity",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=0.05,
                min_value=0.001,
                max_value=0.5,
                description="Velocity process noise"
            ),
            ParameterSchema(
                "kalman_filter/noise/measurement/position",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=0.05,
                min_value=0.001,
                max_value=0.5,
                description="Position measurement noise"
            ),
            ParameterSchema(
                "kalman_filter/parameters/dt",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=0.1,
                min_value=0.01,
                max_value=1.0,
                description="Kalman filter time step"
            )
        ]
        
    def _get_system_params_schema(self) -> List[ParameterSchema]:
        """Schema for system_params.yaml"""
        return [
            ParameterSchema(
                "safety/geofence/radius",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=100.0,
                min_value=10.0,
                max_value=500.0,
                description="Geofence radius in meters"
            ),
            ParameterSchema(
                "safety/geofence/height",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=50.0,
                min_value=5.0,
                max_value=200.0,
                description="Geofence height in meters"
            ),
            ParameterSchema(
                "safety/failsafe/battery_critical",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=15.0,
                min_value=5.0,
                max_value=50.0,
                description="Critical battery percentage"
            ),
            ParameterSchema(
                "safety/failsafe/lost_connection",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_FLOAT,
                default=5.0,
                min_value=1.0,
                max_value=30.0,
                description="Lost connection timeout in seconds"
            ),
            ParameterSchema(
                "performance/publish_rate",  # Fixed: changed dot to slash
                ParameterType.POSITIVE_INT,
                default=50,
                min_value=10,
                max_value=200,
                description="Publish rate in Hz"
            )
        ]
        
    def validate_all_parameters(self) -> Tuple[bool, List[str]]:
        """Validate all parameters against schemas"""
        errors = []
        warnings = []
        
        for config_name, schemas in self.schemas.items():
            for schema in schemas:
                param_path = f"/{config_name}/{schema.name}"
                value = rospy.get_param(param_path, None)
                
                if value is None:
                    if schema.required:
                        errors.append(f"Required parameter {param_path} is missing")
                        if schema.default is not None:
                            rospy.set_param(param_path, schema.default)
                            warnings.append(f"Set default value for {param_path}: {schema.default}")
                    continue
                    
                is_valid, error_msg = self._validate_parameter(value, schema)
                if not is_valid:
                    errors.append(f"Parameter {param_path}: {error_msg}")
                    
        if errors:
            for error in errors:
                rospy.logerr(f"Parameter validation error: {error}")
                
        if warnings:
            for warning in warnings:
                rospy.logwarn(f"Parameter validation warning: {warning}")
                
        valid = len(errors) == 0
        if valid:
            rospy.loginfo("All parameters validated successfully")
        else:
            rospy.logerr(f"Parameter validation failed with {len(errors)} errors")
            
        return valid, errors + warnings
        
    def _validate_parameter(self, value: Any, schema: ParameterSchema) -> Tuple[bool, str]:
        """Validate a single parameter against its schema"""
        # Type checking
        if schema.type == ParameterType.INT:
            if not isinstance(value, int):
                return False, f"Expected int, got {type(value).__name__}"
                
        elif schema.type == ParameterType.POSITIVE_INT:
            if not isinstance(value, int) or value <= 0:
                return False, f"Expected positive int, got {value}"
                
        elif schema.type == ParameterType.FLOAT:
            if not isinstance(value, (int, float)):
                return False, f"Expected float, got {type(value).__name__}"
                
        elif schema.type == ParameterType.POSITIVE_FLOAT:
            if not isinstance(value, (int, float)) or value <= 0:
                return False, f"Expected positive float, got {value}"
                
        elif schema.type == ParameterType.PROBABILITY:
            if not isinstance(value, (int, float)) or not (0 <= value <= 1):
                return False, f"Expected probability (0-1), got {value}"
                
        elif schema.type == ParameterType.STRING:
            if not isinstance(value, str):
                return False, f"Expected string, got {type(value).__name__}"
                
        elif schema.type == ParameterType.LIST:
            if not isinstance(value, list):
                return False, f"Expected list, got {type(value).__name__}"
                
        elif schema.type == ParameterType.DICT:
            if not isinstance(value, dict):
                return False, f"Expected dict, got {type(value).__name__}"
                
        # Value range checking
        if schema.min_value is not None:
            if isinstance(value, (int, float)) and value < schema.min_value:
                return False, f"Value {value} is below minimum {schema.min_value}"
                
        if schema.max_value is not None:
            if isinstance(value, (int, float)) and value > schema.max_value:
                return False, f"Value {value} exceeds maximum {schema.max_value}"
                
        # Allowed values checking
        if schema.allowed_values is not None:
            if value not in schema.allowed_values:
                return False, f"Value '{value}' not in allowed values {schema.allowed_values}"
                
        return True, "OK"
        
    def generate_schema_documentation(self, output_file: str):
        """Generate JSON schema documentation"""
        schema_doc = {
            "version": "1.0",
            "description": "Drone Control System Parameter Schemas",
            "schemas": {}
        }
        
        for config_name, schemas in self.schemas.items():
            schema_doc["schemas"][config_name] = []
            
            for schema in schemas:
                schema_entry = {
                    "name": schema.name,
                    "type": schema.type.value,
                    "required": schema.required,
                    "default": schema.default,
                    "description": schema.description
                }
                
                if schema.min_value is not None:
                    schema_entry["min_value"] = schema.min_value
                    
                if schema.max_value is not None:
                    schema_entry["max_value"] = schema.max_value
                    
                if schema.allowed_values is not None:
                    schema_entry["allowed_values"] = schema.allowed_values
                    
                schema_doc["schemas"][config_name].append(schema_entry)
                
        # Write to file
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        with open(output_file, 'w') as f:
            json.dump(schema_doc, f, indent=2)
            
        rospy.loginfo(f"Schema documentation generated at {output_file}")

if __name__ == "__main__":
    try:
        validator = ParameterValidator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass