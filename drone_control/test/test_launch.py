#!/usr/bin/env python3
# drone_control/test/test_launch.py

import os
import subprocess
import time
import unittest

import rospy
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


class TestLaunchSystem(unittest.TestCase):
    """Tests for launch configuration and system startup"""
    
    @classmethod
    def setUpClass(cls):
        """Setup test environment"""
        cls.package_dir = get_package_share_directory('drone_control')
        cls.launch_dir = os.path.join(cls.package_dir, 'launch')
        cls.config_dir = os.path.join(cls.package_dir, 'config')
        cls.test_results_dir = os.path.join(cls.package_dir, 'test_results')
        
        # Create test results directory
        os.makedirs(cls.test_results_dir, exist_ok=True)
        
    def test_launch_file_syntax(self):
        """Test all launch files for syntax errors"""
        launch_files = ['kamikaze.launch', 'test.launch']
        
        for launch_file in launch_files:
            file_path = os.path.join(self.launch_dir, launch_file)
            self.assertTrue(os.path.exists(file_path), f"Launch file {launch_file} not found")
            
            # Check XML syntax
            try:
                import xml.etree.ElementTree as ET
                ET.parse(file_path)
            except ET.ParseError as e:
                self.fail(f"XML syntax error in {launch_file}: {e}")
                
    def test_config_file_loading(self):
        """Test all YAML config files are valid"""
        config_files = [
            'test_params.yaml',
            'debug_params.yaml', 
            'diagnostics.yaml',
            'production_params.yaml'
        ]
        
        for config_file in config_files:
            file_path = os.path.join(self.config_dir, config_file)
            self.assertTrue(os.path.exists(file_path), f"Config file {config_file} not found")
            
            # Check YAML syntax
            try:
                with open(file_path, 'r') as f:
                    yaml.safe_load(f)
            except yaml.YAMLError as e:
                self.fail(f"YAML syntax error in {config_file}: {e}")
                
    def test_launch_modes(self):
        """Test different launch modes"""
        modes = ['production', 'test', 'debug']
        
        for mode in modes:
            print(f"Testing {mode} mode...")
            
            # Build launch command
            launch_cmd = [
                'roslaunch',
                'drone_control',
                'kamikaze.launch',
                f'mode:={mode}',
                'simulation:=true',
                'gui:=false'
            ]
            
            # Start process
            process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Wait for startup
            time.sleep(5)
            
            # Check if nodes are running
            try:
                nodes = rospy.get_published_topics()
                
                # Check for mode-specific nodes
                if mode == 'test':
                    self._check_test_nodes()
                elif mode == 'debug':
                    self._check_debug_nodes()
                else:
                    self._check_production_nodes()
                    
            except Exception as e:
                self.fail(f"Error checking nodes in {mode} mode: {e}")
            finally:
                # Cleanup
                process.terminate()
                process.wait(timeout=5)
                
    def _check_test_nodes(self):
        """Check test mode nodes are running"""
        topics = rospy.get_published_topics()
        topic_names = [t[0] for t in topics]
        
        # Check for test-specific topics
        self.assertIn('/test_detections', topic_names)
        self.assertIn('/test_results', topic_names)
        
    def _check_debug_nodes(self):
        """Check debug mode nodes are running"""
        topics = rospy.get_published_topics()
        topic_names = [t[0] for t in topics]
        
        # Check for debug-specific topics
        self.assertIn('/debug_info', topic_names)
        self.assertIn('/profiling_data', topic_names)
        
    def _check_production_nodes(self):
        """Check production mode nodes are running"""
        topics = rospy.get_published_topics()
        topic_names = [t[0] for t in topics]
        
        # Check for essential topics
        essential_topics = [
            '/target_detections',
            '/tracked_targets', 
            '/control_command',
            '/safety_status'
        ]
        
        for topic in essential_topics:
            self.assertIn(topic, topic_names, f"Essential topic {topic} not found")
            
    def test_scenario_configs(self):
        """Test all scenario configuration files"""
        scenarios_dir = os.path.join(self.config_dir, 'test_scenarios')
        self.assertTrue(os.path.exists(scenarios_dir), "Scenarios directory not found")
        
        scenario_files = ['default.yaml', 'edge_cases.yaml', 'stress.yaml']
        
        for scenario_file in scenario_files:
            file_path = os.path.join(scenarios_dir, scenario_file)
            self.assertTrue(os.path.exists(file_path), f"Scenario file {scenario_file} not found")
            
            # Validate scenario content
            try:
                with open(file_path, 'r') as f:
                    scenario = yaml.safe_load(f)
                    
                self.assertIn('scenario', scenario)
                self.assertIn('duration', scenario['scenario'])
                self.assertIn('target_count', scenario['scenario'])
                self.assertIn('target_behavior', scenario['scenario'])
                
            except Exception as e:
                self.fail(f"Error validating scenario {scenario_file}: {e}")
                
    def test_ros_node_health(self):
        """Test all nodes start and remain healthy"""
        # Launch system
        launch_cmd = [
            'roslaunch',
            'drone_control',
            'kamikaze.launch',
            'simulation:=true',
            'gui:=false'
        ]
        
        process = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        try:
            # Wait for system to stabilize
            time.sleep(10)
            
            # Check node health
            node_names = rospy.get_published_topics()
            self.assertGreater(len(node_names), 5, "Fewer nodes than expected")
            
            # Check each essential node
            essential_nodes = ['safety_monitor', 'health_checker']
            for node in essential_nodes:
                self.assertTrue(
                    rospy.has_param(f'/{node}/status'),
                    f"Node {node} not responsive"
                )
                
        finally:
            process.terminate()
            process.wait(timeout=5)
            
    def test_error_handling(self):
        """Test error handling and recovery"""
        # Launch with invalid parameter
        launch_cmd = [
            'roslaunch',
            'drone_control',
            'kamikaze.launch',
            'simulation:=true',
            'test_scenario:=invalid_scenario'
        ]
        
        process = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Should gracefully handle invalid scenario
        time.sleep(3)
        
        # Check process exited or continued
        return_code = process.poll()
        if return_code is not None:
            # Should exit with error code
            self.assertNotEqual(return_code, 0)
            
    def test_memory_usage(self):
        """Test memory usage during launch"""
        import psutil
        
        # Launch system
        launch_cmd = [
            'roslaunch',
            'drone_control',
            'kamikaze.launch',
            'simulation:=true',
            'gui:=false'
        ]
        
        process = subprocess.Popen(launch_cmd)
        
        try:
            time.sleep(5)
            
            # Check memory usage
            process = psutil.Process(process.pid)
            memory_info = process.memory_info()
            
            memory_mb = memory_info.rss / 1024 / 1024
            self.assertLess(memory_mb, 1024, "Memory usage exceeds 1GB")
            
        finally:
            process.terminate()
            process.wait(timeout=5)

if __name__ == '__main__':
    unittest.main(verbosity=2)