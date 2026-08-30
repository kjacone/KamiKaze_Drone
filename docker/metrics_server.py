#!/usr/bin/env python3
"""
Simple metrics server for Prometheus
Runs on port 8001 (kamikaze), 8002 (predictive), 8003 (trajectory)
"""
import os
import sys
import time
import json
import subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

# Try to import prometheus_client, install if not available
try:
    from prometheus_client import Gauge, Counter, generate_latest, REGISTRY, Info
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "prometheus-client"])
    from prometheus_client import Gauge, Counter, generate_latest, REGISTRY, Info

# Default port from environment
PORT = int(os.environ.get('METRICS_PORT', 8001))
SERVICE_NAME = os.environ.get('SERVICE_NAME', 'kamikaze')

class MetricsCollector:
    def __init__(self):
        # Service Info
        self.service_info = Info('service_info', 'Service information')
        self.service_info.info({
            'name': SERVICE_NAME,
            'version': '1.0.0',
            'started': str(time.time())
        })
        
        # ROS Metrics
        self.node_count = Gauge('ros_node_count', 'Number of active ROS nodes')
        self.topic_count = Gauge('ros_topic_count', 'Number of active ROS topics')
        self.service_count = Gauge('ros_service_count', 'Number of active ROS services')
        
        # Service Metrics
        self.cpu_usage = Gauge('service_cpu_usage_percent', 'CPU usage percentage')
        self.memory_usage = Gauge('service_memory_usage_bytes', 'Memory usage in bytes')
        self.uptime = Gauge('service_uptime_seconds', 'Service uptime in seconds')
        self.start_time = time.time()
        
        # ROS Topic metrics
        self.topic_messages = {}
        
        # Start background collection
        self.running = True
        self.collect_thread = threading.Thread(target=self.collect_metrics)
        self.collect_thread.daemon = True
        self.collect_thread.start()
    
    def get_ros_nodes(self):
        try:
            result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=2)
            nodes = result.stdout.strip().split('\n')
            return [n for n in nodes if n and n != 'rosout']
        except:
            return []
    
    def get_ros_topics(self):
        try:
            result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=2)
            topics = result.stdout.strip().split('\n')
            return [t for t in topics if t]
        except:
            return []
    
    def get_ros_services(self):
        try:
            result = subprocess.run(['rosservice', 'list'], capture_output=True, text=True, timeout=2)
            services = result.stdout.strip().split('\n')
            return [s for s in services if s]
        except:
            return []
    
    def get_process_stats(self):
        try:
            import psutil
            process = psutil.Process()
            cpu = process.cpu_percent(interval=0.1)
            memory = process.memory_info().rss
            return cpu, memory
        except:
            return 0, 0
    
    def collect_metrics(self):
        while self.running:
            try:
                nodes = self.get_ros_nodes()
                topics = self.get_ros_topics()
                services = self.get_ros_services()
                
                self.node_count.set(len(nodes))
                self.topic_count.set(len(topics))
                self.service_count.set(len(services))
                
                cpu, memory = self.get_process_stats()
                self.cpu_usage.set(cpu)
                self.memory_usage.set(memory)
                self.uptime.set(time.time() - self.start_time)
                
                # Update topic counters
                for topic in topics:
                    if topic not in self.topic_messages:
                        self.topic_messages[topic] = Counter(
                            'ros_topic_messages_total',
                            'Total messages published',
                            ['topic']
                        )
                    self.topic_messages[topic].labels(topic=topic).inc()
                
            except Exception as e:
                print(f"Metrics collection error: {e}")
            
            time.sleep(5)

class MetricsHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/metrics':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; version=0.0.4')
            self.end_headers()
            metrics_data = generate_latest(REGISTRY)
            self.wfile.write(metrics_data)
            
        elif self.path == '/health':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "healthy",
                "service": SERVICE_NAME,
                "timestamp": time.time()
            }).encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        pass

def run_metrics_server():
    server_address = ('0.0.0.0', PORT)
    httpd = HTTPServer(server_address, MetricsHandler)
    print(f"✅ Metrics server running on port {PORT} for {SERVICE_NAME}")
    httpd.serve_forever()

if __name__ == '__main__':
    collector = MetricsCollector()
    run_metrics_server()