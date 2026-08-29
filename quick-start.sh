#!/bin/bash
# Quick start with all services

# Build images
./build.sh

# Start with monitoring
docker-compose -f docker/docker-compose.prod.yml --profile full up -d

# Check status
echo "Checking service status..."
sleep 10
docker ps | grep kamikaze

echo ""
echo "Services running:"
echo "  - Main drone: http://localhost:11311 (ROS)"
echo "  - Grafana: http://localhost:3000 (admin/admin)"
echo "  - Prometheus: http://localhost:9090"
echo "  - Node Exporter: http://localhost:9100"
echo "  - cAdvisor: http://localhost:8081"
echo ""
echo "View logs: docker-compose -f docker/docker-compose.prod.yml logs -f"