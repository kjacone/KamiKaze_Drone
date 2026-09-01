# 🚁 Kamikaze Drone — Docker Build Guide

Docker build instructions for the Kamikaze Drone ROS system.

> **Platform:** `linux/amd64`
> These commands are intended for building AMD64 images, including when developing on Apple Silicon (`arm64`) Macs.

---

## Prerequisites

Make sure Docker is installed and running:

```bash
docker --version
docker buildx version
```

If you are pushing images to Docker Hub, log in first:

```bash
docker login
```

---

## 1. Build the Base Image

Build the shared base image for AMD64:

```bash
docker buildx build \
  --platform linux/amd64 \
  -f Dockerfile.base \
  -t kjacone/kamikaze-drone-base:latest \
  --load \
  .
```

This image is loaded into your local Docker image store and can be used by subsequent local builds.

---

## 2. Build the Main Drone Image

Build the main Kamikaze Drone image:

```bash
docker buildx build \
  --platform linux/amd64 \
  -f Dockerfile \
  -t kjacone/kamikaze-drone:latest \
  --load \
  .
```

Check the image:

```bash
docker images | grep kamikaze-drone
```

---

## 3. Build and Push Python Dependencies

The Python dependency image changes less frequently, so it can be built and pushed to Docker Hub:

```bash
docker buildx build \
  --platform linux/amd64 \
  -f docker/Dockerfile.python-deps \
  -t kjacone/kamikaze-drone-python:latest \
  --push \
  .
```

---

## 4. Build and Push Predictive Base

Build and push the predictive-controller base image:

```bash
docker buildx build \
  --platform linux/amd64 \
  -f docker/Dockerfile.predictive-base \
  -t kjacone/kamikaze-drone-predictive:latest \
  --push \
  .
```

---

# Frequently Changing Images

Images that change frequently can be built locally using `--load`. This avoids pushing every development build to Docker Hub.

## Main Drone

```bash
docker buildx build \
  --platform linux/amd64 \
  -f Dockerfile \
  -t kamikaze-drone:latest \
  --load \
  .
```

## Predictive Controller

```bash
docker buildx build \
  --platform linux/amd64 \
  -f docker/Dockerfile.predictive \
  -t predictive-controller:latest \
  --load \
  .
```

## Trajectory Planner

```bash
docker buildx build \
  --platform linux/amd64 \
  -f docker/Dockerfile.trajectory \
  -t trajectory-planner:latest \
  --load \
  .
```

## Performance Monitor

```bash
docker buildx build \
  --platform linux/amd64 \
  -f docker/Dockerfile.performance \
  -t performance-monitor:latest \
  --load \
  .
```

---

# Build All Frequently Changing Images

You can run the following commands sequentially:

```bash
docker buildx build \
  --platform linux/amd64 \
  -f Dockerfile \
  -t kamikaze-drone:latest \
  --load \
  .

docker buildx build \
  --platform linux/amd64 \
  -f docker/Dockerfile.predictive \
  -t predictive-controller:latest \
  --load \
  .

docker buildx build \
  --platform linux/amd64 \
  -f docker/Dockerfile.trajectory \
  -t trajectory-planner:latest \
  --load \
  .

docker buildx build \
  --platform linux/amd64 \
  -f docker/Dockerfile.performance \
  -t performance-monitor:latest \
  --load \
  .
```

---

# Verify Image Platforms

After building, verify that the images were created for AMD64:

```bash
docker image inspect kamikaze-drone:latest \
  --format '{{.Os}}/{{.Architecture}}'
```

Expected:

```text
linux/amd64
```

You can check all relevant images with:

```bash
docker images | grep -E 'kamikaze|predictive|trajectory|performance'
```

---

# Build vs Push

| Image                       | Build Location | Push to Docker Hub |
| --------------------------- | -------------- | ------------------ |
| `kamikaze-drone-base`       | AMD64          | ✅                  |
| `kamikaze-drone`            | Local          | ❌                  |
| `kamikaze-drone-python`     | AMD64          | ✅                  |
| `kamikaze-drone-predictive` | AMD64          | ✅                  |
| `predictive-controller`     | Local          | ❌                  |
| `trajectory-planner`        | Local          | ❌                  |
| `performance-monitor`       | Local          | ❌                  |

### `--load`

Use `--load` when you want the resulting image available in your local Docker environment:

```bash
--load
```

### `--push`

Use `--push` when you want to send the image directly to Docker Hub:

```bash
--push
```

---

# Docker Hub Images

The published images are available under:

```text
kjacone/kamikaze-drone-base:latest
kjacone/kamikaze-drone-python:latest
kjacone/kamikaze-drone-predictive:latest
```

Pull an image on an AMD64 machine with:

```bash
docker pull kjacone/kamikaze-drone-base:latest
```

Or:

```bash
docker pull kjacone/kamikaze-drone-python:latest
```

---

## Apple Silicon Macs

If you are building on an Apple Silicon Mac (`arm64`), always specify:

```bash
--platform linux/amd64
```

This ensures the generated images can run on AMD64 Linux servers.

For example:

```bash
docker buildx build \
  --platform linux/amd64 \
  -f docker/Dockerfile.performance \
  -t performance-monitor:latest \
  --load \
  .
```

> **Note:** AMD64 builds on Apple Silicon may be slower because Docker needs to emulate the target architecture.
