# Hackathon Project: Observability in Robotics and IoT with OpenTelemetry

We’ll explore how to instrument code and visualize telemetry data and logs using OpenTelemetry and the Grafana open-source stack.

## Project Ideas

### ROS2 Telemetry
- Add OpenTelemetry metrics to a basic ROS2 service tutorial.
- Create a dashboard to show the telemetry in Grafana.

### Sensor Monitoring
- Read a sensor (temperature/humidity, ...) from your laptop and instrument it as a metric or log event with OpenTelemetry.
- Create a Grafana dashboard to view the data.

### Bring Your Own!
- Add metrics, traces, or logs to an existing C++ (or Python) project and create a dashboard.

## Requirements
- Bring a laptop capable of running a Linux-based Docker container and VSCode with the devcontainer extension.
- Pre-install Docker and ensure docker-compose is working.
- A docker-compose file that sets up Grafana, Loki, Mimir, Tempo, and the OpenTelemetry Collector will be provided so you can quickly instrument and visualize your data.

## What to Expect
- Hands-on experience with opentelemetry metrics, logs, and traces.
- Collaborative coding to learn how observability helps diagnose and optimize robotics/IoT systems.
- Quick demos where teams share their dashboards and share what you've developed.

## Quick Start
- Clone this repository

    `git clone https://github.com/dbarker/cmu-africa-hackathon.git`

    `cd cmu-africa-hackathon`

- Open VSCode  

    `code .`

- Open the DevContainer

    `Cntr+shift+P`

    `Dev Containers: Reopen in Container`

- Wait for the container to build (requires internet access)
- Open a termainal and build the example 

    `colcon build`

- Run the example 

    `source install/setup.bash`

    `ros2 run opentelemetry_ros2_example_cpp simple_process_cpp`

- View the logs, metrics, and traces in Grafana
   - [Explore Logs](http://localhost:3000/explore?schemaVersion=1&panes=%7B%22gov%22:%7B%22datasource%22:%22loki%22,%22queries%22:%5B%7B%22refId%22:%22A%22,%22expr%22:%22%7Bservice_name%3D%5C%22simple_process%5C%22%7D%20%7C%3D%20%60%60%22,%22queryType%22:%22range%22,%22datasource%22:%7B%22type%22:%22loki%22,%22uid%22:%22loki%22%7D,%22editorMode%22:%22builder%22,%22direction%22:%22backward%22%7D%5D,%22range%22:%7B%22from%22:%22now-15m%22,%22to%22:%22now%22%7D%7D%7D&orgId=1)
   - [Explore Traces](http://localhost:3000/explore?schemaVersion=1&panes=%7B%22gov%22:%7B%22datasource%22:%22tempo%22,%22queries%22:%5B%7B%22refId%22:%22A%22,%22datasource%22:%7B%22type%22:%22tempo%22,%22uid%22:%22tempo%22%7D,%22queryType%22:%22traceqlSearch%22,%22limit%22:20,%22tableType%22:%22traces%22,%22filters%22:%5B%7B%22id%22:%22b4966122%22,%22operator%22:%22%3D%22,%22scope%22:%22span%22%7D,%7B%22id%22:%22service-name%22,%22tag%22:%22service.name%22,%22operator%22:%22%3D%22,%22scope%22:%22resource%22,%22value%22:%5B%22simple_process%22%5D,%22valueType%22:%22string%22%7D%5D%7D%5D,%22range%22:%7B%22from%22:%22now-15m%22,%22to%22:%22now%22%7D%7D%7D&orgId=1)
   - [Explore Metrics](http://localhost:3000/explore?schemaVersion=1&panes=%7B%22gov%22:%7B%22datasource%22:%22mimir%22,%22queries%22:%5B%7B%22refId%22:%22A%22,%22expr%22:%22%7Bservice_name%3D%5C%22simple_process%5C%22%7D%22,%22range%22:true,%22datasource%22:%7B%22type%22:%22prometheus%22,%22uid%22:%22mimir%22%7D%7D%5D,%22range%22:%7B%22from%22:%22now-15m%22,%22to%22:%22now%22%7D%7D%7D&orgId=1)

## Helpful References
- [OpenTelemetry documentation](https://opentelemetry.io/docs/what-is-opentelemetry/)
- [Grafana documentation](https://grafana.com/docs/grafana/latest/)
- [Windows WSL install](https://learn.microsoft.com/en-us/windows/wsl/install)
- [Docker install](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
- [VSCode devcontainer extension install](https://code.visualstudio.com/docs/devcontainers/tutorial#_install-the-extension)
