# TAM Stability Control

[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ros2-jazzy-blue.svg)](https://docs.ros.org/en/jazzy/index.html)
[![Paper](https://img.shields.io/badge/Paper-10.48550%2FarXiv.2608.17779-blue?logo=doi&logoColor=white)](https://doi.org/10.48550/arXiv.2608.17779)

This repository provides a vehicle stability-control system that can safeguard an arbitrary motion controller for real-world testing in autonomous racing. Its core is the `stability_control_tam_cpp` package, which combines wheel-slip estimation, Anti-lock Braking System (ABS), Traction Control (TC), Electronic Stability Control (ESC), and active countersteering.

The repository also contains a pure-pursuit tracking controller. The tracking controller is included as an example controller and to simplify integration. The tracking controller can be combined with the following longitudinal controller ([TAM Longitudinal Control](https://github.com/TUMFTM/TAM__long_acc_control.git)) to get a full motion controller that can be used with the stability-control system.

## Stability-Control System

The system computes wheel slip and tire slip angles from vehicle feedback, then applies the following control functions:

- **ABS** modulates brake pressure to prevent wheel lock during braking.
- **TC** limits drive-wheel slip during acceleration through brake intervention and throttle reduction.
- **ESC** applies differential front braking to track yaw-rate and sideslip references.
- **Countersteer** corrects the steering request when rear-axle sideslip indicates oversteer.

The ROS 2 stability-controller node combines these functions and publishes corrected lateral and longitudinal control commands.

<img src="packages/stability_control_tam_cpp/doc/system_architecture.svg" alt="Stability-control system architecture" width="650"/>

### Main Packages

| Package | Purpose |
| --- | --- |
| `stability_control_tam_cpp` | Stability-control algorithms, including slip calculation, ABS, TC, ESC, and countersteer. |
| `stability_controller_node_cpp` | ROS 2 node that connects vehicle feedback, controller requests, and the stability-control algorithms. |
| `tracking_controller_pure_pursuit` | Example pure-pursuit tracking controller used to support testing. |
| `tracking_controller_node_cpp` | ROS 2 wrapper for the example tracking controller. |

## Running the Code

The project was developed for ROS 2 Jazzy. Initialize all submodules before building:

```bash
git submodule update --init --recursive
```

Controller parameters are stored in [`config`](config). Parameters not provided in a YAML file use the defaults declared in the code. The `vehicle_handler_cpp` package also requires a vehicle configuration; example configurations are available under [`config/vehicle_handler`](config/vehicle_handler).

### Local

Build the stability controller and the included example tracking controller:

```bash
colcon build --packages-up-to stability_controller_node_cpp tracking_controller_node_cpp
```

Copy the desired vehicle configuration into the `vehicle_handler_cpp` configuration-overwrite directory. For the included dummy vehicle:

```bash
cp -r config/vehicle_handler/DummyVehicle/* install/vehicle_handler_cpp/share/vehicle_handler_cpp/config_overwrite/DummyVehicle/
```

Run the stability controller:

```bash
source install/setup.bash
ros2 run stability_controller_node_cpp stability_controller_node --ros-args --params-file config/stability_controller_config.yml
```

Run the example tracking controller when required for testing:

```bash
source install/setup.bash
ros2 run tracking_controller_node_cpp tracking_controller_node --ros-args --params-file config/tracking_controller_config.yml
```

### Docker

Build the Docker image:

```bash
docker build -f docker/Dockerfile -t stability_control:v1 .
```

Start the stability controller and the example tracking controller:

```bash
docker compose -f docker/compose-file.yml up
```

## Configuration

- [`config/stability_controller_config.yml`](config/stability_controller_config.yml) configures the stability-controller node and its ABS, TC, ESC, and countersteer functions.
- [`config/tracking_controller_config.yml`](config/tracking_controller_config.yml) configures the included example tracking controller.
- [`config/vehicle_handler`](config/vehicle_handler) contains vehicle and tire configurations used by the vehicle model.

## References

If you use the Stability Controller in your work please consider citing our paper:

[Stability Control for Real World Testing in Autonomous Racing](https://arxiv.org/abs/2608.17779)


```
@misc{pitschi2026stabilitycontrolrealworld,
      title={Stability Control for Real World Testing in Autonomous Racing}, 
      author={Phillip Pitschi and Simon Sagmeister and Frederik Werner and Markus Lienkamp and Boris Lohmann},
      year={2026},
      eprint={2608.17779},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2608.17779}, 
}
```

## Contact

- [Phillip Pitschi](mailto:phillip.pitschi@tum.de)

## Acknowledgements

We thank A2RL, the IAC and the TUM Autonomous Motorsports team for their support during the data acquisition and the development of the introduced stability controller.
