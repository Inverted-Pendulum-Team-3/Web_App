Web App

This web application provides a centralized interface for monitoring and operating the Inverted Pendulum Robot remotely over Wi-Fi. It includes an about page with a project overview, toggle controls for starting and stopping all major robot processes, a manual control mode with directional input, an autonomous navigation mode, and a live data dashboard displaying raw sensor readings alongside real-time graphs for IMU tilt and motor encoder velocity.

Auto Nav

Autonomous wall-avoidance navigation layer running at ~20 Hz on top of the PID balance controller. Reads ultrasonic sensor cache and writes forward/turn velocity targets to a shared command file. The balance loop runs independently and is never interrupted by navigation commands.
