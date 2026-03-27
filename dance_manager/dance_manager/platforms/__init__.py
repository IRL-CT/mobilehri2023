"""
Platform implementations for the Robot Choreography System.

Each module provides a RobotPlatform subclass for a specific robot type.
The dance_server loads the appropriate platform based on a ROS parameter.

Available platforms:
    differential_drive  — two-wheeled robot (Twist-based velocity commands)
"""

from dance_manager.platforms.differential_drive import DiffDrivePlatform

__all__ = ["DiffDrivePlatform"]
