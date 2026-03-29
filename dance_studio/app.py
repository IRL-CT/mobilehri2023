"""Dance Studio — Web-based choreography design tool.

Run with:
    # Source ROS first for execute/live features:
    source /opt/ros/humble/setup.bash
    source ~/dancerobot_ws/install/setup.bash
    cd ~/dancerobot_ws/src/mobilehri2023/dance_studio
    python app.py

Then open http://localhost:8000 in your browser.
ROS features (execute, live position) are optional — the tool works without ROS
for AI generation, simulation preview, and save/load.
"""

import asyncio
import dataclasses
import json
import math
import os
import sys
import threading

# Add dance_manager to Python path (no ROS imports needed for choreographer)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "dance_manager"))

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse
from pydantic import BaseModel
from typing import Optional

from simulator import StubPlatform, simulate, DISPLACEMENTS, CATEGORIES, PLATFORM_DESCRIPTION
from storage import list_dances, load_dance, save_dance

app = FastAPI(title="Dance Studio")

# Stub platform for AI choreographer
stub_platform = StubPlatform()

# ROS bridge (lazy-initialized)
_ros_bridge = None


# ── ROS Bridge ───────────────────────────────────────────────────────────────

class ROSBridge:
    """Manages ROS connection for executing dances and reading odometry."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node

        rclpy.init()
        self._node = rclpy.create_node('dance_studio_bridge')
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._executing = False
        self._ws_clients = set()

        # Odom subscriber
        from nav_msgs.msg import Odometry
        self._node.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Action client for dance
        from rclpy.action import ActionClient
        from dance_interfaces.action import Dance
        self._action_client = ActionClient(self._node, Dance, 'dance')

        # Spin ROS in background thread
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self._node.get_logger().info('Dance Studio ROS bridge started')

    def _spin(self):
        import rclpy
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def _odom_cb(self, msg):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._odom_theta = 2.0 * math.atan2(qz, qw)

    def get_position(self):
        return {
            "x": round(self._odom_x, 4),
            "y": round(self._odom_y, 4),
            "theta": round(self._odom_theta, 4),
        }

    @property
    def is_executing(self):
        return self._executing

    def execute_sequence(self, sequence_dict, energy_scale=1.0):
        """Execute a dance sequence by sending goals one by one."""
        if self._executing:
            return False

        self._executing = True
        thread = threading.Thread(
            target=self._execute_thread,
            args=(sequence_dict, energy_scale),
            daemon=True,
        )
        thread.start()
        return True

    def _execute_thread(self, sequence_dict, energy_scale):
        import json as _json
        from dance_interfaces.action import Dance
        import rclpy
        import time

        try:
            if not self._action_client.wait_for_server(timeout_sec=3.0):
                self._node.get_logger().error('Dance action server not available')
                return

            for phrase in sequence_dict.get("phrases", []):
                for motif in phrase.get("motifs", []):
                    if not self._executing:
                        return  # cancelled
                    move = motif.get("move", "")
                    energy = min(2.0, max(0.05, motif.get("energy", 0.5) * energy_scale))
                    texture = motif.get("texture", "neutral")
                    params = motif.get("params", {})
                    gap = motif.get("gap_before", 0.0)

                    if gap > 0:
                        time.sleep(gap)

                    goal = Dance.Goal()
                    goal.dance_move = move
                    goal.energy = float(energy)
                    goal.texture = str(texture)
                    goal.params = _json.dumps(params) if params else ""

                    self._node.get_logger().info(f'Executing: {move} e={energy:.1f}')
                    future = self._action_client.send_goal_async(goal)

                    # Wait for acceptance
                    while not future.done():
                        time.sleep(0.05)
                    goal_handle = future.result()
                    if not goal_handle.accepted:
                        continue

                    # Wait for result
                    result_future = goal_handle.get_result_async()
                    while not result_future.done():
                        time.sleep(0.05)

                gap_after = phrase.get("gap_after", 0.0)
                if gap_after > 0:
                    time.sleep(gap_after)

        finally:
            self._executing = False

    def stop(self):
        self._executing = False

    def reset_robot(self):
        """Reset robot to origin by publishing to Gazebo set_pose service."""
        import subprocess
        try:
            # Use gz service to set pose back to origin
            subprocess.run([
                'ign', 'service', '-s', '/world/dance_stage/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000',
                '--req',
                'name: "chair_robot", position: {x: 0, y: 0, z: 0.01}, '
                'orientation: {x: 0, y: 0, z: 0, w: 1}'
            ], timeout=3, capture_output=True)
            self._node.get_logger().info('Robot reset to origin')
        except Exception as e:
            self._node.get_logger().warn(f'Reset failed: {e}')


def get_ros_bridge():
    """Lazy-init ROS bridge. Returns None if ROS is not available."""
    global _ros_bridge
    if _ros_bridge is not None:
        return _ros_bridge
    try:
        _ros_bridge = ROSBridge()
        return _ros_bridge
    except Exception as e:
        print(f"ROS not available: {e}")
        return None


# ── Pydantic models ──────────────────────────────────────────────────────────

class GenerateRequest(BaseModel):
    description: str
    stage_width: float = 6.0
    stage_depth: float = 4.0

class SimulateRequest(BaseModel):
    sequence: dict
    energy_scale: float = 1.0
    tempo_bpm: Optional[float] = None
    stage_width: float = 6.0
    stage_depth: float = 4.0

class ExecuteRequest(BaseModel):
    sequence: dict
    energy_scale: float = 1.0


# ── API routes ───────────────────────────────────────────────────────────────

@app.get("/api/moves")
def get_moves():
    return {
        "moves": list(DISPLACEMENTS.keys()),
        "categories": CATEGORIES,
        "displacements": DISPLACEMENTS,
        "platform_description": PLATFORM_DESCRIPTION,
    }


@app.post("/api/prompt")
def get_prompt(req: GenerateRequest):
    """Return the full system prompt that would be sent to the AI."""
    from dance_manager.choreographer import AIChoreographer
    try:
        ai = AIChoreographer(
            api_key=os.environ.get("GOOGLE_API_KEY", "dummy"),
            platform=stub_platform,
            stage_width=req.stage_width, stage_depth=req.stage_depth,
        )
        system = ai.SYSTEM_PROMPT_TEMPLATE.format(
            platform_description=ai._get_platform_description(),
            move_list=ai._get_move_list_str(),
            texture_options=ai.TEXTURE_OPTIONS,
            modifier_schema=ai.MODIFIER_SCHEMA,
            stage_constraints=ai._get_stage_constraints_str(),
        )
        return {"system_prompt": system, "user_prompt": req.description}
    except Exception as e:
        # Fallback: build prompt without the full class
        return {
            "system_prompt": f"(Could not build full prompt: {e})",
            "user_prompt": req.description,
        }


@app.post("/api/generate")
def generate_dance(req: GenerateRequest):
    api_key = os.environ.get("GOOGLE_API_KEY", "")
    if not api_key:
        raise HTTPException(status_code=500, detail="GOOGLE_API_KEY not set")
    try:
        from dance_manager.choreographer import AIChoreographer
        ai = AIChoreographer(
            api_key=api_key, platform=stub_platform,
            stage_width=req.stage_width, stage_depth=req.stage_depth,
        )
        sequence = ai.generate(req.description)
        return dataclasses.asdict(sequence)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/simulate")
def simulate_sequence(req: SimulateRequest):
    try:
        return simulate(req.sequence, energy_scale=req.energy_scale, tempo_bpm=req.tempo_bpm)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/execute")
def execute_dance(req: ExecuteRequest):
    """Execute a dance sequence on the real robot via ROS."""
    bridge = get_ros_bridge()
    if bridge is None:
        raise HTTPException(status_code=503, detail="ROS not available. Source ROS and restart.")
    if bridge.is_executing:
        raise HTTPException(status_code=409, detail="Already executing a dance")
    bridge.execute_sequence(req.sequence, energy_scale=req.energy_scale)
    return {"status": "executing"}


@app.post("/api/stop")
def stop_dance():
    """Stop the currently executing dance."""
    bridge = get_ros_bridge()
    if bridge:
        bridge.stop()
    return {"status": "stopped"}


@app.post("/api/reset")
def reset_robot():
    """Reset robot to starting position in Gazebo."""
    bridge = get_ros_bridge()
    if bridge is None:
        raise HTTPException(status_code=503, detail="ROS not available")
    bridge.reset_robot()
    return {"status": "reset"}


@app.get("/api/ros_status")
def ros_status():
    """Check if ROS is connected."""
    bridge = get_ros_bridge()
    if bridge is None:
        return {"connected": False, "executing": False}
    return {"connected": True, "executing": bridge.is_executing}


@app.get("/api/dances")
def get_dances():
    return {"dances": list_dances()}


@app.get("/api/dances/{name}")
def get_dance(name: str):
    try:
        return load_dance(name)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail=f"Dance '{name}' not found")


@app.post("/api/dances/{name}")
def post_dance(name: str, sequence: dict):
    save_dance(name, sequence)
    return {"status": "saved", "name": name}


# ── WebSocket: live robot position ───────────────────────────────────────────

@app.websocket("/ws/position")
async def ws_position(websocket: WebSocket):
    """Stream live robot position from ROS odometry at ~20Hz."""
    await websocket.accept()
    bridge = get_ros_bridge()
    if bridge is None:
        await websocket.send_json({"error": "ROS not available"})
        await websocket.close()
        return

    try:
        while True:
            pos = bridge.get_position()
            pos["executing"] = bridge.is_executing
            await websocket.send_json(pos)
            await asyncio.sleep(0.05)  # 20 Hz
    except WebSocketDisconnect:
        pass


# ── Static files ─────────────────────────────────────────────────────────────

@app.get("/")
def index():
    return RedirectResponse(url="/static/index.html")

app.mount("/static", StaticFiles(directory=os.path.join(os.path.dirname(__file__), "static")), name="static")


# ── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import uvicorn
    print("Dance Studio starting at http://localhost:8000")
    print("For ROS features: source ROS setup before running")
    uvicorn.run(app, host="0.0.0.0", port=8000)
