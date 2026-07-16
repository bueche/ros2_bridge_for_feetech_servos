import math
import os
import tempfile
import pytest
import rclpy

def make_mock_response(ticks=2048, load=0, volt=120, temp=45, current=0):
    # Create a clean blank 21-byte frame response payload
    resp = bytearray([0x00] * 21)
     
    # Pack standard Feetech response frames matching exact indices
    resp[0] = 0xFF
    resp[1] = 0xFF
    resp[2] = 0x01 # Mock Servo ID
    resp[3] = 15   # Data Length
    resp[4] = 0x00 # Error status flag
     
    # Target Telemetry Registers matching bridge node index offsets
    resp[5] = (ticks >> 8) & 0xFF
    resp[6] = ticks & 0xFF
    resp[9] = (load >> 8) & 0xFF
    resp[10] = load & 0xFF
    resp[11] = volt & 0xFF
    resp[12] = temp & 0xFF
    resp[18] = (current >> 8) & 0xFF
    resp[19] = current & 0xFF
    return resp

def test_decoupled_alignment_matrix():
    # 1. Initialize the true ROS2 context for the scope of the test execution
    if not rclpy.ok():
        rclpy.init()
        
    from ros2_waveshare_bridge.ros2_waveshare_bridge import Ros2WaveshareBridge
    
    node = Ros2WaveshareBridge(init_serial=False)
    servo_id = 1 # Odd Index ID

    try:
        # (1) Test Temperature Alignment
        data = node.decode_telemetry_packet(make_mock_response(temp=50), servo_id)
        assert data['temperature'] == 50.0

        # (2) Test Current Alignment (preserving directional negative tracking)
        data_pos_curr = node.decode_telemetry_packet(make_mock_response(current=30), servo_id)
        assert data_pos_curr['current'] == 195 # 30 steps * 6.5mA
        
        data_neg_curr = node.decode_telemetry_packet(make_mock_response(current=65506), servo_id) # -30 steps
        assert data_neg_curr['current'] == -195

        # (3) Test Load Alignment (Mapping Feetech CW/CCW to Dynamixel Signed -1000/1000)
        data_ccw_load = node.decode_telemetry_packet(make_mock_response(load=200), servo_id)
        assert data_ccw_load['load'] == 200
        
        data_cw_load = node.decode_telemetry_packet(make_mock_response(load=1224), servo_id)
        assert data_cw_load['load'] == -200

        # (4) Test Voltage Alignment
        data_v = node.decode_telemetry_packet(make_mock_response(volt=116), servo_id)
        assert data_v['voltage'] == 116.0

        # (5) Test Position Alignment
        data_p_center = node.decode_telemetry_packet(make_mock_response(ticks=2048), servo_id)
        assert data_p_center['position'] == 0.0

        # (6) CRITICAL NEW TEST: Test Multi-turn Overflow Protection
        # Raw ticks = 6144 (which is exactly 1 full rotation of 4096 + 2048 center offset)
        # This must cleanly wrap back to exactly 0.0 radians
        data_p_overflow = node.decode_telemetry_packet(make_mock_response(ticks=6144), servo_id)
        assert data_p_overflow['position'] == 0.0

        # Test Negative Multi-turn wrap
        # Raw ticks = -2048 (should resolve to 2048 after modulo, resulting in 0.0 radians)
        # Since raw_ticks is read as unsigned 16-bit: -2048 maps to 63488
        data_p_neg_overflow = node.decode_telemetry_packet(make_mock_response(ticks=63488), servo_id)
        assert data_p_neg_overflow['position'] == 0.0

    finally:
        # Cleanly tear down the context pool
        node.destroy_node()
        rclpy.shutdown()

def test_v9_xml_parser():
    if not rclpy.ok():
        rclpy.init()
        
    from ros2_waveshare_bridge.ros2_waveshare_bridge import Ros2WaveshareBridge
    
    v9_sample_urdf = """<?xml version="1.0"?>
    <robot name="twin_servos">
      <joint name="shoulder_pan" type="revolute">
        <ros2_control name="FeetechServo">
          <param name="servo_id">1</param>
        </ros2_control>
      </joint>
    </robot>
    """
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
        tmp.write(v9_sample_urdf)
        tmp_path = tmp.name

    node = Ros2WaveshareBridge(init_serial=False)
    try:
        node.parse_urdf_file(tmp_path)
        assert "shoulder_pan" in node.joint_names
        assert node.joint_map["shoulder_pan"] == 1
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        node.destroy_node()
        rclpy.shutdown()
