import os
import socket
from collections import defaultdict
from yamcs.client import YamcsClient
from util_images import create_background_image, create_depth_image, save_image, publish_image_metadata

# Configuration constants
YAMCS_HOST = "localhost:8090"
YAMCS_INSTANCE = "husky"
YAMCS_PROCESSOR = "realtime"
ROVER_IMG_DIR = "/tmp/images_oncommand"
DEPTH_IMG_DIR = "/tmp/images_depth"
WIDTH = 1440
HEIGHT = 1080
FONT_SIZE = HEIGHT // 15
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BACKGROUND_IMG = os.path.join(SCRIPT_DIR, 'NavCam_high.png')

# defined in yamcs-server/src/main/yamcs/etc/yamcs.husky.yaml for the datalink with the "tc_realtime" stream
TC_RECEIVE_ADDRESS = '127.0.0.1'
TC_RECEIVE_PORT    = 10025


def detect_command(data):
    for cmd_name in [
        b'camera_capture_high',
        b'camera_streaming',
        b'camera_capture_depth',
        b'lander_camera_capture',
    ]:
        if data.startswith(cmd_name):
            return cmd_name, len(cmd_name)
    return None, 0


def handle_command(processor, command_counts, cmd_name, cmd_name_len, data):
    if cmd_name == b'camera_capture_high' and len(data) == cmd_name_len:
        print(f'Received: camera_capture_high')
        command_counts["rover_high"] += 1
        seq = command_counts["rover_high"]
        text = f"High Resolution OnCommand Image {seq:04d}"
        image_name = f"image_oncommand_{seq:04d}.png"
        save_image(create_background_image(BACKGROUND_IMG, WIDTH, HEIGHT), text, FONT_SIZE, WIDTH, HEIGHT, ROVER_IMG_DIR, image_name)
        publish_image_metadata(processor, YAMCS_HOST, "/Rover/camera/images_oncommand", "images_oncommand", seq, image_name)

    elif cmd_name == b'camera_streaming':
        if len(data) == cmd_name_len + 1:
            action_byte = data[cmd_name_len]
            action = "START" if action_byte == 1 else "STOP"
            print(f'Received: camera_streaming {action}')
        else:
            print(f'Error: camera_streaming command has {len(data)} bytes, expected {cmd_name_len + 1}')

    elif cmd_name == b'camera_capture_depth' and len(data) == cmd_name_len:
        print(f'Received: camera_capture_depth')
        command_counts["rover_depth"] += 1
        seq = command_counts["rover_depth"]
        text = f"Depth OnCommand Image {seq:04d}"
        image_name = f"image_depth_{seq:04d}.png"
        save_image(create_depth_image(WIDTH, HEIGHT), text, FONT_SIZE, WIDTH, HEIGHT, DEPTH_IMG_DIR, image_name, fill=(0, 0, 0, 255))
        publish_image_metadata(processor, YAMCS_HOST, "/Rover/camera/images_depth", "images_depth", seq, image_name)

    else:
        print(f"Unknown command: {len(data)} bytes, hex: {data.hex()}")


def main():
    client = YamcsClient(YAMCS_HOST)
    processor = client.get_processor(instance=YAMCS_INSTANCE, processor=YAMCS_PROCESSOR)

    for d in (ROVER_IMG_DIR, DEPTH_IMG_DIR):
        os.makedirs(d, exist_ok=True)

    tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tc_socket.bind((TC_RECEIVE_ADDRESS, TC_RECEIVE_PORT))

    command_counts = defaultdict(int)

    print(f"Listening for commands on UDP {TC_RECEIVE_ADDRESS}:{TC_RECEIVE_PORT}...")
    while True:
        data, _ = tc_socket.recvfrom(4096)
        cmd_name, cmd_name_len = detect_command(data)
        handle_command(processor, command_counts, cmd_name, cmd_name_len, data)


if __name__ == "__main__":
    main()
