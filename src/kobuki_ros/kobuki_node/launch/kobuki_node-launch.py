import os

import ament_index_python.packages
import launch
import launch_ros.actions
import tkinter as tk
import threading
import subprocess
import yaml
from playsound import playsound

def mp3_player_thread(mp3_file):
    try:
        playsound(mp3_file)
    except Exception as e:
        print(f"Error playing MP3: {e}")

def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_node')
    # There are two different ways to pass parameters to a non-composed node;
    # either by specifying the path to the file containing the parameters, or by
    # passing a dictionary containing the key -> value pairs of the parameters.
    # When starting a *composed* node on the other hand, only the dictionary
    # style is supported.  To keep the code between the non-composed and
    # composed launch file similar, we use that style here as well.
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    mp3_file = "UC3000.mp3"

    mp3_thread = threading.Thread(target=mp3_player_thread, args=(mp3_file,))
    mp3_thread.start()


    kobuki_ros_node = launch_ros.actions.Node(package='kobuki_node',
                                              executable='kobuki_ros_node',
                                              output='both',
                                              parameters=[params])

    return launch.LaunchDescription([kobuki_ros_node])
