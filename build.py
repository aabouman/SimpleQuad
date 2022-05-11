import argparse
import os
import string

def arduino_cli(scriptfile: str, fqbn: str, action="compile", verbose=False, port="tty/ACM0"):
    if action == "compile":
        cmd = "arduino-cli compile --warnings all --fqbn {} --libraries {} --library {} --build-path {} --build-cache-path {} {}".format(
            fqbn,
            arduino_libs_dir,
            core_lib_dir,
            bin_dir,
            cache_dir,
            scriptfile
        )
    elif action == "upload":
        cmd = "arduino-cli upload --fqbn {} --port {} --input-dir {} {}".format(
            fqbn,
            port,
            bin_dir,
            scriptfile
        )
    if verbose:
        print(cmd, "\n")
    os.system(cmd)


def build_lora_tx(action, **kwargs):
    dir = os.path.join(rootdir, "src", "Arduino", "lora_relay")
    scriptfile = os.path.join(dir, "lora_relay.ino")
    boardname = "adafruit:samd:adafruit_feather_m0"
    arduino_cli(scriptfile, boardname, action, **kwargs)

def build_lora_rx(action, **kwargs):
    dir = os.path.join(rootdir, "src", "Arduino", "lora_rx")
    scriptfile = os.path.join(dir, "lora_rx.ino")
    boardname = "adafruit:samd:adafruit_feather_m0"
    arduino_cli(scriptfile, boardname, action, **kwargs)

# Parse arguments
targets = [
    # "mocap",
    "lora_tx",
    "lora_rx",
]
parser = argparse.ArgumentParser()
parser.add_argument("target",
                    help="The target to build.",
                    type=str,
                    choices=targets
                    )
parser.add_argument("action",
                    help="Action to perform",
                    type=str,
                    choices=["compile", "upload"]
                    )
parser.add_argument("-v", "--verbose", action="store_true")
parser.add_argument("-p", "--port",
                    help="Port of the board, e.g. /dev/ttyACM0",
                    default="/dev/ttyACM0"
                    )
args = parser.parse_args()

# Set directories
rootdir = os.path.dirname(os.path.realpath(__file__))
arduino_libs_dir = os.path.join(rootdir, "src", "Arduino", "libraries")
core_lib_dir = os.path.join(rootdir, "src", "core")
bin_dir = os.path.join(rootdir, "bin", args.target)
cache_dir = os.path.join(bin_dir, "cache")

# Call the right build function
if args.target == "lora_tx":
    build_lora_tx(args.action, verbose=args.verbose, port=args.port)
elif args.target == "lora_rx":
    build_lora_rx(args.action, verbose=args.verbose, port=args.port)
    
