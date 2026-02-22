import asyncio
import argparse
import cProfile

from modules.utils import set_config

# Parse command line arguments
parser = argparse.ArgumentParser(description='py_bt_ros')
parser.add_argument('--config', type=str, default='scenarios/example_simple/configs/config.yaml', help='Path to the configuration file (default: --config=scenarios/example_simple/configs/config.yaml)')
parser.add_argument('--ns', type=str, default=None, help='Override agent namespace, e.g. --ns /Fire_UGV_2')
args = parser.parse_args()

# Load configuration and initialize the environment
set_config(args.config)
from modules.utils import config
if args.ns is not None:
    config['agent']['namespaces'] = args.ns
from modules.bt_runner import BTRunner
bt_runner = BTRunner(config)


async def loop():
    while bt_runner.running:
        bt_runner.handle_keyboard_events()
        if not bt_runner.paused:
            await bt_runner.step()
        bt_runner.render()

    bt_runner.close()



if __name__ == "__main__":
    if config['bt_runner']['profiling_mode']:
        cProfile.run('main()', sort='cumulative')
    else:
        asyncio.run(loop())