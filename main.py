import asyncio
import argparse
import cProfile
import signal

from modules.utils import set_config

# Parse command line arguments
parser = argparse.ArgumentParser(description='py_bt_ros')
parser.add_argument('--config', type=str, default='scenarios/example_simple/configs/grape.yaml', help='Path to the configuration file (default: --config=scenarios/example_simple/configs/config.yaml)')
parser.add_argument('--ns', type=str, default='/Fire_UGV_3', help='Override agent namespace, e.g. --ns /Fire_UGV_2')
args = parser.parse_args()

# Load configuration and initialize the environment
set_config(args.config)
from modules.utils import config
if args.ns is not None:
    config['agent']['namespaces'] = args.ns
from modules.bt_runner import BTRunner
bt_runner = BTRunner(config)


async def loop():
    # SIGTERM → running=False → loop 종료 → finally에서 close() 호출
    asyncio.get_event_loop().add_signal_handler(
        signal.SIGTERM, lambda: setattr(bt_runner, 'running', False)
    )
    try:
        while bt_runner.running:
            bt_runner.handle_keyboard_events()
            if not bt_runner.paused:
                await bt_runner.step()
            bt_runner.render()
    finally:
        bt_runner.close()  # halt_tree() → cancel active ROS Action goals


if __name__ == "__main__":
    if config['bt_runner']['profiling_mode']:
        cProfile.run('main()', sort='cumulative')
    else:
        try:
            asyncio.run(loop())
        except KeyboardInterrupt:
            pass  # bt_runner.close()는 loop() finally에서 이미 호출됨