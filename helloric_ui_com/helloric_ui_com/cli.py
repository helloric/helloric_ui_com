#!/usr/bin/env python3

# PYTHON_ARGCOMPLETE_OK

"""HelloRIC command line tool."""
import argparse
import asyncio
import logging

from .main import init_fastapi


LOG_LEVEL = {
    'critical': logging.CRITICAL,
    'error': logging.ERROR,
    'info': logging.INFO,
    'warn': logging.WARN,
    'debug': logging.DEBUG
}


def setup_logging(log_level):
    logging.basicConfig(level=LOG_LEVEL[log_level])
    return logging.getLogger(__name__)


def setup_argparse():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-l', '--log_level', type=str, default='info')
    parser.add_argument('--host', type=str, default='0.0.0.0')
    parser.add_argument('--port', type=int, default=7000)
    return parser

def setup():
    """parse data and create websocket and grpc endpoints."""
    args = setup_argparse().parse_args()
    logger = setup_logging(log_level=args.log_level)
    loop = asyncio.get_event_loop()
    return logger, loop, args


async def async_main(server, mgr):
    await asyncio.gather(
        server.serve(),
        mgr.main_loop())

def main():
    logger, loop, args = setup()
    mgr, server, _ = init_fastapi(
        logger=logger,
        loop=loop,
        host=args.host,
        port=int(args.port))
    loop.run_until_complete(async_main(server, mgr))
    logger.info('exiting')


if __name__ == '__main__':
    main()