import sys
import ssl
import socket
import logging
import argparse
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from functools import partial

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger(__name__)


class LoggingHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        logger.info("%s - %s", self.address_string(), format % args)

    def log_error(self, format, *args):
        logger.error("%s - %s", self.address_string(), format % args)


def parse_args():
    parser = argparse.ArgumentParser(description="Simple HTTPS file server")

    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Address to bind to (default: all interfaces)"
    )

    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="Port number"
    )

    parser.add_argument(
        "--directory",
        default=".",
        help="Directory to serve"
    )

    parser.add_argument(
        "--cert",
        default="cert.pem",
        help="TLS certificate file"
    )

    parser.add_argument(
        "--key",
        default="key.pem",
        help="TLS private key file"
    )

    return parser.parse_args()


def get_local_ip():
    """
    Returns the primary LAN IP address.
    Works across Wi-Fi, Ethernet, and hotspots.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # No traffic is actually sent
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()

    return ip


def print_connection_info(port):
    ip = get_local_ip()

    print("\n" + "=" * 60)
    print("HTTPS Server Started")
    print("=" * 60)
    print(f"Local machine: https://localhost:{port}")
    print(f"LAN access:    https://{ip}:{port}")
    print("\nOpen the LAN address from your phone if")
    print("it is connected to the same Wi-Fi network.")
    print("=" * 60 + "\n")


def main():
    args = parse_args()

    handler = partial(
        LoggingHandler,
        directory=args.directory
    )

    try:
        server = ThreadingHTTPServer(
            (args.host, args.port),
            handler
        )

        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)

        ctx.load_cert_chain(
            certfile=args.cert,
            keyfile=args.key
        )

        server.socket = ctx.wrap_socket(
            server.socket,
            server_side=True
        )

    except OSError as e:
        logger.error("Could not start server: %s", e)
        sys.exit(1)

    except ssl.SSLError as e:
        logger.error("SSL error — check cert/key paths: %s", e)
        sys.exit(1)

    logger.info(
        "Serving '%s' on port %d",
        args.directory,
        args.port
    )

    print_connection_info(args.port)

    try:
        server.serve_forever()

    except KeyboardInterrupt:
        logger.info("Shutting down...")
        server.shutdown()


if __name__ == "__main__":
    main()
