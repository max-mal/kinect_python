from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading
import time
import base64
from kinect import Kinect
from config import Config

config: Config
hostName = "0.0.0.0"
serverPort = 56000
kinect = Kinect()
kinect.set_led(0)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""


class WebServer(BaseHTTPRequestHandler):

    def request_authorization(self):
        self.send_response(401)
        self.send_header(
            'WWW-Authenticate',
            'Basic realm=\"Authentication required\"'
        )
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def authenticate(self) -> bool:
        global config
        if not config.require_authentication:
            return True

        header = self.headers.get('Authorization', None)
        if not header:
            return False

        base64_message = header[6:]
        base64_bytes = base64_message.encode('ascii')
        message_bytes = base64.b64decode(base64_bytes)
        message = message_bytes.decode('ascii')

        parts = message.split(':')

        if parts[0] == config.username and parts[1] == config.password:
            return True

        return False

    def do_GET(self):
        if not self.authenticate():
            self.request_authorization()
            self.wfile.write(bytes("not authorized", "utf-8"))
            return

        if self.path == '/':
            self.route_root()
        elif self.path == '/jpeg':
            self.route_jpeg()
        elif self.path == '/depth_jpeg':
            self.route_jpeg(depth=True)
        elif self.path == '/mjpeg':
            self.route_mjpeg()
        elif self.path == '/depth_mjpeg':
            self.route_mjpeg(depth=True)
        else:
            self.route_404()

    def route_jpeg(self, depth=False):
        self.send_response(200)
        self.send_header("Content-type", "image/jpeg")
        self.end_headers()
        print("Getting image")
        data = kinect.get_jpeg(depth)
        print("Got image")
        self.wfile.write(data.getbuffer())

    def route_mjpeg(self, depth=False):
        global config
        self.send_response(20)
        self.send_header(
            'Content-type',
            'multipart/x-mixed-replace; boundary=--jpgboundary'
        )
        self.end_headers()

        while True:
            try:
                data = kinect.get_jpeg(depth)
                buffer = data.getbuffer()
                self.wfile.write("--jpgboundary\r\n".encode())
                self.send_header('Content-type', 'image/jpeg')
                # self.send_header('Content-length', str(buffer.nbytes))
                self.end_headers()
                self.wfile.write(buffer)
                time.sleep(config.mjpeg_frame_delay)
            except KeyboardInterrupt:
                break
            except BrokenPipeError:
                break
        return

    def route_root(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes("<html><head><title>Kinect web server</title></head>", "utf-8"))
        self.wfile.write(bytes("<p>Request: %s</p>" % self.path, "utf-8"))
        self.wfile.write(bytes("<body>", "utf-8"))
        self.wfile.write(bytes("<p><a href='/jpeg'>Jpeg<a></p>", "utf-8"))
        self.wfile.write(bytes("<p><a href='/depth_jpeg'>Depth Jpeg<a></p>", "utf-8"))
        self.wfile.write(bytes("<p><a href='/mjpeg'>MJpeg<a></p>", "utf-8"))
        self.wfile.write(bytes("<p><a href='/depth_mjpeg'>Depth MJpeg<a></p>", "utf-8"))
        self.wfile.write(bytes("</body></html>", "utf-8"))

    def route_404(self):
        self.send_response(404)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes("Not found", "utf-8"))
        self.wfile.write(bytes("<p>Request: %s</p>" % self.path, "utf-8"))

    @staticmethod
    def start(_config: Config):
        global config
        config = _config
        server = ThreadedHTTPServer((config.server_addr, config.server_port), WebServer)
        print('Starting server, use <Ctrl-C> to stop')
        server.serve_forever()
