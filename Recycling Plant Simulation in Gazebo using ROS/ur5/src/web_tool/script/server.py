#!/usr/bin/python3

import os
import functools
import http.server, ssl

dir_path = __file__.replace("/server.py", "")

certfile = os.path.join(dir_path, "certfile.pem")
directory = os.path.join(dir_path, "site")

server_address = ('0.0.0.0', 55000)

print("")
print("[SERVER] dir_path:", dir_path)
print("[SERVER] certfile:", certfile)
print("[SERVER] server_address:", server_address)

os.chdir(directory)

httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)
httpd.socket = ssl.wrap_socket(httpd.socket,
                               server_side=True,
                               certfile=certfile,
                               ssl_version=ssl.PROTOCOL_TLSv1_2)
httpd.serve_forever()

# openssl req -new -x509 -keyout server.pem -out server.pem -days 365 -nodes