import websocket

import logging

class referee_command():
    def __init__(self):
        self.ws = websocket.WebSocket()
           

    def connect(self):
        self.ws.connect("ws://172.17.157.128:8222") 

    def listen(self):
        msg = self.ws.recv()
        print(msg)
        return msg

if __name__ == "__main__":
    rf_command = referee_command()
    rf_command.connect()
    try:
        while True:
            rf_command.listen()

    except KeyboardInterrupt:
        rf_command.ws.close()