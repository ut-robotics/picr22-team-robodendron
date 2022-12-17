import websocket

import logging

class referee_command():
    def __init__(self):
        self.ws = websocket.WebSocket()
           

    def connect(self):
        self.ws.connect("ws://192.168.3.220:8111") 

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