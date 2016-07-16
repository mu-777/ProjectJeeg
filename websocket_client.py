#!/usr/bin/env python
# -*- coding: utf-8 -*-

# http://mktktmr.hatenablog.jp/entry/2016/03/15/141217

import websocket
import json


class WebSocketServer:
    def __init__(self, urt):
        self._url = url
        self._ws = websocket.WebSocket


class WebSocketClient:
    def __init__(self, url):
        self._url = url
        websocket.enableTrace(True)
        self._ws = websocket.WebSocketApp(url,
                                          on_open=self._on_open,
                                          on_message=self._on_message,
                                          on_error=self._on_error,
                                          on_close=self._on_close)

    def run(self):
        self._ws.run_forever()
        return self

    def _on_message(self, ws, message):
        print("debug: called _on_message")
        data = json.loads(message)
        print(data["angle1"], data["angle2"], data["angle3"], data["angle4"])

    def _on_error(self, ws, error):
        print("debug: called on_error")
        print(error)


    def _on_close(self, ws):
        print("### closed ###")


    def _on_open(self, ws):
        print("open")


# --------------------------------------------
if __name__ == "__main__":
    url = "ws://localhost:3000"

    ws_client = WebSocketClient(url)
    ws_client.run()