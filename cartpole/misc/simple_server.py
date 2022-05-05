import logging
from pathlib import Path

from aiohttp import web

from cartpole.sessions.collector import SessionData
from cartpole.common.util import find_free_port
import IPython.display as ipd
from threading import Thread
import dataclasses
import asyncio


class SimpleServer:
    FRONTEND_PATH = Path('web_view/frontend')

    def __init__(self, session_data: SessionData = None, port: int = 8080):
        self.port = port
        self.data: SessionData = session_data
        self.thread = Thread(target=self.run)

    async def index(self, _):
        return web.FileResponse(self.FRONTEND_PATH / 'index.html')

    async def ws_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        data = dataclasses.asdict(self.data)
        await ws.send_json(data)
        return ws

    def run(self):
        # loop = asyncio.get_event_loop()
        # if loop is None:
        #     loop = asyncio.new_event_loop()
        #     asyncio.set_event_loop(loop)
        app = web.Application()
        app.add_routes([
            web.get('/', self.index),
            web.get('/ws', self.ws_handler),
            web.static('/static', str(self.FRONTEND_PATH)),
        ])
        runner = web.AppRunner(app)
        bg_thread = Thread(target=self._bg_thread, args=(runner,))
        bg_thread.start()

        # web.run_app(app, host='0.0.0.0', port=self.port, access_log=None, print=False)

    def _bg_thread(self, runner):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, '0.0.0.0', self.port)
        loop.run_until_complete(site.start())
        loop.run_forever()


def session_widget(session_data: SessionData, height=None):
    server = getattr(session_widget, '__server_instance', None)
    if server is None:
        server = SimpleServer(session_data, port=find_free_port())
        setattr(session_widget, '__server_instance', server)
        server.run()
    src = f'http://localhost:{server.port}/?id=_'
    server.data = session_data
    if height is None:
        height = len(session_data.groups) * 300 + 100
    return ipd.IFrame(src=src, width='', height=height, extras=['style="width: 100%"'])
