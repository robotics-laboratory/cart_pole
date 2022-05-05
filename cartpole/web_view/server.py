import json
import logging
import threading
import weakref
import asyncio

from aiohttp import web

from cartpole.sessions.collector import CollectorProxy
from cartpole.common.util import init_logging


LOGGER = logging.getLogger(__name__)


class Server:
    FRONTEND_FOLDER = 'web_view/frontend'

    def __init__(self, collector: CollectorProxy) -> None:
        self.collector = collector

    async def index_handler(self, _: web.Request) -> web.Response:
        return web.FileResponse(f'{self.FRONTEND_FOLDER}/index.html')

    async def ws_handler(self, request: web.Request) -> web.Response:
        session_id = request.query.get('session_id')

        if session_id is not None:
            try:
                with open(f'{session_id}') as file:
                    session = json.load(file)
            except FileNotFoundError as e:
                LOGGER.error(f'Cannot open session file: {e}')
                raise web.HTTPUnprocessableEntity()

        ws = web.WebSocketResponse()
        await ws.prepare(request)
        request.app['websockets'].add(ws)
        try:
            if session_id is None:
                while True:
                    value, ok = self.collector.consume_value()
                    for key in list(value.keys()):
                        if key not in ['id', 'x', 'y']:
                            value.pop(key)
                    if not ok:
                        await ws.close()
                    await ws.send_json(value)
            else:
                await ws.send_json(session)
        finally:
            request.app['websockets'].discard(ws)
            
        return ws

def _run_server(collector: CollectorProxy):
    server = Server(collector)

    app = web.Application()
    app['websockets'] = weakref.WeakSet()
    app.add_routes([
        web.get('/', server.index_handler),
        web.static('/static', Server.FRONTEND_FOLDER),
        web.get('/ws', server.ws_handler),
    ])

    async def on_shutdown(app):
        for ws in set(app['websockets']):
            await ws.close(code=web.WSCloseCode.GOING_AWAY, message='Server shutdown')

    # app.on_shutdown.append(on_shutdown)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    web.run_app(app)


def run_server(collector: CollectorProxy = None):
    init_logging()
    t = threading.Thread(target=_run_server, args=(collector,), daemon=True)
    t.start()
    LOGGER.info(f'Starting server on thread {t.ident}')
