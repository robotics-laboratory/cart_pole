import json
import logging
import threading
import weakref

from aiohttp import web

from web_view.collector import Collector


LOGGER = logging.getLogger(__name__)


class Server:
    def __init__(self, collector: Collector) -> None:
        self.collector = collector

    async def index_handler(self, _: web.Request) -> web.Response:
        return web.FileResponse('frontend/index.html')

    async def ws_handler(self, request: web.Request) -> web.Response:
        session_id = request.query.get('session_id')
        group = request.query.get('group')
        field = request.query.get('field')

        if session_id is None:
            try:
                with open(f'{Collector.STORAGE_DIR}/{session_id}') as file:
                    session = json.load(file)
            except FileNotFoundError as e:
                LOGGER.error(f'Cannot open session file: {e}')
                raise web.HTTPUnprocessableEntity()

        if group is None:
            LOGGER.error(f'No group in request parameters')
            raise web.HTTPBadRequest()

        if field is None:
            LOGGER.error(f'No field in request parameters')
            raise web.HTTPBadRequest()

        ws = web.WebSocketResponse()
        await ws.prepare(request)
        request.app['websockets'].add(ws)
        try:
            if session_id is None:
                index = 0
                while self.collector.is_running:
                    batch = self.collector.consume(group, field, index)
                    await ws.send_json(batch)
                    index += len(batch)
            else:
                await ws.send_json(session)
        finally:
            request.app['websockets'].discard(ws)
            
        return ws

def _run_server(collector: Collector):
    if collector is None:
        collector = Collector()
        created_collector = True
    else:
        created_collector = False
    
    server = Server(collector)

    app = web.Application()
    app['websockets'] = weakref.WeakSet()
    app.add_routes([
        web.get('/', server.index_handler),
        web.static('/static', 'frontend'),
        web.get('/ws', server.ws_handler),
    ])

    async def on_shutdown(app):
        for ws in set(app['websockets']):
            await ws.close(code=web.WSCloseCode.GOING_AWAY, message='Server shutdown')

    app.on_shutdown.append(on_shutdown)

    try:
        web.run_app(app)
    except:
        if created_collector:
            collector.stop()
        raise
        


def run_server(collector: Collector = None):
    t = threading.Thread(target=_run_server, args=(collector,), daemon=True)
    LOGGER.info(f'Starting server on thread {t.ident}')
    t.start()
    

if __name__ == '__main__':
    _run_server(None)
