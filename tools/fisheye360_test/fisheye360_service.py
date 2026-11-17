"""345's independent two-camera calibration viewer. No robot control routes."""
import argparse
import asyncio
import json
from pathlib import Path

from aiohttp import web
from fisheye_camera import FisheyeCamera, FisheyeEndpoint
from fisheye_profile import DEFAULT_PROFILE_PATH, load_profile, save_profile

WEB = Path(__file__).resolve().parent / 'web'
NO_CACHE = {'Cache-Control': 'no-store'}
ASSETS = {'/': 'index.html', '/index.html': 'index.html', '/app.js': 'app.js',
          '/projection.js': 'projection.js', '/projection_renderer.js': 'projection_renderer.js',
          '/jpeg_stream.js': 'jpeg_stream.js'}


def build_app(front_factory=None, rear_factory=None, profile_path=DEFAULT_PROFILE_PATH):
    defaults = json.loads((WEB/'profile.json').read_text(encoding='utf-8'))
    cameras = {
        'front': FisheyeCamera(
            module_name='mod_camera_head_right', image_key='head_right', label='前',
            consumer_name='vr345_f360_v3_front', consumer_factory=front_factory),
        'rear': FisheyeCamera(
            module_name='mod_camera_head_rear', image_key='head_rear', label='后',
            consumer_name='vr345_f360_v3_rear', consumer_factory=rear_factory),
    }
    endpoints = {name: FisheyeEndpoint(cam) for name, cam in cameras.items()}
    app = web.Application(client_max_size=8*1024)

    async def asset(request):
        return web.FileResponse(WEB/ASSETS[request.path], headers=NO_CACHE)

    async def config(_request):
        saved, source = await asyncio.to_thread(load_profile, defaults, profile_path)
        return web.json_response({
            'app': 'fisheye360-test-345', 'version': '345-surround-sync-1',
            'camera_only': True, 'width': 640, 'height': 360, 'profile': defaults,
            'vr_profile': saved if source == 'synced' else None, 'calibration_source': source,
            **{name: {'websocket_path': f'/ws/{name}', 'status_url': f'/api/{name}/status',
                      'producer_status': cam.status()} for name, cam in cameras.items()},
        }, headers=NO_CACHE)

    async def sync_profile(request):
        # The calibration button is same-origin; reject cross-site form posts.
        if request.headers.get('Origin') != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden(text='仅允许从鱼眼测试页面同步。')
        if request.content_type != 'application/json':
            raise web.HTTPUnsupportedMediaType()
        try:
            payload = await request.json()
            profile = await asyncio.to_thread(save_profile, payload, defaults, profile_path)
        except (ValueError, TypeError):
            return web.json_response({'error': '标定参数不完整或超出范围。'}, status=400, headers=NO_CACHE)
        except OSError:
            return web.json_response({'error': '无法保存到机器人，请检查配置目录。'}, status=500, headers=NO_CACHE)
        return web.json_response({'profile':profile, 'calibration_source':'synced'}, headers=NO_CACHE)

    async def shutdown(_app):
        await asyncio.gather(*(ep.close() for ep in endpoints.values()), return_exceptions=True)

    async def cleanup(_app):
        await asyncio.gather(*(asyncio.to_thread(cam.close) for cam in cameras.values()))

    for route in ASSETS:
        app.router.add_get(route, asset)
    app.router.add_get('/api/config', config)
    app.router.add_post('/api/vr-profile', sync_profile)
    for name, endpoint in endpoints.items():
        app.router.add_get(f'/api/{name}/status', endpoint.status)
        app.router.add_get(f'/ws/{name}', endpoint.websocket)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(cleanup)
    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8452)
    args = parser.parse_args()
    web.run_app(build_app(), host=args.host, port=args.port, shutdown_timeout=3)
