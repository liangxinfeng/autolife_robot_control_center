"""No robot nodes or motion. File operations only in pytest temp directories."""
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
import yaml
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
source = Path(__file__).resolve().parents[1] / 'scripts/navigation_ui_node.py'
spec = importlib.util.spec_from_file_location('map_editor_under_test', source)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


@pytest.fixture(scope='module')
def app():
    return m.QtWidgets.QApplication.instance() or m.QtWidgets.QApplication([])


def new_map(root, name='test'):
    grid = np.array([[-1, 0, 100], [100, -1, 0]], dtype=np.int16)
    meta = dict(resolution=.05, origin_x=-1., origin_y=2., origin_yaw=.2)
    ok, msg, path = m.save_map_files(root, name, grid, meta)
    assert ok, msg
    return Path(path), grid, meta


def read_grid(path):
    data = yaml.safe_load(path.read_text())
    image = m.QtGui.QImage(str(path.parent / data['image']))
    gray = np.array([[image.pixelColor(x,y).red() for x in range(image.width())]
                     for y in range(image.height())])
    occupancy = 1. - gray / 255.
    values = np.full(gray.shape, -1)
    values[occupancy > data['occupied_thresh']] = 100
    values[occupancy < data['free_thresh']] = 0
    return np.flipud(values)


def test_save_load_unknown_and_canvas_colors_preserved(tmp_path, app):
    path, grid, meta = new_map(tmp_path)
    restored = read_grid(path)
    assert np.array_equal(restored, grid)
    c = m.MapCanvas();c.grid = restored;c._render_grid()
    assert c.image.pixelColor(0, 1).getRgb()[:3] == (42,47,56)
    assert c.image.pixelColor(1, 1).getRgb()[:3] == (224,228,232)
    c.close()


@pytest.mark.parametrize('fmt',['pgm','png'])
def test_overwrite_backup_original_paths_geometry_unknown(tmp_path, fmt):
    path, grid, meta = new_map(tmp_path)
    data = yaml.safe_load(path.read_text())
    if fmt=='png':
        old = path.parent / data['image']
        dest = path.with_suffix('.png')
        assert m.QtGui.QImage(str(old)).save(str(dest), 'PNG')
        data['image']=dest.name;path.write_text(yaml.safe_dump(data))
    image = path.parent / data['image'];original=image.read_bytes();old_yaml=path.read_bytes()
    grid[0,1]=100
    backup = m.overwrite_map_files(path, grid, meta, [path])
    assert (backup / image.name).read_bytes()==original
    assert (backup / path.name).read_bytes()==old_yaml
    updated=yaml.safe_load(path.read_text())
    assert updated['image']==image.name and updated['origin']==data['origin']
    assert np.array_equal(grid,read_grid(path))


def test_overwrite_failure_rolls_back_both_files(tmp_path, monkeypatch):
    path, grid, meta=new_map(tmp_path);old=path.read_bytes();image=path.with_suffix('.pgm');old_image=image.read_bytes()
    original=Path.replace
    def fail_yaml(self,target):
        if str(self).endswith('.tmp') and Path(target)==path:raise OSError('injected')
        return original(self,target)
    monkeypatch.setattr(Path,'replace',fail_yaml)
    with pytest.raises(OSError):m.overwrite_map_files(path,grid+0,meta,[path])
    assert path.read_bytes()==old and image.read_bytes()==old_image


def test_wrong_geometry_rejected(tmp_path):
    path,grid,meta=new_map(tmp_path)
    with pytest.raises(ValueError,match='坐标不一致'):
        m.overwrite_map_files(path,grid,{**meta,'origin_x':9},[path])
    assert not (tmp_path/'.map_backups').exists()


def test_delete_recoverable_active_protected_waypoints_untouched(tmp_path):
    path,grid,meta=new_map(tmp_path);state=path.with_suffix('.pbstream');state.write_bytes(b'slam')
    wp=tmp_path/'waypoints/test/maps_index.json';wp.parent.mkdir(parents=True);wp.write_text('{}')
    with pytest.raises(ValueError,match='不能删除'):m.archive_map_files(path,[path],[path])
    trash=m.archive_map_files(path,[path],[])
    assert not path.exists() and not state.exists()
    assert (trash/path.name).exists() and (trash/state.name).read_bytes()==b'slam'
    assert (trash/'test.pgm').exists() and wp.read_text()=='{}'


def test_shared_image_not_deleted_or_overwritten(tmp_path):
    path,grid,meta=new_map(tmp_path);other=tmp_path/'alias.yaml';other.write_bytes(path.read_bytes())
    with pytest.raises(ValueError,match='共用'):m.overwrite_map_files(path,grid,meta,[path,other])
    m.archive_map_files(path,[path,other]);assert path.with_suffix('.pgm').exists()
    assert np.array_equal(read_grid(other),grid)


def test_delete_failure_restores_yaml(tmp_path,monkeypatch):
    path,grid,meta=new_map(tmp_path);original=Path.rename
    def fail_image(self,target):
        if self.suffix=='.pgm':raise OSError('injected')
        return original(self,target)
    monkeypatch.setattr(Path,'rename',fail_image)
    with pytest.raises(OSError):m.archive_map_files(path,[path])
    assert path.exists() and np.array_equal(read_grid(path),grid)


def test_uncatalogued_and_external_image_rejected(tmp_path):
    path,grid,meta=new_map(tmp_path)
    with pytest.raises(ValueError):m.archive_map_files(path,[])
    data=yaml.safe_load(path.read_text());data['image']='../external.pgm';path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError,match='同一目录'):m.overwrite_map_files(path,grid,meta,[path])


def test_ui_buttons_are_present_and_confirmed(app, monkeypatch, tmp_path):
    path,grid,meta=new_map(tmp_path);requests=[]
    node=NS(_loaded_map_path=str(path),_active_map_config=tmp_path/'active.yaml',
            _map_load_in_progress=False,map_catalog=lambda:[{'label':'test','path':str(path)}],
            refresh_waypoints=lambda:None,cancel_direct_goal=lambda:None)
    monkeypatch.setattr(m.NavigationWindow,'_connect_ros',lambda self:None)
    w=m.NavigationWindow(node);w.health_timer.stop()
    buttons={b.text():b for b in w.findChildren(m.QtWidgets.QPushButton)}
    assert {'保存当前地图','删除选中地图','另存新地图'}<=buttons.keys()
    w.map_canvas.grid=grid.copy();w.map_canvas.loaded_grid=grid.copy();w.map_canvas.grid[0,1]=100
    w.map_canvas.map_meta=meta
    monkeypatch.setattr(m.QtWidgets.QMessageBox,'question',lambda *a:requests.append(a) or m.QtWidgets.QMessageBox.No)
    buttons['保存当前地图'].click()
    assert requests and '保存当前地图' in requests[0]
    # No Yes: must not invoke persistence or send a load request.
    assert np.array_equal(read_grid(path),grid)
    w.map_canvas._render_grid()
    if os.environ.get('MAP_EDITOR_SCREENSHOT'):
        w.show();app.processEvents()
        assert w.grab().save(os.environ['MAP_EDITOR_SCREENSHOT'])
    w.hide();w.deleteLater()
