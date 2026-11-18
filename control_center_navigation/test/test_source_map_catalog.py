"""Test source preference without opening Qt or starting ROS."""
import ast
from pathlib import Path

def resolver(source, installed):
    file=Path(__file__).resolve().parents[1]/'scripts/navigation_ui_node.py'
    tree=ast.parse(file.read_text(encoding='utf-8'))
    f=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_legacy_default_map_yaml')
    ns={'FilePath':Path,'_source_package_directory':lambda:source,
        'get_package_share_directory':lambda _:str(installed)}
    exec(compile(ast.Module(body=[f],type_ignores=[]),str(file),'exec'),ns)
    return ns['_legacy_default_map_yaml']()

def test_source_map_wins_over_stale_install(tmp_path):
    source=tmp_path/'src/autolife_robot_control_center/control_center_navigation'
    target=source.parent/'multi_lidar_slam/maps/yundie_2f.yaml'
    target.parent.mkdir(parents=True);target.write_text('image: yundie_2f.png')
    assert resolver(source,tmp_path/'install')==str(target.resolve())

def test_binary_only_deployment_keeps_install_fallback(tmp_path):
    source=tmp_path/'src/control_center_navigation';installed=tmp_path/'install/share/multi_lidar_slam'
    assert resolver(source,installed)==str(installed/'maps/yundie_2f.yaml')
