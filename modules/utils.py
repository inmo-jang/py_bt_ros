import yaml
import os
import xml.etree.ElementTree as ET
import importlib

def load_config(config_file):
    with open(config_file, 'r', encoding="utf-8") as f:
        return yaml.safe_load(f)

# Global variable to hold the configuration
config = None

def set_config(config_file):
    global config
    config = load_config(config_file)
    config['config_file_path'] = config_file

def get_file_dirname(file):
    return os.path.dirname(os.path.abspath(file))  # 모듈 파일 기준

# BT xml
def parse_behavior_tree(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    return root
    

def convert_value(v): # "None" → None; 문자열 숫자는 숫자로 변환
    if v == "None":
        return None
    if isinstance(v, str):
        if v.isdigit() or (v.startswith('-') and v[1:].isdigit()):
            return int(v)
        try:
            return float(v)
        except ValueError:
            pass
    return v


class AttrDict(dict):
    """dict 키를 속성처럼 접근할 수 있는 래퍼.
    task.position == task['position'] == task.get('position')
    """
    def __getattr__(self, key):
        try:
            return self[key]
        except KeyError:
            raise AttributeError(key)


def optional_import(name):
    if not name:
        return None
    try:
        return importlib.import_module(name)
    except ModuleNotFoundError as e:
        # 요청한 모듈 자체가 없을 때만 None 반환
        if e.name == name:
            return None
        # 내부 의존 모듈 누락 등은 그대로 올림
        raise