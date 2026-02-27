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


def merge_dicts(dict1, dict2):
    # 두 개의 딕셔너리를 복사하여 합칠 딕셔너리를 초기화합니다.
    merged_dict = dict1.copy()
    
    # dict2의 항목을 순회합니다.
    for key, value in dict2.items():
        # 이미 merged_dict에 같은 키가 있으면 값을 비교하여 최대 값을 설정합니다.
        if key in merged_dict:
            merged_dict[key] = max(merged_dict[key], value)
        else:
            # 새로운 키일 경우 추가합니다.
            merged_dict[key] = value
            
    return merged_dict    

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