"""Подставляет заглушки rclpy/std_msgs/ament в sys.modules до импорта модуля.

ai_assistant_node.py импортирует rclpy на верхнем уровне. В окружении без ROS
toolchain (как в CI) rclpy отсутствует. Здесь модуль импортируется с моками,
чтобы тестировать чистую логику (_ErrorClassifier, _extract_text_from_response,
_format_search_results, _filter_text_for_tts) без ROS.

httpx и openai — pure-Python зависимости; ставятся реально
(`pip install httpx openai`), нужны для isinstance-проверок в _ErrorClassifier.
"""
import sys
import types


def _install_stub(name, attrs=None):
    """Создаёт модуль-заглушку с атрибутами и регистрирует в sys.modules."""
    module = types.ModuleType(name)
    if attrs:
        for key, value in attrs.items():
            setattr(module, key, value)
    sys.modules.setdefault(name, module)
    return module


# rclpy.node.Node — базовый класс; подставляем тривиальный object-наследник.
class _StubNode:
    def __init__(self, *args, **kwargs):
        pass


_install_stub('rclpy', {
    'init': lambda *a, **k: None,
    'spin': lambda *a, **k: None,
    'shutdown': lambda *a, **k: None,
})
_install_stub('rclpy.node', {'Node': _StubNode})
_install_stub('std_msgs')
_install_stub('std_msgs.msg', {'String': object})


def _get_package_share_directory(*args, **kwargs):
    raise Exception('share directory not available in unit tests')


_install_stub('ament_index_python')
_install_stub('ament_index_python.packages', {
    'get_package_share_directory': _get_package_share_directory,
})
