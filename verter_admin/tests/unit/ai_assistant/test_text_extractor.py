"""Unit tests for _extract_text_from_response. Zero rclpy imports."""
from types import SimpleNamespace

from verter_admin.ai_assistant.ai_assistant_node import _extract_text_from_response


class _RaisingOutputText:
    """response, где output_text бросает при доступе."""
    @property
    def output_text(self):
        raise Exception("no output_text")


def _resp_raising_output_text(output_items):
    resp = _RaisingOutputText()
    resp.output = output_items
    return resp


# --- Способ 1: output_text ---

def test_extract_via_output_text_normal():
    resp = SimpleNamespace(output_text="Привет", output=[])
    assert _extract_text_from_response(resp) == "Привет"


# --- Способ 2: output scan (когда способ 1 упал) ---

def test_extract_falls_back_to_output_scan_when_output_text_raises():
    content = SimpleNamespace(text="Ответ из scan")
    item = SimpleNamespace(content=[content])
    resp = _resp_raising_output_text([item])
    assert _extract_text_from_response(resp) == "Ответ из scan"


# --- output_text=None (без исключения) → TypeError → scan ---

def test_extract_output_text_none_falls_to_scan():
    empty_content = SimpleNamespace(text="")
    filled_content = SimpleNamespace(text="Реальный ответ")
    item = SimpleNamespace(content=[empty_content, filled_content])
    resp = SimpleNamespace(output_text=None, output=[item])
    assert _extract_text_from_response(resp) == "Реальный ответ"


# --- Оба способа упали / пусто → None ---

def test_extract_returns_none_when_both_fail():
    resp = SimpleNamespace(output_text=None, output=None)
    assert _extract_text_from_response(resp) is None


def test_extract_returns_none_when_no_content():
    item = SimpleNamespace()  # нет .content
    resp = _resp_raising_output_text([item])
    assert _extract_text_from_response(resp) is None


def test_extract_skips_empty_content_text():
    content = SimpleNamespace(text="")
    item = SimpleNamespace(content=[content])
    resp = _resp_raising_output_text([item])
    assert _extract_text_from_response(resp) is None
