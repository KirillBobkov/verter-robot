"""Unit tests for error classification helpers. Zero rclpy imports."""
import httpx

from verter_admin.ai_assistant.ai_assistant_node import (
    is_network_error,
    ERROR_MESSAGES,
)


# --- is_network_error ---

def test_network_error_unreachable():
    assert is_network_error(Exception("Network is unreachable")) is True


def test_network_error_connect_error_string():
    assert is_network_error(Exception("ConnectError: ...")) is True


def test_non_network_error():
    assert is_network_error(Exception("timeout")) is False


def test_non_network_error_empty():
    assert is_network_error(Exception("")) is False


# --- ERROR_MESSAGES (тексты + fallback) ---

def test_error_messages_contain_all_types():
    assert "слишком долго" in ERROR_MESSAGES['timeout']
    assert "интернетом" in ERROR_MESSAGES['network']
    assert "не отвечает" in ERROR_MESSAGES['unavailable']


def test_error_messages_fallback_to_unavailable():
    assert ERROR_MESSAGES.get('nonexistent', ERROR_MESSAGES['unavailable']) == ERROR_MESSAGES['unavailable']
