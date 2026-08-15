"""Unit tests for _filter_text_for_tts. Zero rclpy imports."""
import pytest

from verter_admin.ai_assistant.ai_assistant_node import _filter_text_for_tts


def test_keeps_russian_and_punctuation():
    assert _filter_text_for_tts("Привет! Как дела?") == "Привет! Как дела?"


def test_keeps_english_and_digits():
    assert _filter_text_for_tts("Test 123, OK.") == "Test 123, OK."


def test_strips_special_chars():
    # @ # и подобные спецсимволы удаляются
    assert _filter_text_for_tts("Привет #123 @user") == "Привет 123 user"


def test_collapses_multiple_spaces():
    assert _filter_text_for_tts("Привет    мир") == "Привет мир"


def test_strips_leading_trailing_spaces():
    assert _filter_text_for_tts("  текст  ") == "текст"


def test_keeps_digits_and_parentheses():
    assert _filter_text_for_tts("Текст (1) - 2.") == "Текст (1) - 2."


def test_empty_string():
    assert _filter_text_for_tts("") == ""


def test_keeps_yo():
    assert _filter_text_for_tts("Всё, ещё ёлка") == "Всё, ещё ёлка"
