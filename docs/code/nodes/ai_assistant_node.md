# `ai_assistant_node`

## Задача

Берёт текст из `ai_question`, ищет релевантные куски в **dataset** (векторный поиск), и формирует ответ (в проде — через YandexGPT), который публикует в `text_to_speech`.

## Входы / выходы

- **Sub**: `ai_question` (String)
- **Sub**: `dialog_control` (String)
- **Pub**: `text_to_speech` (String)
- **Pub**: `play` (String) — если нужно озвучить “эффекты”

## Dataset (важное)

Код сначала пытается найти dataset через `get_package_share_directory('verter_admin')/dataset`, а если не удалось — берёт `dataset` рядом с файлом ноды (fallback).

Dataset как данные **не является частью MkDocs** и не должен переезжать в `docs/`.

## Режимы

В коде есть `self.is_testing = True`. В тестовом режиме SDK не инициализируется, а нода отвечает заглушкой (“Готов к работе в тестовом режиме”).


