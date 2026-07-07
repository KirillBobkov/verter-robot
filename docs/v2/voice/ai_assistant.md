# AI-ассистент

Реализован в `ai_assistant/ai_assistant_node.py`. Интеграция с YandexGPT через Responses API с векторным поиском по базе знаний медицинского центра.

## Переменные окружения

| Переменная | Обязательно | По умолчанию | Назначение |
|------------|-------------|--------------|------------|
| `YANDEX_CLOUD_FOLDER` | да | — | ID каталога Yandex Cloud |
| `YANDEX_CLOUD_API_KEY` | да | — | API-ключ |
| `YANDEX_CLOUD_MODEL` | нет | `aliceai-llm` | Выбор модели |

При отсутствии `FOLDER`/`API_KEY` нода падает с `RuntimeError` при инициализации.

## System instruction

Заложена в константу `INSTRUCTION`. Роль — информационный робот-ассистент на входе эндокринологического центра. Правила: не даёт медицинских рекомендаций, не подтверждает записи, обязательно использует `file_search`, отвечает кратко, числа словами.

## Контекст диалога

Поддержка двух независимых контекстов через `previous_response_id`:

| Режим | Флаг | ID |
|-------|------|----|
| Основной | `dialog_active=False` | `main_previous_id` |
| Диалог | `dialog_active=True` | `dialog_previous_id` |

Управление — топик `dialog_control`:

- `start_dialog` → сброс `dialog_previous_id`, `dialog_active=True`.
- `end_dialog` → сброс `dialog_previous_id`, `dialog_active=False`.

## Векторный поиск

| Параметр | Значение |
|----------|----------|
| Имя vector store | `verter-medical-index` |
| Срок жизни | 10 дней (`expires_after`) |
| Max results | 10 |
| Timeout создания | 900 с |
| Polling | 2 с |
| Датасет | `share/verter_admin/dataset/chunks.jsonl` |
| Кэш ID | `vector_store_id.txt` |

`tool_choice=required` — модель обязана использовать `file_search`.

## Параметры запроса

| Параметр | Значение |
|----------|----------|
| Модель | `gpt://{folder_id}/{model_name}` |
| Temperature | 0.3 |
| Max tokens | 350 |

## Обработка ошибок

Любое исключение логируется с traceback (`logger.exception`), публикуется fallback-ответ «Сервис временно недоступен.» в `text_to_speech`. Retry отсутствует — один запрос, при ошибке сразу fallback.

!!! warning "Vector store expiry"
    `expires_after=10 дней` требует периодического пересоздания. Нет автоматического мониторинга — при истечении поиск перестанет работать.
