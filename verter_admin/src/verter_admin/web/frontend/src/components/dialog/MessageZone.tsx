/**
 * MessageZone — одно текущее сообщение в облачке (+ опциональное доп. сообщение
 * ниже, для thinking: «ВЫ СПРОСИЛИ» + вопрос, под ним «Думаю…»).
 *
 * Облачко различает AI↔пользователя (НЕ меняем фон страницы — решение команды):
 *  - фон/хвостик/подпись облачка зависят от роли;
 *  - подпись капсом «РОБОТ ОТВЕЧАЕТ» / «ВЫ СПРОСИЛИ» / «ОШИБКА» / «ГОворите…».
 *
 * Адаптация длинного текста: размер шрифта авто-масштабируется под высоту stage.
 * Полный текст всегда в DOM (доступность для скринридеров).
 *
 * Плавная смена по роли (cross-fade): при смене MessageRole старый блок плавно
 * гаснет (opacity → 0), новый — плавно появляется (fadeDown). Смена текста при
 * той же роли — мгновенно, без анимации. Достигается двумя слоями в DOM:
 * `displayed` (видимый) и `outgoing` (гаснущий, удаляется после transition).
 */

import { useEffect, useLayoutEffect, useRef, useState } from "react";
import type { DialogMessage, MessageRole } from "../../types/dialog";
import styles from "./MessageZone.module.css";

interface MessageZoneProps {
  message: DialogMessage | null;
  /** Доп. сообщение под основным (thinking: «Думаю…» под вопросом). */
  subMessage?: DialogMessage | null;
}

const ROLE_LABEL: Partial<Record<MessageRole, string>> = {
  question: "ВЫ СПРОСИЛИ",
  answer: "РОБОТ",
  error: "РОБОТ",
  welcome: "РОБОТ",
  farewell: "РОБОТ",
  listening: "РОБОТ",
  speaking: "РОБОТ",
  // thinking — без подписи: текст «Думаю…» сам по себе.
};

/** Длительность перехода opacity (мс) — должна совпадать с CSS var --transition-slow. */
const FADE_MS = 1050;

/** Задержка появления каждого слова эффекта печатания (мс). */
const WORD_DELAY_MS = 300;

/** Минимальный размер шрифта при авто-масштабировании (px) — потолок читаемости. */
const MIN_FONT_PX = 12;

/** Рендер текста по словам: каждое слово — span с задержкой появления (эффект печатания). */
function TypingText({ text }: { text: string }) {
  // split с захватом разделителей сохраняет пробелы между словами.
  const tokens = text.split(/(\s+)/);
  let wordIndex = -1;
  return (
    <>
      {tokens.map((token, i) => {
        if (/^\s+$/.test(token)) {
          return <span key={i}>{token}</span>;
        }
        wordIndex += 1;
        return (
          <span
            key={i}
            className={styles.word}
            style={{ animationDelay: `${wordIndex * WORD_DELAY_MS}ms` }}
          >
            {token}
          </span>
        );
      })}
    </>
  );
}

/** Ключ для React: меняется при любом изменении текста/роли. */
function msgKey(m: DialogMessage | null): string {
  return m ? `${m.role}::${m.text}` : "empty";
}

interface Layer {
  message: DialogMessage;
  subMessage?: DialogMessage | null;
}

export default function MessageZone({ message, subMessage }: MessageZoneProps) {
  const zoneRef = useRef<HTMLDivElement>(null);
  const bubbleRef = useRef<HTMLDivElement>(null);
  const messageRef = useRef<HTMLParagraphElement>(null);

  // Базовый font-size (px). Им масштабируем при переполнении.
  const [fontSize, setFontSize] = useState<number | null>(null);

  // Видимый слой и уходящий (гаснущий). outgoing === null — анимации нет.
  const [displayed, setDisplayed] = useState<Layer | null>(
    message ? { message, subMessage } : null,
  );
  const [outgoing, setOutgoing] = useState<Layer | null>(null);

  const fadeTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const prevRole = useRef<MessageRole | null>(message?.role ?? null);

  // Очистка таймера при размонтировании.
  useEffect(() => {
    return () => {
      if (fadeTimer.current) clearTimeout(fadeTimer.current);
    };
  }, []);

  // Реакция на смену входящего сообщения.
  useEffect(() => {
    const nextRole = message?.role ?? null;
    const roleChanged = nextRole !== prevRole.current;
    prevRole.current = nextRole;

    if (!message) {
      // Сообщение убрали — гасим текущий, ничего не показываем.
      if (displayed) {
        setOutgoing(displayed);
        setDisplayed(null);
        if (fadeTimer.current) clearTimeout(fadeTimer.current);
        fadeTimer.current = setTimeout(() => setOutgoing(null), FADE_MS + 30);
      }
      return;
    }

    if (!roleChanged) {
      // Та же роль (возможно другой текст) — обновляем in-place, без fade.
      setDisplayed({ message, subMessage });
      return;
    }

    // Сменилась роль — старый слой уходит, новый входит.
    const prev = displayed;
    setDisplayed({ message, subMessage });
    if (prev) {
      setOutgoing(prev);
      if (fadeTimer.current) clearTimeout(fadeTimer.current);
      fadeTimer.current = setTimeout(() => setOutgoing(null), FADE_MS + 30);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [message?.role, message?.text, subMessage?.role, subMessage?.text]);

  // Авто-масштабирование шрифта под доступную высоту (для видимого слоя).
  // Гибрид: базовый размер уже подобран CSS (container query cqh + --dialog-font-size).
  // JS — только подстраховка при реальном переполнении по объёму текста: один
  // замер, одна формула (ratio), без итеративного цикла reflow.
  useLayoutEffect(() => {
    const zone = zoneRef.current;
    const bubble = bubbleRef.current;
    const msg = messageRef.current;
    if (!zone || !bubble || !msg || !displayed) {
      setFontSize(null);
      return;
    }

    const available = zone.clientHeight;
    if (available <= 0) return;

    // Базовый размер — тот, что CSS уже вычислил для .message (container query
    // cqh + clamp по --dialog-font-size). Берём computed, а не переменную, чтобы
    // уменьшать от реально применённого размера.
    const base = parseFloat(getComputedStyle(msg).fontSize);
    if (!base || Number.isNaN(base)) {
      setFontSize(null);
      return;
    }

    // Сбрасываем инлайн-переопределение, чтобы замерить чистый CSS-размер.
    msg.style.fontSize = "";
    const naturalHeight = bubble.scrollHeight;

    const overflow = naturalHeight - available;
    if (overflow <= 4) {
      // Текст уместился — отдаём приоритет CSS-размеру, инлайн не нужен.
      setFontSize(null);
      return;
    }

    // Одна формула вместо цикла: scale = доступно / нужно.
    const size = Math.max(MIN_FONT_PX, base * (available / naturalHeight));
    setFontSize(size);
  }, [displayed]);

  useEffect(() => {
    zoneRef.current?.scrollTo({ top: 0 });
  }, [displayed]);

  if (!displayed && !outgoing) {
    return <div className={styles.zone} ref={zoneRef} />;
  }

  return (
    <div className={styles.zone} ref={zoneRef}>
      {outgoing && (
        <div className={styles.layer} aria-hidden="true">
          <Bubble key={msgKey(outgoing.message)} layer={outgoing} fading="out" />
        </div>
      )}
      {displayed && (
        <div className={styles.layer}>
          <Bubble
            ref={bubbleRef}
            messageRef={messageRef}
            key={msgKey(displayed.message)}
            layer={displayed}
            fading="in"
            fontSize={fontSize}
          />
        </div>
      )}
    </div>
  );
}

interface BubbleProps {
  layer: Layer;
  fading: "in" | "out";
  fontSize?: number | null;
  ref?: React.Ref<HTMLDivElement>;
  messageRef?: React.Ref<HTMLParagraphElement>;
}

function Bubble({ layer, fading, fontSize, ref, messageRef }: BubbleProps) {
  const { message, subMessage } = layer;
  const label = ROLE_LABEL[message.role];
  const isAssertive = message.role === "error";
  const isOutgoing = fading === "out";

  return (
    <div
      ref={ref}
      className={styles.bubble}
      data-role={message.role}
      data-fading={fading}
    >
      {label && <div className={styles.label}>{label}</div>}
      <p
        ref={messageRef}
        className={styles.message}
        data-role={message.role}
        aria-live={isOutgoing ? undefined : isAssertive ? "assertive" : "polite"}
        style={fontSize != null ? { fontSize: `${fontSize}px` } : undefined}
      >
        <TypingText text={message.text} />
      </p>
      {subMessage && (
        <p
          className={styles.subMessage}
          data-role={subMessage.role}
          aria-live={isOutgoing ? undefined : "polite"}
        >
          {subMessage.text}
        </p>
      )}
    </div>
  );
}
