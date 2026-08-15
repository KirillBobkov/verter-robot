/**
 * RobotAvatar — 2D-SVG-риг робота Verter.
 *
 * Один постоянный SVG, состояние задаётся атрибутом data-avatar на корневом
 * <svg>: idle | listening | thinking | speaking | sad (error → sad).
 *
 * Риг разделён на независимые группы (#base #body #arm-left #arm-right #neck
 * #head → #face #eyes #mouth), каждая анимируемая часть — на своей группе со
 * своим transform-origin, чтобы трансформации не складывались.
 * Анимации и keyframes — в RobotAvatar.module.css (чистый CSS, без GSAP).
 *
 * UX по статусам (голова статична во всех состояниях):
 *  - idle      — дыхание, моргание (круглые глаза+зрачок, раз в 2с), правая рука машет.
 *  - listening — звуковые волны идут справа налево к голове и тают у неё.
 *  - thinking  — глаза-кольца крутятся как спиннер загрузки, точки «•••».
 *  - speaking  — рот открывается/закрывается, лёгкое покачивание корпуса.
 *  - sad       — грустные дуги глаз, пожимание плечами (обе руки), наклон корпуса.
 */

import type { DialogStatus } from "../../types/dialog";
import styles from "./RobotAvatar.module.css";

interface RobotAvatarProps {
  status: DialogStatus;
}

export default function RobotAvatar({ status }: RobotAvatarProps) {
  // sad — только в самом статусе error. В speaking/thinking ошибка уже не
  // «прилипает»: робот озвучивает ошибку как обычную речь (говорит).
  const avatarState = status === "error" ? "sad" : status;

  return (
    <svg
      className={styles.avatar}
      data-avatar={avatarState}
      viewBox="0 0 500 680"
      role="img"
      aria-label={`Робот: ${avatarState}`}
    >
      
      <defs>
        {/* Белый градиент корпуса (верх светлее — объём). */}
        <linearGradient id="bodyWhite" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#FFFFFF" />
          <stop offset="1" stopColor="#E9EBEC" />
        </linearGradient>
        {/* Белый градиент головы. */}
        <linearGradient id="headWhite" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#FFFFFF" />
          <stop offset="1" stopColor="#E2E4E5" />
        </linearGradient>
        {/* Лаймовый градиент экрана на груди. */}
        <linearGradient id="limeScreen" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#D6F04A" />
          <stop offset="1" stopColor="#A8D400" />
        </linearGradient>
        {/* Мягкие тени. */}
        <filter id="shadow" x="-20%" y="-20%" width="140%" height="140%">
          <feDropShadow
            dx="0"
            dy="3"
            stdDeviation="4"
            floodColor="#000"
            floodOpacity="0.15"
          />
        </filter>
        <filter id="softShadow" x="-20%" y="-20%" width="140%" height="140%">
          <feDropShadow
            dx="0"
            dy="2"
            stdDeviation="3"
            floodColor="#000"
            floodOpacity="0.10"
          />
        </filter>
      </defs>

      {/* ======================================================= */}
      {/* BASE — основание, колёса, тень (самый задний слой) */}
      {/* ======================================================= */}
      <g id="base">
        <ellipse
          cx="250"
          cy="655"
          rx="155"
          ry="22"
          fill="#000000"
          opacity="0.12"
        />
        <path
          d="M120 585 Q125 555 155 545 L345 545 Q375 555 380 585 L395 630 Q398 648 375 650 L125 650 Q102 648 105 630 Z"
          fill="url(#bodyWhite)"
          stroke="#D6D9DA"
          strokeWidth="3"
          filter="url(#shadow)"
        />
        <path
          d="M120 585 L145 560 L160 560 L145 635 L115 635 Z"
          fill="#C8F020"
        />
        <path
          d="M380 585 L355 560 L340 560 L355 635 L385 635 Z"
          fill="#C8F020"
        />
      </g>

      {/* ======================================================= */}
      {/* SOUND WAVES — звуковые дуги справа от головы, идут к голове.
          Видны только в listening. Не вложены в #head: голова смещается
          вправо-вниз НАВСТРЕЧУ волнам (источник звука справа). */}
      {/* ======================================================= */}
      <g className={styles.soundWaves}>
        <path className={styles.soundWave} d="M460 70 A 30 30 0 0 1 460 122" />
        <path className={styles.soundWave} d="M475 54 A 52 52 0 0 1 475 138" />
        <path className={styles.soundWave} d="M490 36 A 76 76 0 0 1 490 156" />
      </g>
      {/* ======================================================= */}
      {/* BODY — корпус + грудной экран + индикаторы + логотип */}
      {/* ======================================================= */}
      <g id="body" className={styles.body} filter="url(#softShadow)">
        <path
          d="M130 275 Q130 250 155 240 Q250 215 345 240 Q370 250 370 275 L380 545 Q380 570 355 580 Q250 600 145 580 Q120 570 120 545 Z"
          fill="url(#bodyWhite)"
          stroke="#D7DADB"
          strokeWidth="3"
        />
        {/* Жёлтые боковые элементы корпуса. */}
        <path
          d="M135 315 Q120 325 118 350 L118 420 Q118 440 130 445 Q142 440 145 420 L150 335 Q150 318 135 315 Z"
          fill="#C8F020"
        />
        <path
          d="M365 315 Q380 325 382 350 L382 420 Q382 440 370 445 Q358 440 355 420 L350 335 Q350 318 365 315 Z"
          fill="#C8F020"
        />

        {/* CHEST — чёрная панель + лаймовый экран + строки + QR. */}
        <g id="chest">
          <rect
            x="115"
            y="290"
            width="270"
            height="125"
            rx="34"
            fill="#151515"
          />
          <rect
            id="chest-screen"
            x="130"
            y="305"
            width="240"
            height="92"
            rx="25"
            fill="url(#limeScreen)"
          />
          <g
            id="screen-lines"
            stroke="#151515"
            strokeWidth="5"
            strokeLinecap="round"
          >
            <line x1="155" y1="335" x2="225" y2="335" />
            <line x1="155" y1="355" x2="215" y2="355" />
            <line x1="155" y1="375" x2="235" y2="375" />
          </g>
          <g id="qr">
            <rect
              x="300"
              y="320"
              width="50"
              height="50"
              rx="5"
              fill="#FFFFFF"
            />
            <rect x="307" y="327" width="12" height="12" fill="#111" />
            <rect x="331" y="327" width="12" height="12" fill="#111" />
            <rect x="307" y="351" width="12" height="12" fill="#111" />
            <rect x="324" y="344" width="7" height="7" fill="#111" />
            <rect x="335" y="350" width="7" height="7" fill="#111" />
            <rect x="322" y="357" width="7" height="7" fill="#111" />
          </g>
        </g>

        {/* INDICATORS — пульсируют в idle/thinking. */}
        <g id="indicators">
          <rect
            x="155"
            y="430"
            width="190"
            height="25"
            rx="12"
            fill="#D9DCDD"
          />
          <circle
            className={styles.indicatorDot}
            cx="245"
            cy="442"
            r="5"
            fill="#FFFFFF"
          />
          <circle
            className={styles.indicatorDot}
            cx="265"
            cy="442"
            r="5"
            fill="#FFFFFF"
          />
        </g>

        {/* LOGO — фирменный знак, центрирован на нижней части корпуса. */}
        <svg
          id="logo"
          x="200"
          y="475"
          width="100"
          height="100"
          viewBox="0 0 640 640"
          preserveAspectRatio="xMidYMid meet"
        >
          <g transform="translate(0,640) scale(0.1,-0.1)" stroke="none">
            <path
              fill="var(--accent)"
              d="M1423 5879 c-81 -10 -213 -56 -296 -102 -136 -77 -247 -209 -304 -364 l-28 -78 -3 -1415 c-3 -1582 -6 -1501 68 -1655 86 -179 257 -310 485 -371 63 -17 111 -19 462 -19 215 0 395 4 400 8 4 5 9 908 11 2008 l3 1999 -368 -1 c-202 -1 -396 -5 -430 -10z"
            />
            <path
              fill="var(--accent)"
              d="M4145 5878 c-3 -7 -4 -911 -3 -2008 l3 -1995 390 1 c344 0 398 2 459 18 274 72 453 226 540 465 l31 86 3 1407 c3 1574 7 1483 -69 1638 -34 69 -59 102 -131 173 -75 75 -103 95 -183 133 -52 25 -138 56 -191 70 -94 23 -107 24 -471 24 -290 0 -375 -3 -378 -12z"
            />
            <path d="M2227 1845 c-7 -20 -8 -954 -1 -1157 l7 -208 942 0 942 0 6 242 c4 133 7 442 7 687 l0 446 -950 0 c-624 0 -951 -3 -953 -10z" />
          </g>
        </svg>
      </g>

      {/* ======================================================= */}
      {/* NECK — шея (между корпусом и головой) */}
      {/* ======================================================= */}
      <g id="neck">
        <path d="M220 235 L225 195 L275 195 L280 235 Z" fill="#1A1A1A" />
        <ellipse cx="250" cy="225" rx="35" ry="12" fill="#0E0E0E" />
      </g>

      {/* ======================================================= */}
      {/* HEAD — голова + лицо + глаза + рот */}
      {/* ======================================================= */}
      <g id="head" className={styles.head} filter="url(#shadow)">
        <rect
          x="70"
          y="10"
          width="360"
          height="190"
          rx="75"
          fill="url(#headWhite)"
          stroke="#D5D8D9"
          strokeWidth="4"
        />
        <rect
          id="face"
          x="92"
          y="35"
          width="316"
          height="135"
          rx="55"
          fill="#151515"
        />

        {/* EYES — круглые глаза (лаймовый круг + белый зрачок), моргают.
            В thinking заменяются спиннер-кольцами (см. eyeSpinners). */}
        <g id="eyes">
          <g id="eye-left" className={styles.eyeLeft}>
            <circle cx="190" cy="83" r="35" fill="#C8F020" />
            <circle
              className={styles.pupil}
              cx="190"
              cy="83"
              r="27"
              fill="#FFFFFF"
            />
          </g>
          <g id="eye-right" className={styles.eyeRight}>
            <circle cx="310" cy="83" r="35" fill="#C8F020" />
            <circle
              className={styles.pupil}
              cx="310"
              cy="83"
              r="27"
              fill="#FFFFFF"
            />
          </g>
          {/* Спиннер-кольца — скрыты по умолчанию, видны только в thinking. */}
          <g className={styles.eyeSpinners}>
            <circle
              className={styles.eyeSpinner}
              cx="190"
              cy="83"
              r="30"
              fill="none"
              stroke="#C8F020"
              strokeWidth="9"
              strokeLinecap="round"
              strokeDasharray="141 189"
            />
            <circle
              className={styles.eyeSpinner}
              cx="310"
              cy="83"
              r="30"
              fill="none"
              stroke="#C8F020"
              strokeWidth="9"
              strokeLinecap="round"
              strokeDasharray="141 189"
            />
            <circle
              className={styles.pupil}
              cx="190"
              cy="83"
              r="27"
              fill="#FFFFFF"
            />
            <circle
              className={styles.pupil}
              cx="310"
              cy="83"
              r="27"
              fill="#FFFFFF"
            />
          </g>
          {/* Sad-дуги (вогнуты вниз) — скрыты по умолчанию, видны в sad. */}
          <g className={styles.eyesSad}>
            <path className={styles.eyeSad} d="M150 90 Q190 124 230 90" />
            <path className={styles.eyeSad} d="M270 90 Q310 124 350 90" />
          </g>
        </g>

        {/* MOUTH — нейтральная улыбка (видна в idle/listening/thinking),
            speaking-овал (виден в speaking), sad-дуга (видна в sad). */}
        <path className={styles.mouthSmile} d="M225 138 Q250 150 275 138" />
        <g id="mouth" className={styles.mouth}>
          <ellipse cx="250" cy="138" rx="34" ry="14" fill="#C8F020" />
        </g>
        <path className={styles.mouthSad} d="M225 146 Q250 130 275 146" />
      </g>



      {/* ======================================================= */}
      {/* ARMS — руки (поверх головы: поднятая кисть видна) */}
      {/* ======================================================= */}
      <g id="arm-left" className={styles.armLeft}>
        <path
          d="M100 280 Q75 300 70 340 L50 450 Q47 470 62 480 Q77 485 85 465 L115 355 Q130 310 120 290 Z"
          fill="url(#bodyWhite)"
          stroke="#D5D8D9"
          strokeWidth="3"
        />
        <path
          d="M80 315 Q68 320 65 340 L55 380 Q52 395 62 400 Q72 402 77 387 L90 340 Q95 320 80 315 Z"
          fill="#C8F020"
        />
        <path
          d="M50 445 Q42 460 50 475 Q60 490 75 480 Q88 470 82 455 Q70 440 50 445 Z"
          fill="#E7E9EA"
        />
      </g>

      <g id="arm-right" className={styles.armRight}>
        <path
          d="M400 280 Q425 300 430 340 L450 450 Q453 470 438 480 Q423 485 415 465 L385 355 Q370 310 380 290 Z"
          fill="url(#bodyWhite)"
          stroke="#D5D8D9"
          strokeWidth="3"
        />
        <path
          d="M420 315 Q432 320 435 340 L445 380 Q448 395 438 400 Q428 402 423 387 L410 340 Q405 320 420 315 Z"
          fill="#C8F020"
        />
        <path
          d="M450 445 Q458 460 450 475 Q440 490 425 480 Q412 470 418 455 Q430 440 450 445 Z"
          fill="#E7E9EA"
        />
      </g>
    </svg>
  );
}
