/**
 * Флаг режима эмуляции.
 *
 * Активируется только через env var VITE_EMULATION=1 (npm-скрипт dev:emu).
 * В прод-сборке (npm run build) флаг не задаётся → import.meta.env.VITE_EMULATION
 * === undefined → эмуляция недоступна на :8080 робота (намеренно: отладочный режим).
 *
 * Vite инлайнит import.meta.env.VITE_* в build-time; без флага код эмуляции
 * tree-shake'ится (при условном импорте/рендере по isEmulation).
 */
export const isEmulation: boolean =
  import.meta.env.VITE_EMULATION === '1';
