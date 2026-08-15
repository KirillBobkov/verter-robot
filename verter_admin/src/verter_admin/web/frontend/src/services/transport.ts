/**
 * transport — фасад над rosbridge и emulationService.
 *
 * В обычном режиме (VITE_EMULATION не задан) делегирует в rosbridge.
 * В режиме эмуляции (npm run dev:emu, VITE_EMULATION=1) — в emulationService.
 *
 * Существующие хуки (useROS, useDialogROS) импортируют этот модуль вместо
 * rosbridge напрямую → логика хуков не меняется, меняется только транспорт.
 *
 * Реализация: статические импорты обоих модулей + тонкие функции-обёртки,
 * выбирающие реализацию по isEmulation в рантайме. Tree-shaking не вырежет
 * emulationService из прод-бандла (статический импорт), но Vite при build
 * без VITE_EMULATION мёржит; для полного исключения EmulationPanel использует
 * lazy-импорт.
 */

import { isEmulation } from '../config/emulation';
import * as rosbridge from './rosbridge';
import * as emulation from './emulationService';

const impl = isEmulation ? emulation : rosbridge;

export const getRosbridgeUrl = () => impl.getRosbridgeUrl();
export const initROS = (url?: string) => impl.initROS(url);
export const onStatus = (cb: Parameters<typeof rosbridge.onStatus>[0]) =>
  impl.onStatus(cb);
export const closeROS = () => impl.closeROS();
export const subscribeDialogStatus = (
  listener: Parameters<typeof rosbridge.subscribeDialogStatus>[0],
) => impl.subscribeDialogStatus(listener);
export const subscribeAIQuestion = (
  listener: Parameters<typeof rosbridge.subscribeAIQuestion>[0],
) => impl.subscribeAIQuestion(listener);
export const subscribeTextToSpeech = (
  listener: Parameters<typeof rosbridge.subscribeTextToSpeech>[0],
) => impl.subscribeTextToSpeech(listener);
export const publishDialogControl = (command: 'start' | 'stop') =>
  impl.publishDialogControl(command);
