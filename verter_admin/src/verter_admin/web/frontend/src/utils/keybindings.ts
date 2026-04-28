/**
 * Keyboard handling utilities
 */

/**
 * Check if key is a movement key (WASD or arrows)
 */
export function isMovementKey(key: string): boolean {
  return ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(
    key.toLowerCase()
  );
}

/**
 * Check if key is a stop key (Space)
 */
export function isStopKey(key: string): boolean {
  return key === ' ' || key.toLowerCase() === 'space';
}

/**
 * Get movement direction from key
 */
export function getDirectionFromKey(
  key: string
): { linear: number; angular: number } | null {
  const k = key.toLowerCase();

  switch (k) {
    case 'w':
    case 'arrowup':
      return { linear: 1, angular: 0 };
    case 's':
    case 'arrowdown':
      return { linear: -1, angular: 0 };
    case 'a':
    case 'arrowleft':
      return { linear: 0, angular: 1 };
    case 'd':
    case 'arrowright':
      return { linear: 0, angular: -1 };
    default:
      return null;
  }
}

/**
 * Format key name for display
 */
export function formatKeyName(key: string): string {
  const k = key.toLowerCase();

  const keyNames: Record<string, string> = {
    ' ': 'SPACE',
    'arrowup': '↑',
    'arrowdown': '↓',
    'arrowleft': '←',
    'arrowright': '→',
  };

  return keyNames[k] || k.toUpperCase();
}

/**
 * Key binding map for manual control
 */
export const MANUAL_CONTROL_BINDINGS = {
  forward: ['w', 'ArrowUp'],
  backward: ['s', 'ArrowDown'],
  left: ['a', 'ArrowLeft'],
  right: ['d', 'ArrowRight'],
  stop: [' ', 'Escape'],
} as const;
