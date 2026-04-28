/**
 * Page configuration for navigation
 * All pages displayed in Russian
 */

export interface PageConfig {
  id: string;
  path: string;
  label: string;
}

export const ACTIVE_PAGES: PageConfig[] = [
  { id: 'idle', path: '/active/idle', label: 'Ожидание' },
  { id: 'menu', path: '/active/menu', label: 'Меню' },
  { id: 'select', path: '/active/select', label: 'Выбор точки' },
  { id: 'confirm', path: '/active/confirm/:name', label: 'Подтверждение' },
  { id: 'navigation', path: '/active/navigation', label: 'Навигация' },
  { id: 'arrived', path: '/active/arrived', label: 'Прибытие' },
  { id: 'find', path: '/active/find', label: 'Поиск' },
  { id: 'info', path: '/active/info', label: 'Информация' },
  { id: 'ask', path: '/active/ask', label: 'Вопрос' },
  { id: 'about', path: '/active/about', label: 'О роботе' },
];

// Helper function to check if a path matches a page config
// Handles dynamic routes like /active/confirm/:name
export function isPathActive(pagePath: string, currentPath: string): boolean {
  // Convert route params to regex pattern
  const pattern = pagePath.replace(/:[^/]+/g, '[^/]+');
  const regex = new RegExp(`^${pattern}$`);
  return regex.test(currentPath);
}
