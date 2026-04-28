/**
 * HomePage - Home position setup interface
 * Phase 2: Setup Mode
 */

import { SetupLayout } from '../../components/layout/SetupLayout';
import { HomePageContent } from './HomePageContent';

export function HomePage() {
  return (
    <SetupLayout title="Домашняя позиция">
      <HomePageContent />
    </SetupLayout>
  );
}
