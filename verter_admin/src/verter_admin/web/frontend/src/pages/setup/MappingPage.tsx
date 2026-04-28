/**
 * MappingPage - Map creation interface (SLAM)
 * Phase 2: Setup Mode
 */

import { SetupLayout } from '../../components/layout/SetupLayout';
import { MappingPageContent } from './MappingPageContent';

export function MappingPage() {
  return (
    <SetupLayout title="Создание карты">
      <MappingPageContent />
    </SetupLayout>
  );
}
