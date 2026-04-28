/**
 * PointsPage - Points of Interest management interface
 * Phase 2: Setup Mode
 */

import { SetupLayout } from '../../components/layout/SetupLayout';
import { PointsPageContent } from './PointsPageContent';

export function PointsPage() {
  return (
    <SetupLayout title="Точки назначения">
      <PointsPageContent />
    </SetupLayout>
  );
}
