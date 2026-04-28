/**
 * PointsPageContent - Content component for PointsPage
 * Reusable by both SetupLayout and ServiceLayout
 */

import { useEffect, useCallback, useState } from 'react';
import { useWaypointStore } from '../../store/waypointStore';
import { MapPlaceholder } from '../../components/common/MapPlaceholder';
import { LocationForm } from '../../components/forms/LocationForm';
import { LocationList } from '../../components/forms/LocationList';
import { Button } from '../../components/common/Button';
import type { Waypoint } from '../../types/ros';
import styles from './PointsPage.module.css';

interface EditingState {
  isEditing: boolean;
  waypoint: Waypoint | null;
}

export function PointsPageContent() {
  const { waypoints, loading, error, fetchWaypoints, addWaypoint, deleteWaypoint, clearError } = useWaypointStore();

  const [editing, setEditing] = useState<EditingState>({ isEditing: false, waypoint: null });
  const [showForm, setShowForm] = useState(false);

  // Load waypoints on mount
  useEffect(() => {
    fetchWaypoints();
  }, [fetchWaypoints]);

  // Handle add new location
  const handleAddNew = useCallback(() => {
    setEditing({ isEditing: false, waypoint: null });
    setShowForm(true);
  }, []);

  // Handle edit location
  const handleEdit = useCallback((waypoint: Waypoint) => {
    setEditing({ isEditing: true, waypoint });
    setShowForm(true);
  }, []);

  // Handle cancel form
  const handleCancelForm = useCallback(() => {
    setShowForm(false);
    setEditing({ isEditing: false, waypoint: null });
  }, []);

  // Handle submit form
  const handleSubmit = useCallback(async (waypoint: Waypoint) => {
    if (editing.isEditing && editing.waypoint) {
      // Delete old and save new (edit = delete + save)
      await deleteWaypoint(editing.waypoint.name);
    }
    await addWaypoint(waypoint);

    // Refresh list and close form
    await fetchWaypoints();
    setShowForm(false);
    setEditing({ isEditing: false, waypoint: null });
  }, [editing.isEditing, editing.waypoint, deleteWaypoint, addWaypoint, fetchWaypoints]);

  // Handle export (stub)
  const handleExport = useCallback(() => {
    // TODO: Implement export functionality
    console.log('Export waypoints - coming soon');
  }, []);

  // Handle import (stub)
  const handleImport = useCallback(() => {
    // TODO: Implement import functionality
    console.log('Import waypoints - coming soon');
  }, []);

  return (
    <div className={styles.content}>
      {/* Map Display */}
      <section className={styles.mapSection}>
        <MapPlaceholder
          width="100%"
          height="250px"
          showRobot
          showGrid
        />
      </section>

      {/* Add New Location Form */}
      {showForm && (
        <section className={styles.section}>
          <h2 className={styles.sectionTitle}>
            {editing.isEditing ? 'Редактирование точки' : 'Добавить новую точку'}
          </h2>
          <LocationForm
            onSubmit={handleSubmit}
            onCancel={handleCancelForm}
            initialData={editing.waypoint || undefined}
            submitLabel={editing.isEditing ? 'Обновить' : 'Сохранить точку'}
          />
        </section>
      )}

      {/* Saved Locations */}
      <section className={styles.section}>
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Сохранённые точки</h2>
          <div className={styles.sectionActions}>
            <Button
              variant="secondary"
              size="sm"
              onClick={handleExport}
            >
              Экспорт
            </Button>
            <Button
              variant="secondary"
              size="sm"
              onClick={handleImport}
            >
              Импорт
            </Button>
          </div>
        </div>

        {error && (
          <div className={styles.error}>
            {error}
            <button onClick={clearError} className={styles.errorClose}>×</button>
          </div>
        )}

        <LocationList
          waypoints={waypoints}
          onEdit={handleEdit}
          onDelete={deleteWaypoint}
          loading={loading}
        />

        {!showForm && (
          <Button
            variant="primary"
            onClick={handleAddNew}
            fullWidth
            className={styles.addButton}
          >
            + Добавить новую точку
          </Button>
        )}
      </section>
    </div>
  );
}
