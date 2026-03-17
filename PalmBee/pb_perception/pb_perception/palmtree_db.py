#!/usr/bin/env python3
import sqlite3
import csv
from typing import Tuple, Optional, List

class PalmTreeDB:
    def __init__(self, db_path: str = "palm_tree.db"):
        self.db_path = db_path
        self.conn = sqlite3.connect(self.db_path)
        self._create_table()

    def _create_table(self):
        with self.conn:
            self.conn.execute("""
            CREATE TABLE IF NOT EXISTS palm_trees (
                tree_id TEXT PRIMARY KEY,
                lat REAL,
                lon REAL,
                alt REAL,
                tag_x REAL,
                tag_y REAL,
                tag_z REAL,
                center_x REAL,
                center_y REAL,
                center_z REAL
            )
            """)

    def add_tree(self, tree_id: str,
                 gps_coord: Tuple[float, float, float],
                 tag_position: Tuple[float, float, float],
                 center_pos: Tuple[float, float, float]):
        with self.conn:
            self.conn.execute("""
            INSERT INTO palm_trees (
                tree_id, lat, lon, alt,
                tag_x, tag_y, tag_z,
                center_x, center_y, center_z
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (tree_id, *gps_coord, *tag_position, *center_pos))

    def update_tree(self, tree_id: str,
                    gps_coord: Optional[Tuple[float, float, float]] = None,
                    tag_position: Optional[Tuple[float, float, float]] = None,
                    center_pos: Optional[Tuple[float, float, float]] = None):
        fields = []
        values = []

        if gps_coord:
            fields += ["lat = ?", "lon = ?", "alt = ?"]
            values += list(gps_coord)
        if tag_position:
            fields += ["tag_x = ?", "tag_y = ?", "tag_z = ?"]
            values += list(tag_position)
        if center_pos:
            fields += ["center_x = ?", "center_y = ?", "center_z = ?"]
            values += list(center_pos)

        values.append(tree_id)

        if fields:
            with self.conn:
                self.conn.execute(f"""
                UPDATE palm_trees
                SET {', '.join(fields)}
                WHERE tree_id = ?
                """, values)

    def get_tree(self, tree_id: str) -> Optional[dict]:
        cursor = self.conn.cursor()
        cursor.execute("SELECT * FROM palm_trees WHERE tree_id = ?", (tree_id,))
        row = cursor.fetchone()
        return self._row_to_dict(row) if row else None

    def get_all_trees(self) -> List[dict]:
        cursor = self.conn.cursor()
        cursor.execute("SELECT * FROM palm_trees")
        rows = cursor.fetchall()
        return [self._row_to_dict(row) for row in rows]

    def delete_tree(self, tree_id: str):
        with self.conn:
            self.conn.execute("DELETE FROM palm_trees WHERE tree_id = ?", (tree_id,))

    def _row_to_dict(self, row):
        keys = ["tree_id", "lat", "lon", "alt", "tag_x", "tag_y", "tag_z", "center_x", "center_y", "center_z"]
        return dict(zip(keys, row))

    def export_to_csv(self, csv_path: str):
        trees = self.get_all_trees()
        if trees:
            keys = trees[0].keys()
            with open(csv_path, mode='w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=keys)
                writer.writeheader()
                writer.writerows(trees)

    def import_from_csv(self, csv_path: str):
        with open(csv_path, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                self.add_tree(
                    row["tree_id"],
                    (float(row["lat"]), float(row["lon"]), float(row["alt"])),
                    (float(row["tag_x"]), float(row["tag_y"]), float(row["tag_z"])),
                    (float(row["center_x"]), float(row["center_y"]), float(row["center_z"]))
                )

    def close(self):
        self.conn.close()

if __name__ == "__main__":
    db = PalmTreeDB("/home/palmbee1/palmbee_ws/src/PalmBee/pb_perception/database/test.db")

    # Add a tree
    db.add_tree("T001", (1.2345, 103.1234, 10.0), (0.5, 0.6, 2.1), (0.0, 0.0, 3.0))

    # Update a tree
    db.update_tree("T001", tag_position=(0.6, 0.7, 2.2))

    # Retrieve a tree
    tree = db.get_tree("T001")
    print(tree['tag_x'])

    # Retrieve all trees
    all_trees = db.get_all_trees()
    print(all_trees)

    # Export database to CSV
    db.export_to_csv("/home/palmbee1/palmbee_ws/src/PalmBee/pb_perception/database/test.csv")

    # Import database from CSV
    # db.import_from_csv("palm_trees.csv")

    # Delete a tree
    db.delete_tree("T001")

    db.close()