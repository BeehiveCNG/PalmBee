import sqlite3

def read_db3_and_save_to_txt(db3_filename, output_filename):
    conn = sqlite3.connect(db3_filename)
    cursor = conn.cursor()

    query = """
    SELECT timestamp, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w
    FROM messages
    WHERE topic = '/localization_pose'
    """

    cursor.execute(query)
    rows = cursor.fetchall()

    with open(output_filename, 'w') as output_file:
        output_file.write('#timestamp x y z qx qy qz qw\n')
        for row in rows:
            timestamp, x, y, z, qx, qy, qz, qw = row
            output_file.write(f'{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n')

    conn.close()

if __name__ == '__main__':
    db3_filename = '/Users/ep51lon-mac/Universe-local/Workspace/BIRL/Penelitian/Proyek/05_Drone-polinasi/Dataset/rosbag_ 10-03-2025 07_40/rosbag2_2025_03_10-07_37_28_0.db3'
    output_filename = 'rtabmap_slam2.txt'
    read_db3_and_save_to_txt(db3_filename, output_filename)
