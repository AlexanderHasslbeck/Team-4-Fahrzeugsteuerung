import open3d as o3d
import numpy as np

# Pfad zu den PCD-Dateien
pcd_file_path1 = "C:/Users/bened/Desktop/Projekt/pcd_real/1683046256.825212558.pcd"
pcd_file_path2 = "C:/Users/bened/Desktop/Projekt/pcd_simulation/402.472503542.pcd"

# Lade die PCD-Dateien
cloud1 = o3d.io.read_point_cloud(pcd_file_path1)
cloud2 = o3d.io.read_point_cloud(pcd_file_path2)

# Zugriff auf die Punkte
points1 = np.asarray(cloud1.points)
points2 = np.asarray(cloud2.points)

# Erstelle K-D-Bäume für schnelle Nachbarschaftsberechnung
kdtree1 = o3d.geometry.KDTreeFlann(cloud1)
kdtree2 = o3d.geometry.KDTreeFlann(cloud2)

# Liste für die gefilterten Punkte
cloud3 = []
cloud4=[]

# Toleranz für den maximalen Abstand zwischen den Punkten
distance_threshold = 1

# Vergleiche die Punkte aus cloud1 mit cloud2
for point_real in points1:
    flag = 1  # Flag, um zu überprüfen, ob der Punkt in cloud2 einen Nachbarn hat
    [_, idx, _] = kdtree2.search_knn_vector_3d(point_real, 1)  # Finde den nächsten Nachbarn in cloud2
    nearest_point = points2[idx[0]]
    distance = np.linalg.norm(nearest_point - point_real)

    if distance <= distance_threshold:
        flag = 0  # Der Punkt hat einen Nachbarn in cloud2, also wird er nicht in cloud3 aufgenommen

    if flag == 1:
        cloud3.append(point_real)

for point_sim in points2:
    flag = 1  # Flag, um zu überprüfen, ob der Punkt in cloud2 einen Nachbarn hat
    [_, idx, _] = kdtree1.search_knn_vector_3d(point_sim, 1)  # Finde den nächsten Nachbarn in cloud2
    nearest_point = points1[idx[0]]
    distance = np.linalg.norm(nearest_point - point_sim)

    if distance <= distance_threshold:
        flag = 0  # Der Punkt hat einen Nachbarn in cloud2, also wird er nicht in cloud3 aufgenommen

    if flag == 1:
        cloud4.append(point_sim)


# Berechne den Prozentsatz der richtigen Punkte
percentage = (1 - ((len(cloud3)+len(cloud4))/(len(points1)+len(points2))))

# Konvertiere cloud3 zurück in eine open3d Punktwolke
cloud3_o3d = o3d.geometry.PointCloud()
cloud3_o3d.points = o3d.utility.Vector3dVector(cloud3)

cloud4_o3d = o3d.geometry.PointCloud()
cloud4_o3d.points = o3d.utility.Vector3dVector(cloud4)

# Speichern der Punktwolke als PCD-Datei
o3d.io.write_point_cloud("C:/Users/bened/Desktop/Projekt/cloud3.pcd", cloud3_o3d)
o3d.io.write_point_cloud("C:/Users/bened/Desktop/Projekt/cloud4.pcd", cloud4_o3d)

# Ausgabe der Ergebnisse
print("Anzahl der Punkte in cloud1:", len(points1))
print("Anzahl der Punkte in cloud2:", len(points2))
print("Anzahl der Punkte in cloud3:", len(cloud3))
print("Anzahl der Punkte in cloud4:", len(cloud4))
print(f"Es sind {percentage * 100:.2f}% richtig!")
