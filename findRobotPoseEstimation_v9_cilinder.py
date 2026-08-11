# Este script é responsável por estimar a pose do robô Pioneer usando dados de um LiDAR 3D. Ele processa os dados do PointCloud2, filtra os pontos dentro de uma região de interesse (ROI),
#  e calcula a pose do robô com base na OBB (Oriented Bounding Box) do cluster mais significativo encontrado. 
# O cluster mais significativo é aquele que se move na nuvem de pontos, indicando a posição do robô.
# Após a identificação do robô o script envia via tópico ROS2 a pose estimada do robô para que o controlador do robô pioneer
# possa usar essa informação para navegação e controle mais precisos.
#  NOTA: Certifique-se de que os pacotes necessários estejam instalados:
# pip install open3d    
# pip install numpy 
# Passos principais:
# 1. Importação das bibliotecas necessárias.
# 2. Definição da classe RobotPoseEstimation, que é um nó ROS2.
# 3. Configuração do assinante para o tópico de PointCloud2 e do publicador para o tópico de pose do robô.
# 4. Leitura, conversão e exibição dos dados do LiDAR.
# 5. Processamento dos dados para estimar a pose do robô.
# 6. Publicação da pose estimada do robô no tópico ROS2.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped       
import numpy as np
import csv
import os
import open3d as o3d
from rclpy.qos import qos_profile_sensor_data 
import time  
import threading  
import matplotlib.cm as cm

import numpy as np


class Kalman2D:
    def __init__(self, dt=0.1):

        self.dt = dt

        # Estado
        self.x = np.zeros((4,1))

        # Covariância
        self.P = np.eye(4)

        # Modelo de movimento
        self.F = np.array([
            [1,0,dt,0],
            [0,1,0,dt],
            [0,0,1,0],
            [0,0,0,1]
        ])

        # Modelo de medição
        self.H = np.array([
            [1,0,0,0],
            [0,1,0,0]
        ])

        # Ruído do processo
        self.Q = np.array([
            [0.0005,0,0,0],
            [0,0.0005,0,0],
            [0,0,0.08,0],
            [0,0,0,0.08]
        ])

        # Ruído da medição
        self.R = np.array([
            [0.015,0],
            [0,0.015]
        ])

        self.initialized = False
        
    def update(self, mx, my):

        if not self.initialized:

            self.x[0] = mx
            self.x[1] = my

            self.initialized = True

            return mx, my

        #################################
        # Predição
        #################################

        self.x = self.F @ self.x

        self.P = self.F @ self.P @ self.F.T + self.Q

        #################################
        # Correção
        #################################

        z = np.array([[mx],
                      [my]])

        y = z - self.H @ self.x

        S = self.H @ self.P @ self.H.T + self.R

        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y

        I = np.eye(4)

        self.P = (I - K @ self.H) @ self.P

        return float(self.x[0]), float(self.x[1])

class RobotPoseEstimation(Node):
    def __init__(self):
        super().__init__('robot_pose_estimation')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.pointcloud_callback,
            qos_profile_sensor_data)
            
        self.publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)

        # Região de interesse (Espaço onde o robô se move)
        self.z_min, self.z_max = 0.10, 0.95
        self.x_min, self.x_max = -0.10, 3.0
        self.y_min, self.y_max = -2.5, 2.5
        
        # Configurar visualização Open3D
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("Occupancy Grid 3D", width=800, height=600)
        
        self.kalman = Kalman2D(dt=0.50)
        
        # Duas nuvens separadas: Uma para o cenário e outra para o robô destacado
        self.point_cloud = o3d.geometry.PointCloud()       # Cenário (Cinza)
        self.robot_cloud = o3d.geometry.PointCloud()       # Robô (Vermelho)
        self.sensor_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.20,
            origin=(0.0, 0.0, 0.0)
        )
        
        self.geometry_added = False
        self.vis.get_render_option().point_size = 2.5      

        self.lock = threading.Lock()
        self.latest_points = None
        self.prev_cluster_info = []
        self.prev_moving_cluster_center = None
        self.box_bottom_height_from_ground = 0.90  # 50 cm acima do chão
        self.sensor_z_is_inverted = False  # Ajuste para lidar com sensores montados de cabeça para baixo, se necessário
        
        # ----------------------------------------------------------------------------------
        # Robot representation
        # ----------------------------------------------------------------------------------

        self.current_cylinder = None
        self.center_sphere = None

        # Pioneer dimensions
        self.cylinder_radius = 0.19      # meters
        self.cylinder_height = 0.24      # meters

        # Sphere representing the estimated center
        self.sphere_radius = 0.03
        #---------------------------------------------------

        # ----------------------------------------------------------------------------------
        # Create cylinder
        # ----------------------------------------------------------------------------------

        mesh = o3d.geometry.TriangleMesh.create_cylinder(
            radius=self.cylinder_radius,
            height=self.cylinder_height,
            resolution=10
        )

        self.current_cylinder = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
        self.current_cylinder.paint_uniform_color([1.0, 0.0, 0.0])
        

        # ----------------------------------------------------------------------------------
        # Create center sphere
        # ----------------------------------------------------------------------------------

        self.center_sphere = o3d.geometry.TriangleMesh.create_sphere(
            radius=self.sphere_radius,
            resolution=20
        )

        self.center_sphere.compute_vertex_normals()
        self.center_sphere.paint_uniform_color([0,1,0])
        
        # Sistema de acumulação estrita
        self.frame_count = 0
        self.frames_to_accumulate =  8  # Número de frames a acumular antes de processar
        self.point_buffer = []          
        self.new_data_ready = False  

        # -----------------------------------------------------------------
        # Visualização da nuvem (independente da estimação da pose)
        # -----------------------------------------------------------------

        self.visual_cloud = o3d.geometry.PointCloud()

        self.visual_buffer = []
        self.visual_frame_count = 0

        # Número de frames usados SOMENTE na visualização
        self.visual_frames_to_accumulate = 5

        self.latest_visual_points = None
        
        self.last_pose_time = time.time()

    def pointcloud_callback(self, msg):
        xyz = self.convert_pointcloud2_to_numpy(msg)
        
        if xyz.size == 0:
            return

        xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
        if xyz.size == 0:
            return
    
        # --------------------------------------------------------------------------------------------------------
        # 2. --- ALINHAMENTO COM O REFERENCIAL DO ROBÔ MÓVEL (YAW + OFFSET) ---
        # --------------------------------------------------------------------------------------------------------
        # Ajuste este ângulo (em graus) até que o movimento do robô na tela coincida com o eixo da odometria
        theta_z = np.radians(-21)  # Ex: np.radians(90) se o LiDAR estiver virado de lado no robô
        
        R_z = np.array([
            [np.cos(theta_z), -np.sin(theta_z), 0],
            [np.sin(theta_z), np.cos(theta_z), 0],
            [0, 0, 1]
        ])
        xyz = np.dot(xyz, R_z.T)

        #"""
       # --------------------------------------------------------------------------------------------------------
       # 1. TRANSFORMAÇÃO DO REFERENCIAL DO SENSOR PARA O REFERENCIAL DO ROBÔ
       # --------------------------------------------------------------------------------------------------------
        
        T_sensor_robot = np.array([
            [ 0, 0, 1],
            [ -1, 0, 0],
            [ 0, -1, 0]
        ])

        xyz = xyz @ T_sensor_robot.T
        #"""
        
        # Distância física [X, Y, Z] em metros do centro do sensor até o centro de rotação do robô.
        # Se o sensor está 15cm para a frente e 25cm acima do eixo das rodas do robô:
        #robot_offset = np.array([0.08, 0.005, 0.51]) # Offset: dx=-0.067, dy=0.001, dz=-0.519
        #xyz = xyz + robot_offset # Subtrai o offset para alinhar o referencial do sensor com o referencial do robô (ajuste os sinais conforme necessário)
        # --------------------------------------------------------------------------------------------------------

        mask = ((xyz[:, 0] >= self.x_min) & (xyz[:, 0] <= self.x_max) &
                (xyz[:, 1] >= self.y_min) & (xyz[:, 1] <= self.y_max) &
                (xyz[:, 2] >= self.z_min) & (xyz[:, 2] <= self.z_max))
        
        filtered_points = xyz[mask]

        # --------------------------------------------------------------
        # Acumulação somente para visualização
        # --------------------------------------------------------------

        if len(xyz) > 0:
            self.visual_buffer.append(xyz.copy())

        self.visual_frame_count += 1

        if self.visual_frame_count >= self.visual_frames_to_accumulate:

            if self.visual_buffer:
                self.latest_visual_points = np.vstack(self.visual_buffer)

            self.visual_buffer = []
            self.visual_frame_count = 0

        with self.lock:
            if len(filtered_points) > 0:
                self.point_buffer.append(filtered_points)
            
            self.frame_count += 1
            self.get_logger().info(f"Acumulando: {self.frame_count}/{self.frames_to_accumulate} frames.", throttle_duration_sec=0.5)

            if self.frame_count >= self.frames_to_accumulate:
                if self.point_buffer:
                    self.latest_points = np.vstack(self.point_buffer)
                    self.new_data_ready = True
                
                self.point_buffer = []
                self.frame_count = 0

    def convert_pointcloud2_to_numpy(self, msg):
        try:
            data_numpy = np.frombuffer(msg.data, dtype=np.uint8)
            num_points = len(data_numpy) // msg.point_step
            if num_points == 0:
                return np.empty((0, 3), dtype=np.float32)
                
            data_numpy = data_numpy[:num_points * msg.point_step].reshape(num_points, msg.point_step)
            xyz = data_numpy[:, :12].copy().view(dtype=np.float32).reshape(-1, 3)
            return xyz
        except Exception as e:
            self.get_logger().error(f"Erro na conversão: {e}")
            return np.empty((0, 3), dtype=np.float32)

    def update_visualization(self):
        pose_to_publish = None

        with self.lock:
            if self.new_data_ready and self.latest_points is not None:
                pose_to_publish = self.poseEstimation(self.latest_points)
                
                if not self.geometry_added:
                    self.vis.add_geometry(self.visual_cloud)
                    self.vis.add_geometry(self.robot_cloud)
                    self.vis.add_geometry(self.sensor_frame)
                    self.vis.add_geometry(self.current_cylinder)
                    self.vis.add_geometry(self.center_sphere)
                    
                    roi_bbox = o3d.geometry.AxisAlignedBoundingBox(
                        min_bound=(self.x_min, self.y_min, self.z_min),
                        max_bound=(self.x_max, self.y_max, self.z_max)
                    )
                    roi_bbox.color = (0.3, 0.3, 0.3) 
                    self.vis.add_geometry(roi_bbox)
                    
                    self.geometry_added = True
                else:
                    self.vis.update_geometry(self.visual_cloud)
                    self.vis.update_geometry(self.robot_cloud)
                    self.vis.update_geometry(self.current_cylinder)
                    self.vis.update_geometry(self.center_sphere)
                
                self.latest_points = None 
                self.new_data_ready = False

        # Atualiza somente a nuvem de visualização

        if self.latest_visual_points is not None:

            points = self.latest_visual_points

            self.visual_cloud.points = o3d.utility.Vector3dVector(points)

            # Distância de cada ponto ao sensor
            distance = np.linalg.norm(points, axis=1)

            max_distance = 4.0

            distance = np.clip(distance, 0, max_distance)

            distance /= max_distance

            # Colormap estilo RViz
            colors = cm.jet(distance)[:, :3]

            self.visual_cloud.colors = o3d.utility.Vector3dVector(colors)

            self.vis.update_geometry(self.visual_cloud)

            self.latest_visual_points = None

        keep_running = self.vis.poll_events() # comente esta linha se quiser apenas a pose sem visualização
        self.vis.update_renderer() # comente esta linha se quiser apenas a pose sem visualização
        return keep_running, pose_to_publish
        #return pose_to_publish # Se você quiser apenas a pose sem visualização comente a linha acima e descomente esta linha

    def poseEstimation(self, points):
        current = time.time()

        dt = current - self.last_pose_time

        self.last_pose_time = current

        print(f"DT = {dt:.3f}")
        if points is None or points.shape[0] < 3:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.voxel_down_sample(voxel_size=0.07)  
        
        """plane_model, inliers = pcd.segment_plane(distance_threshold=0.03, ransac_n=3, num_iterations=100)
        pcd_without_ground = pcd.select_by_index(inliers, invert=True)
        """ #Removendo a segmentação de plano para evitar a perda de pontos do robô, especialmente se o robô estiver próximo ao chão ou se o sensor estiver inclinado.

        pcd_without_ground = pcd  # Usar a nuvem original sem segmentação de plano para evitar perda de pontos do robô, especialmente se o robô estiver próximo ao chão ou se o sensor estiver inclinado.

        if len(pcd_without_ground.points) < 10:
            return None

        labels = np.array(pcd_without_ground.cluster_dbscan(eps=0.2, min_points=20)) # eps é a distância máxima entre pontos para serem considerados no mesmo cluster, min_points é o número mínimo de pontos para formar um cluster
        if labels.size == 0 or np.all(labels < 0):
            return None

        # Seleciona o cluster que mais se moveu em relação ao frame anterior, ou o maior cluster se for o primeiro frame
        cluster_points = np.asarray(pcd_without_ground.points)
        moving_cluster_points = self.select_moving_cluster(cluster_points, labels)
        
        # verifica se o cluster móvel não está vazio e se tem tamanho suficiente para ser considerado o robô, e se a altura do centro do cluster está dentro dos limites esperados para o robô
        if moving_cluster_points is not None and moving_cluster_points.size > 0:
            cluster_center_z = np.mean(moving_cluster_points[:, 2]) # Calcula a altura média do cluster em relação ao sensor
            cluster_dist_x = np.mean(moving_cluster_points[:, 0]) # Calcula a distância média do cluster em relação ao sensor no eixo X

           # verificar se o centro do cluster está acima de 40 cm do sensor em z e se o cluster está distante o suficiente do sensor em x para evitar falsos positivos de objetos muito próximos ou do chão
            if cluster_center_z < 0.40 and cluster_dist_x < 0.1:  # Ajuste os limites conforme necessário
                self.get_logger().info(f"Cluster descartado por estar fora dos limites de altura: z={cluster_center_z:.2f}m")
                return None
        else:
            self.get_logger().info("Nenhum cluster móvel encontrado ou cluster vazio.")
            return None
        
        #if moving_cluster_points is None or moving_cluster_points.size == 0:
        #    return None

        pcd_without_ground.paint_uniform_color([0.75, 0.75, 0.75])
        #self.point_cloud.points = pcd_without_ground.points
        #elf.point_cloud.colors = pcd_without_ground.colors

        self.robot_cloud.points = o3d.utility.Vector3dVector(moving_cluster_points)
        self.robot_cloud.paint_uniform_color([1.0, 0.0, 0.0])

        cluster_center = np.mean(moving_cluster_points, axis=0)
        #obb = self.robot_cloud.get_oriented_bounding_box()
        #cluster_center = obb.center

        cylinder_center_z = self.box_bottom_height_from_ground + (
            self.cylinder_height / 2.0
        )

        x_filtrado, y_filtrado = self.kalman.update(
            cluster_center[0],
            cluster_center[1]
        )

        center = np.array([
            x_filtrado,
            y_filtrado,
            cylinder_center_z
        ])

        # Atualiza cilindro
        # Open3D cria o cilindro centrado na origem
        desired_center = center.copy()
        desired_center[2] -= self.cylinder_height/2

        current_center = self.current_cylinder.get_center()

        self.current_cylinder.translate(
            desired_center - current_center,
            relative=True
        )

        current_center = self.center_sphere.get_center()
        current_center[2] += 0.12

        self.center_sphere.translate(
            center - current_center,
            relative=True
        )

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(center[0]) - 0.25 # Ajuste o offset conforme necessário  
        pose.pose.position.y = float(center[1]) + 0.05
        pose.pose.position.z = float(center[2]) - 1.02
        pose.pose.orientation.w = 1.0 # Orientação neutra (sem rotação) - pode ser ajustada se necessário

        #pose.pose.position.x -= 0.084
       #pose.pose.position.y += 0.117
        #pose.pose.position.z -= 1.02
        
        
        # Imprime a pose e o centro do OBB para depuração
        self.get_logger().info(f"Pose estimada: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, z={pose.pose.position.z:.2f}")
        self.get_logger().info(f"Centro do OBB: x={center[0]:.2f}, y={center[1]:.2f}, z={center[2]:.2f}")       
        
        # salvar a pose estimada em um arquivo csv para análise posterior
        csv_file = os.path.join(os.path.dirname(__file__), 'estimated_poses_P2.csv')
        file_exists = os.path.isfile(csv_file)
        try:
            with open(csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow(['sec','nanosec','x','y','z','qx','qy','qz','qw'])
                stamp = pose.header.stamp
                writer.writerow([
                    stamp.sec,
                    stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    getattr(pose.pose.orientation, 'x', 0.0),
                    getattr(pose.pose.orientation, 'y', 0.0),
                    getattr(pose.pose.orientation, 'z', 0.0),
                    getattr(pose.pose.orientation, 'w', 0.0)
                ])
        except Exception as e:
            self.get_logger().error(f"Erro ao salvar pose em CSV: {e}")

        return pose

    def select_moving_cluster(self, points, labels):
        valid_labels = np.unique(labels[labels >= 0])
        clusters = []
        for lab in valid_labels:
            mask = labels == lab
            pts = points[mask]
            if pts.size == 0:
                continue
            clusters.append({
                'label': int(lab),
                'centroid': pts.mean(axis=0),
                'points': pts,
                'size': pts.shape[0]
            })

        if not clusters:
            return None

        if not self.prev_cluster_info:
            self.prev_cluster_info = clusters
            return max(clusters, key=lambda c: c['size'])['points']

        best_cluster = None
        best_disp = -1.0
        for cluster in clusters:
            nearest = min(
                self.prev_cluster_info,
                key=lambda prev: np.linalg.norm(prev['centroid'] - cluster['centroid'])
            )
            disp = np.linalg.norm(nearest['centroid'] - cluster['centroid'])
            if disp > best_disp:
                best_disp = disp
                best_cluster = cluster

        self.prev_cluster_info = clusters
        return best_cluster['points'] if best_cluster is not None else max(clusters, key=lambda c: c['size'])['points']

    def destroy_node(self):
        if self.vis is not None:
            self.vis.destroy_window()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseEstimation()
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    try:
        while rclpy.ok():
            keep_running, pose_robot = node.update_visualization() # Se você quiser a pose e a visualização
            # pose_robot = node.update_visualization() # Se você quiser apenas a pose sem visualização comente a linha acima e descomente esta linha
            if not keep_running:
                break
                
            if pose_robot is not None:
                node.publisher.publish(pose_robot)
                
            time.sleep(0.01)  # Pequena pausa para reduzir o uso da CPU e permitir que a visualização seja responsiva
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
