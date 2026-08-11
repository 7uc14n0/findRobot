# Este script controla a trajetória de um robô pionner 3DX de um ponto a outro
# usando a odometria e a pose estimada pelo processamento de nuvem de pontos 3D
# de um sensor LiDAR  3D, para garantir que o robô chegue corretamente ao ponto
#de destino com o menor erro possível

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import numpy as np
import time
import csv
from pathlib import Path

# Inicializa o nó de controle do robô e se inscreve nos tópicos necessários
# para receber a odometria do robô e a pose estimada do processamento da nuvem de pontos
# e se inscreve em um tópico para publicar as velocidades angular e linear para o robô
class Controller(Node):
    def __init__(self):
        super().__init__('trajectory_controller')

        self.subscription = self.create_subscription(
            Odometry,
            '/pose',
            self.odom_callback,
            20)

        self.subscription_lidar = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.robot_pose_callback,
            20)

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            20)

        self.cmd_msg = Twist()

        # Odometria
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0

        # Pose estimada pelo LiDAR
        self.x_lidar = None
        self.y_lidar = None

        # Pose fusionada
        self.x_fused = 0.0
        self.y_fused = 0.0

        self.first_odom = False

        # Ganho do filtro complementar
        self.K = 0.05

        # Distância máxima aceitável entre LiDAR e odometria
        self.max_pose_error = 0.40

        # Trajetória desejada
        self.XD = [1.0, 0.0, 1.0, 0.0]
        self.YD = [-1.0, 0.0, 1.0, 0.0]

        self.C = 0
        # Ganhos das velocidades do robô
        self.ku = 0.5
        self.kw = 0.5
        self.ko = 0.1

        # True  = Odometria + LiDAR
        # False = Somente LiDAR
        self.use_fusion = True

        print(
            f"Modo selecionado: "
            f"{'FUSION' if self.use_fusion else 'LIDAR_ONLY'}"
        )

        # Diretório de saída
        # Save inside the same directory where the script is executed
        output_dir = Path.cwd() / "trajectory_logs"

        output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")

        self.csv_path = output_dir / f"trajectory_{timestamp}.csv"

        self.csv_file = open(self.csv_path, "w", newline="")

        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "time",
            "waypoint",
            "fusion_mode",
            "target_x",
            "target_y",
            "odom_x",
            "odom_y",
            "lidar_x",
            "lidar_y",
            "fused_x",
            "fused_y",
            "psi",
            "pose_error",
            "rho",
            "u",
            "w"
        ])

    # recebe a odometria do robô
    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        roll, pitch, yaw = quat2euler([qw, qx, qy, qz])

        self.psi = yaw

        # Inicializa pose fusionada na primeira leitura
        if not self.first_odom:
            self.x_fused = self.x
            self.y_fused = self.y
            self.first_odom = True

        # Atualiza pose fusionada
        self.update_fusion()

    # recebe a pose estimada no processamento da nuvem de pontos do sensor LiDAR 3D
    def robot_pose_callback(self, msg):

        self.x_lidar = msg.pose.position.x
        self.y_lidar = -msg.pose.position.y

        print(
            f"LiDAR pose recebida: "
            f"x={self.x_lidar:.3f}, "
            f"y={self.y_lidar:.3f}"
        )

        self.update_fusion()

    def update_fusion(self):

        if not self.first_odom:
            return

        # MODO SOMENTE LIDAR
        if not self.use_fusion:

            if self.x_lidar is not None:
                self.x_fused = self.x_lidar
                self.y_fused = self.y_lidar

            return

        # MODO FUSÃO
        if self.x_lidar is None or self.y_lidar is None:
            self.x_fused = self.x
            self.y_fused = self.y
            return

        erro = np.hypot(
            self.x_lidar - self.x,
            self.y_lidar - self.y
        )

        if erro < self.max_pose_error:

            self.x_fused += self.K * (
                self.x_lidar - self.x_fused
            )

            self.y_fused += self.K * (
                self.y_lidar - self.y_fused
            )

        else:

            self.x_fused = self.x
            self.y_fused = self.y
            
    def run(self):

        start_time = time.time()
        last_control_time = time.time()

        tmax = 300

        while rclpy.ok() and (time.time() - start_time) < tmax:

            rclpy.spin_once(self)

            if self.C >= len(self.XD):
                break

            if time.time() - last_control_time > 0.01:

                last_control_time = time.time()

                Pd = np.array([
                    self.XD[self.C],
                    self.YD[self.C]
                ])

                Xtil = Pd - np.array([
                    self.x_fused,
                    self.y_fused
                ])

                rho = np.linalg.norm(Xtil)

                if self.x_lidar is not None:

                    pose_error = np.hypot(
                        self.x_lidar - self.x,
                        self.y_lidar - self.y
                    )

                else:

                    pose_error = -1.0

                if rho < 0.1:

                    print(f"Waypoint {self.C} alcançado")

                    self.csv_writer.writerow([
                        round(time.time() - start_time, 3),
                        self.C,
                        "fusion" if self.use_fusion else "lidar_only",
                        Pd[0],
                        Pd[1],
                        round(self.x, 4),
                        round(self.y, 4),
                        round(self.x_lidar, 4) if self.x_lidar is not None else "",
                        round(self.y_lidar, 4) if self.y_lidar is not None else "",
                        round(self.x_fused, 4),
                        round(self.y_fused, 4),
                        round(self.psi, 4),
                        round(pose_error, 4),
                        round(rho, 4),
                        0.0,
                        0.0
                    ])

                    self.csv_file.flush()

                    self.C += 1
                    continue

                theta = np.arctan2(
                    Xtil[1],
                    Xtil[0]
                )

                alfa = theta - self.psi

                alfa = (alfa + np.pi) % (2 * np.pi) - np.pi
                theta = (theta + np.pi) % (2 * np.pi) - np.pi

                u = self.ku * np.cos(alfa) * np.tanh(rho)

                w = (
                    (u * np.sin(alfa) / rho)
                    + self.kw * alfa
                    + self.ko * (theta ** 2) * alfa
                )

                self.cmd_msg.linear.x = float(u)
                self.cmd_msg.angular.z = float(w)

                self.publisher.publish(self.cmd_msg)

                self.csv_writer.writerow([
                    round(time.time() - start_time, 3),

                    self.C,

                    "fusion" if self.use_fusion else "lidar_only",

                    Pd[0],
                    Pd[1],

                    round(self.x, 4),
                    round(self.y, 4),

                    round(self.x_lidar, 4) if self.x_lidar is not None else "",

                    round(self.y_lidar, 4) if self.y_lidar is not None else "",

                    round(self.x_fused, 4),
                    round(self.y_fused, 4),

                    round(self.psi, 4),

                    round(pose_error, 4),

                    round(rho, 4),

                    round(u, 4),
                    round(w, 4)
                ])

                self.csv_file.flush()

                print(
                    f"odom=({self.x:.2f},{self.y:.2f}) "
                    f"lidar=({self.x_lidar},{self.y_lidar}) "
                    f"fused=({self.x_fused:.2f},{self.y_fused:.2f}) "
                    f"psi={self.psi:.2f}"
                )

        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0

        self.publisher.publish(self.cmd_msg)

        self.csv_file.close()

        print(f"CSV salvo em: {self.csv_path}")
        print(
            f"Modo utilizado: "
            f"{'FUSION' if self.use_fusion else 'LIDAR_ONLY'}"
        )

        print("Trajetória finalizada.")


def main(args=None):

    rclpy.init(args=args)

    controller = Controller()

    controller.run()

    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
