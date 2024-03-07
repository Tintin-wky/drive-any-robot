import networkx as nx
import numpy as np
from PIL import Image
import os
import pickle
import math
import shutil
import argparse
import matplotlib.pyplot as plt
from tqdm import tqdm
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion,Pose
from models import gnm
import piexif
from fractions import Fraction

TOPOMAP_IMAGES_DIR = "../topomaps/images"

def remove_files_in_dir(dir_path: str):
    for f in os.listdir(dir_path):
        file_path = os.path.join(dir_path, f)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print("Failed to delete %s. Reason: %s" % (file_path, e)) 

def quaternion_to_euler(orientation:Quaternion):
    return Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_euler('xyz')

def euler_to_quaternion(euler:np.ndarray):
    return Rotation.from_euler('xyz', euler).as_quat()

def offset_calculate(pose1:Pose,pose2:Pose):
    dx = pose2.position.x-pose1.position.x
    dy = pose2.position.y-pose1.position.y
    dyaw = (quaternion_to_euler(pose2.orientation))[2] - (quaternion_to_euler(pose1.orientation))[2]
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi 
    return {'dx':dx, 'dy':dy, 'dyaw':dyaw}

def rotate(point, angle):
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    return rotation_matrix.dot(point)

def get_transform(pose:Pose,newpose:Pose):
    original_point = np.array([pose.position.x,pose.position.y])
    new_point = np.array([newpose.position.x,newpose.position.y])
    original_yaw = quaternion_to_euler(pose.orientation)[2]
    new_yaw = quaternion_to_euler(newpose.orientation)[2]
    angle_diff = new_yaw - original_yaw
    rotated_point = rotate(original_point, angle_diff)
    translation = new_point - rotated_point
    return angle_diff, translation

def transform(pose:Pose,angle_diff,translation):
    original_point = np.array([pose.position.x,pose.position.y])
    rotated_point = rotate(original_point, angle_diff)
    transformed_point = rotated_point + translation
    return transformed_point

class Topomap(nx.DiGraph):
    def __init__(self):
        super().__init__()
        self.path = []
        self.last_node_ID = -1
        self.loop_back = True
        self.name = None

    def get_adjacency_matrix(self):
        last_node = list(self.nodes())[-1] + 1
        adjacency_matrix = np.full((last_node, last_node), np.inf, dtype=float)
        np.fill_diagonal(adjacency_matrix, 0)

        for node,data in self.nodes(data=True):
            adjacency_matrix[node][node] = data['count']

        for node1,node2,data in self.edges(data=True):
            adjacency_matrix[node1][node2] = data['weight']

        return adjacency_matrix
    
    def reset(self):
        for i in self.nodes():
            self.nodes[i]['count'] = 0
        self.loop_back = False

    
    def save_node_images(self):
        def to_dms(value):
            degree = int(value)
            temp_minute = (value - degree) * 60
            minute = int(temp_minute)
            second = (temp_minute - minute) * 60
            return (Fraction(degree, 1), Fraction(minute, 1), Fraction(int(second * 10000), 10000))

        topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, self.name)
        if not os.path.isdir(topomap_name_dir):
            os.makedirs(topomap_name_dir)
        else:
            print(f"{topomap_name_dir} already exists. Removing previous images...")
            remove_files_in_dir(topomap_name_dir)
        for node in tqdm(self.nodes(),total=self.number_of_nodes(), desc="Saving"):
            image = self.nodes[node]['image']
            if self.nodes[node].get('gps') is not None:
                gps_latitude = to_dms(self.nodes[node]['gps'].latitude)
                gps_longitude = to_dms(self.nodes[node]['gps'].longitude)

                # 创建EXIF数据，包含GPS信息
                exif_dict = {
                    "GPS": {
                        piexif.GPSIFD.GPSLatitudeRef: 'N' if self.nodes[node]['gps'].latitude >= 0 else 'S',
                        piexif.GPSIFD.GPSLatitude: tuple(map(lambda x: (x.numerator, x.denominator), gps_latitude)),
                        piexif.GPSIFD.GPSLongitudeRef: 'E' if self.nodes[node]['gps'].longitude >= 0 else 'W',
                        piexif.GPSIFD.GPSLongitude: tuple(map(lambda x: (x.numerator, x.denominator), gps_longitude)),
                    }
                }
                exif_bytes = piexif.dump(exif_dict)
                image.save(os.path.join(topomap_name_dir, f"{node}.jpg"), "JPEG", exif=exif_bytes)
            else:
                image.save(os.path.join(topomap_name_dir, f"{node}.jpg"))

    def save(self,name):
        self.name = name
        self.last_node_ID = list(self.nodes())[-1]
        topomap_path = f"../topomaps/{self.name}.pkl"
        with open(topomap_path, 'wb') as file:
            pickle.dump(self, file)
        self.save_node_images()
        self.visualize()

    def add_node(self, node_for_adding, **attr):
        count = 1
        return super().add_node(node_for_adding, count=count ,**attr)
    
    def add_edge(self, u_of_edge, v_of_edge, **attr):
        offset = offset_calculate(pose1=self.nodes[u_of_edge]['pose'], pose2=self.nodes[v_of_edge]['pose'])
        return super().add_edge(u_of_edge, v_of_edge, offset=offset,**attr)

    def update_node(self,node,image,pose):
        None

    def loopback(self,node:int,newpose:Pose):
        pose = self.nodes[node]['pose']
        angle_diff, translation = get_transform(pose,newpose)
        for node in [n for n in self.nodes() if  n <= self.last_node_ID]:
            self.nodes[node]['pose'].position.x, self.nodes[node]['pose'].position.y = transform(self.nodes[node]['pose'],angle_diff,translation)
            yaw = quaternion_to_euler(self.nodes[node]['pose'].orientation)[2]
            yaw += angle_diff
            yaw = (yaw + np.pi) % (2 * np.pi) - np.pi 
            quaternion = euler_to_quaternion([0, 0, yaw])
            self.nodes[node]['pose'].orientation.x = quaternion[0]
            self.nodes[node]['pose'].orientation.y = quaternion[1]
            self.nodes[node]['pose'].orientation.z = quaternion[2]
            self.nodes[node]['pose'].orientation.w = quaternion[3]
        self.loop_back = True
    
    def merge(self,node_reserved,node_replaced):
        for u, v, data in list(self.in_edges(node_replaced, data=True)):
            if u != node_reserved:
                self.add_edge(u, node_reserved, weight=data['weight'])
        for u, v, data in list(self.out_edges(node_replaced, data=True)):
            if v != node_reserved:
                self.add_edge(node_reserved, v, weight=data['weight'])
        self.remove_node(node_replaced)

    def shortest_path(self,node1,node2):
        try:
            return nx.shortest_path(self, source=node1, target=node2, weight='weight')
        except nx.NetworkXNoPath:
            node_list = [node1]
            node_list.extend(super().neighbors(node1))
            return node_list

    def neighbors(self, n, area):
        neighbors=set()
        for node in super().neighbors(n):
            neighbors.add(node)
        x0=self.nodes[n]['pose'].position.x
        y0=self.nodes[n]['pose'].position.y
        for node in self.nodes():
            if self.loop_back is False and node < self.last_node_ID:
                continue
            x=self.nodes[node]['pose'].position.x
            y=self.nodes[node]['pose'].position.y
            distance = np.linalg.norm(np.array([x0,y0]) - np.array([x,y]))
            if distance < area:
                neighbors.add(node)
            if self.nodes[n]['gps'] is not None and self.nodes[node]['gps'] is not None:
                if abs(self.nodes[n]['gps']['latitude']-self.nodes()[node]['gps'].latitude) < 0.0001 and abs(self.nodes[n]['gps']['longitude']-self.nodes()[node]['gps'].longitude) < 0.0001:
                    neighbors.add(node)
        return neighbors
    
    def visualize(self,show_image=True,show_distance=True, use_gps=True):
        fig, ax = plt.subplots()
        
        nodes=list(self.nodes())

        if(use_gps):
            latitude_scale = 102662
            longitude_scale = 111195
            x0=self.nodes[nodes[0]]['gps'].longitude*longitude_scale
            y0=self.nodes[nodes[0]]['gps'].latitude*latitude_scale
            xmin,xmax,ymin,ymax=0,0,0,0
            pos = {}
            x = x0
            y = y0
            for node,data in self.nodes(data=True):
                x=data['gps'].longitude*longitude_scale - x0 if data['gps'].longitude >= 1e-3 else x
                y=data['gps'].latitude*latitude_scale - y0 if data['gps'].latitude >= 1e-3 else y
                pos.update({node:np.array([x,y])}) 
                xmin=x if x<xmin else xmin
                xmax=x if x>xmax else xmax
                ymin=y if y<ymin else ymin
                ymax=y if y>ymax else ymax
        else:
            x0=self.nodes[nodes[0]]['pose'].position.x
            y0=self.nodes[nodes[0]]['pose'].position.y
            xmin,xmax,ymin,ymax=0,0,0,0
            pos = {}
            for node,data in self.nodes(data=True):
                x=data['pose'].position.x - x0
                y=data['pose'].position.y - y0
                pos.update({node:np.array([x,y])})
                xmin=x if x<xmin else xmin
                xmax=x if x>xmax else xmax
                ymin=y if y<ymin else ymin
                ymax=y if y>ymax else ymax

        for node, (x, y) in pos.items():
            if(show_image):
                image = self.nodes[node]['image']
                ax.imshow(image, extent=[x-2, x+2, y-2, y+2], aspect='auto')
            ax.annotate(node, (x, y-1), fontsize=12, ha="center", va="center",color="r")
        nx.draw_networkx_edges(self, pos,node_size=2000)

        if(show_distance):
            edge_labels = nx.get_edge_attributes(self, 'weight')
            edge_labels = {(k[0], k[1]): f"{v:.2f}" for k, v in edge_labels.items()}
            nx.draw_networkx_edge_labels(self, pos, edge_labels=edge_labels, font_size=10)

        margin=2
        egde = max(xmax-xmin,ymax-ymin)/2
        xmid = (xmin+xmax)/2
        ymid = (ymin+ymax)/2
        plt.xlim(xmid-egde-margin,xmid+egde+margin)
        plt.ylim(ymid-egde-margin,ymid+egde+margin)
        plt.axis('on')
        plt.savefig(os.path.join(TOPOMAP_IMAGES_DIR,f"/{self.name}.jpg"))
        plt.show()

def main(args: argparse.Namespace):
    topomap_path = f"../topomaps/{args.name}.pkl"
    with open(topomap_path, 'rb') as file:
        topomap = pickle.load(file)
    # # print(topomap.get_adjacency_matrix())
    # print(topomap)
    # print(topomap.path)
    print(topomap.nodes()[0]['gps'].latitude)
    print(topomap.nodes()[1]['gps'].latitude)
    print(topomap.nodes()[2]['gps'].latitude)
    # print(topomap.shortest_path(3,12))
    # topomap.visualize(use_gps=False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=f"get info of your chosen topomap")
    parser.add_argument(
        "--name",
        "-n",
        default="topomap",
        type=str,
        help="name of your topomap (default: topomap)",
    )
    args = parser.parse_args()
    main(args)