import networkx as nx
import numpy as np
from PIL import Image
import os
import pickle
import shutil
import matplotlib.pyplot as plt
from tqdm import tqdm
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion,Pose

TOPOMAP_IMAGES_DIR = "../topomaps/images"
TOPOMAPS="../topomaps/topomaps.pkl"

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

class Topomap(nx.DiGraph):
    def __init__(self):
        super().__init__()
        self.path = []
        self.last_number_of_nodes = 0
        self.loop_back = True

    def get_adjacency_matrix(self):
        num_nodes = len(self.nodes())
        adjacency_matrix = np.full((num_nodes, num_nodes), np.inf, dtype=float)
        np.fill_diagonal(adjacency_matrix, 0)

        for node,data in self.nodes(data=True):
            adjacency_matrix[node][node] = data['count']

        for node1,node2,data in self.edges(data=True):
            adjacency_matrix[node1][node2] = data['weight']

        return adjacency_matrix
    
    def reset(self):
        for i in range(len(self.nodes())):
            self.nodes[i]['count'] = 0
        self.loop_back = False
        self.last_number_of_nodes = self.number_of_nodes()

    def save(self,name):
        with open(TOPOMAPS, 'wb') as file:
            pickle.dump(dict([(name,self)]), file)
        topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, name)
        if not os.path.isdir(topomap_name_dir):
            os.makedirs(topomap_name_dir)
        else:
            print(f"{topomap_name_dir} already exists. Removing previous images...")
            remove_files_in_dir(topomap_name_dir)
        for node in tqdm(self.nodes(),total=self.number_of_nodes(), desc="Saving"):
            image = self.nodes[node]['image']
            image.save(os.path.join(topomap_name_dir, f"{node}.png"))


    def add_node(self, node_for_adding, **attr):
        count = 1
        return super().add_node(node_for_adding, count=count ,**attr)
    
    def add_edge(self, u_of_edge, v_of_edge, **attr):
        offset = offset_calculate(pose1=self.nodes[u_of_edge]['pose'], pose2=self.nodes[v_of_edge]['pose'])
        return super().add_edge(u_of_edge, v_of_edge, offset=offset,**attr)

    def update_node(self,node,image,pose):
        None

    def loopback(self,node:int,newpose:Pose,num_nodes:int):
        pose = self.nodes[node]['pose']
        offset = offset_calculate(pose1=pose,pose2=newpose)
        for node in range(num_nodes):
            self.nodes[node]['pose'].position.x += offset['dx']
            self.nodes[node]['pose'].position.y += offset['dy']
            yaw = quaternion_to_euler(self.nodes[node]['pose'].orientation)[2]
            yaw += offset['dyaw']
            yaw = (yaw + np.pi) % (2 * np.pi) - np.pi 
            quaternion = euler_to_quaternion([0, 0, yaw])
            self.nodes[node]['pose'].orientation.x = quaternion[0]
            self.nodes[node]['pose'].orientation.y = quaternion[1]
            self.nodes[node]['pose'].orientation.z = quaternion[2]
            self.nodes[node]['pose'].orientation.w = quaternion[3]
        self.loop_back = True

    def neighbors(self, n, area):
        neighbors=set()
        if n is None:
            for node in range(self.number_of_nodes()):
                neighbors.add(node)
            return neighbors
        for node in super().neighbors(n):
            neighbors.add(node)
        x0=self.nodes[n]['pose'].position.x
        y0=self.nodes[n]['pose'].position.y
        for node in range(self.number_of_nodes()):
            x=self.nodes[node]['pose'].position.x
            y=self.nodes[node]['pose'].position.y
            distance = np.linalg.norm(np.array([x0,y0]) - np.array([x,y]))
            if distance < area:
                neighbors.add(node)
        return neighbors
    
    def visualize(self,show_image=True,show_distance=True):
        fig, ax = plt.subplots()
        
        x0=self.nodes[0]['pose'].position.x
        y0=self.nodes[0]['pose'].position.y
        xmin,xmax,ymin,ymax,i=0,0,0,0,0
        pos = {}
        for node,data in self.nodes(data=True):
            x=data['pose'].position.x - x0
            y=data['pose'].position.y - y0
            pos.update({i:np.array([x,y])})
            i += 1
            xmin=x if x<xmin else xmin
            xmax=x if x>xmax else xmax
            ymin=y if y<ymin else ymin
            ymax=y if y>ymax else ymax

        for node, (x, y) in pos.items():
            if(show_image):
                image = self.nodes[node]['image']
                ax.imshow(image, extent=[x-0.8, x+0.8, y-0.6, y+0.6], aspect='auto')
            ax.annotate(node, (x, y-1), fontsize=12, ha="center", va="center",color="r")
        nx.draw_networkx_edges(self, pos)

        if(show_distance):
            labels = {}
            edge_weights = [edge[2]['weight'] for edge in self.edges(data=True)]
            for i, weight in enumerate(edge_weights):
                labels[(f"Node_{i}", f"Node_{i+1}")] = f"{weight:.2f}"
            edge_labels = nx.get_edge_attributes(self, 'weight')
            edge_labels = {(k[0], k[1]): f"{v:.2f}" for k, v in edge_labels.items()}
            nx.draw_networkx_edge_labels(self, pos, edge_labels=edge_labels, font_size=10)

        margin=1
        plt.xlim(xmin-margin,xmax+margin)
        plt.ylim(ymin-margin,ymax+margin)
        plt.axis('off')
        plt.show()