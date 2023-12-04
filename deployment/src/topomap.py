import networkx as nx
import numpy as np
from PIL import Image
import os
import matplotlib.pyplot as plt

class Topomap(nx.DiGraph):
    def __init__(self):
        super().__init__()

    def update(self,node,image,weight):
        self.add_node(node, image=image)
        if node != 0:
            self.add_edge(node-1, node, weight=weight)

    def get_adjacency_matrix(self):
        num_nodes = len(self.nodes())
        adjacency_matrix = np.full((num_nodes, num_nodes), np.inf, dtype=float)
        np.fill_diagonal(adjacency_matrix, 0)

        for node1,node2,data in self.edges(data=True):
            adjacency_matrix[node1][node2] = data['weight']

        return adjacency_matrix
      
    def load_nodes(self,topomap):
        image_files = sorted(os.listdir(topomap))
        for i, image_file in enumerate(image_files):
            self.add_node(i, image=Image.open(os.path.join(topomap, image_file)))

    def add_edges(self,adjacency_matrix):
        num_nodes = len(self.nodes())
        for i in range(num_nodes):
            for j in range(i + 1, num_nodes):
                weight = adjacency_matrix[i][j]
                if not np.isinf(weight):
                    self.add_edge(i, j, weight=weight)
    
    def visualize(self,show_image=True):
        pos = nx.spring_layout(self)
        edge_weights = [edge[2]['weight'] for edge in self.edges(data=True)]

        fig, ax = plt.subplots()
        
        # 绘制节点图像并添加标签
        for node, (x, y) in pos.items():
            if(show_image):
                image = self.nodes[node]['image']
                ax.imshow(image, extent=[x-0.05, x+0.05, y-0.05, y+0.05], aspect='auto')
            ax.annotate(node, (x, y), fontsize=18, ha="center", va="center")

        labels = {}
        for i, weight in enumerate(edge_weights):
            labels[(f"Node_{i}", f"Node_{i+1}")] = f"{weight:.2f}"
        
        edge_labels = nx.get_edge_attributes(self, 'weight')
        edge_labels = {(k[0], k[1]): f"{v:.2f}" for k, v in edge_labels.items()}

        nx.draw_networkx_edges(self, pos, width=edge_weights)
        nx.draw_networkx_edge_labels(self, pos, edge_labels=edge_labels, font_size=10)
        plt.axis('off')
        plt.show()