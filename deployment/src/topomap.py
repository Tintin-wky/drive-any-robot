import networkx as nx
import numpy as np
from PIL import Image
import os
import matplotlib.pyplot as plt

class Topomap(nx.DiGraph):
    def __init__(self):
        super().__init__()

    def get_adjacency_matrix(self):
        num_nodes = len(self.nodes())
        adjacency_matrix = np.full((num_nodes, num_nodes), np.inf, dtype=float)
        np.fill_diagonal(adjacency_matrix, 0)

        for node,data in self.nodes(data=True):
            adjacency_matrix[node][node] = data['count']

        for node1,node2,data in self.edges(data=True):
            adjacency_matrix[node1][node2] = data['weight']

        return adjacency_matrix
      
    def load(self,topomap_folder,adjacency_matrix):
        image_files = sorted(os.listdir(topomap_folder))
        for i, image_file in enumerate(image_files):
            self.add_node(i, image=Image.open(os.path.join(topomap_folder, image_file)),count=adjacency_matrix[i][i])
        num_nodes = len(self.nodes())
        for i in range(num_nodes):
            for j in range(num_nodes):
                if i == j:
                    continue
                weight = adjacency_matrix[i][j]
                if not np.isinf(weight):
                    self.add_edge(i, j, weight=weight)

    def find_path_to_nearest_frontier_node(self, start_node, count=3):
        # 使用深度优先搜索（DFS）来遍历图
        visited = set()
        stack = [(start_node, [start_node])]

        while stack:
            node, path = stack.pop()
            if self.nodes[node]['count'] < count:
                return path  # 找到满足条件的路径

            if node not in visited:
                visited.add(node)
                neighbors = list(self.neighbors(node))
                for neighbor in neighbors:
                    stack.append((neighbor, path + [neighbor]))

        return None  # 如果没有找到满足条件的路径

    def find_nearest_frontier_node(self, start_node, count=3):
        # 使用广度优先搜索（BFS）来遍历图
        visited = set()
        queue = [start_node]

        while queue:
            node = queue.pop(0)
            if self.nodes[node]['count'] < count:
                return node

            if node not in visited:
                visited.add(node)
                neighbors = list(self.neighbors(node))
                queue.extend(neighbors)

        return None  # 如果没有找到满足条件的节点
    
    def visualize(self,show_image=True):
        pos = nx.circular_layout(self)
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

        nx.draw_networkx_edges(self, pos)
        nx.draw_networkx_edge_labels(self, pos, edge_labels=edge_labels, font_size=10)
        plt.axis('off')
        plt.show()