import pickle
import argparse
from topomap import Topomap
import os

TOPOMAP_IMAGES_DIR = "../topomaps/images"
TOPOMAP_MATRIX = "../topomaps/matrix.pkl"

def main(args: argparse.Namespace):
    with open(TOPOMAP_MATRIX, 'rb') as file:
        loaded_data = pickle.load(file)

    image_folder=os.path.join(TOPOMAP_IMAGES_DIR, args.name)
    adjacency_matrix=loaded_data[args.name]
    
    topomap = Topomap()
    topomap.load(image_folder,adjacency_matrix)
    # print(topomap.get_adjacency_matrix())
    # frontier_node = topomap.find_nearest_frontier_node(7,count=2)
    # sorted_node = sorted(topomap.nodes(), key=lambda n: topomap.nodes[n]['count'])
    # print(frontier_node)
    topomap.visualize()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=f"print adjacency matrix of your chosen topomap")
    parser.add_argument(
        "--name",
        "-n",
        default="topomap",
        type=str,
        help="name of your topomap (default: topomap)",
    )
    args = parser.parse_args()
    main(args)