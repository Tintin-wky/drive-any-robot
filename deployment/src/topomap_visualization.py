import pickle
import argparse
from topomap import Topomap
import os

TOPOMAP_IMAGES_DIR = "../topomaps/images"
TOPOMAPS="../topomaps/topomaps.pkl"

def main(args: argparse.Namespace):
    with open(TOPOMAPS, 'rb') as file:
        topomap = pickle.load(file)[args.name]
    print(topomap.get_adjacency_matrix())
    print(topomap.path)
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