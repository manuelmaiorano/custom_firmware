from os import listdir
from os.path import isfile, join
import argparse

parser = argparse.ArgumentParser(prog="list_files")
parser.add_argument('dir')   

args = parser.parse_args()

onlyfiles = [f for f in listdir(args.dir) if isfile(join(args.dir, f))]

list(map(lambda file: print(f"SRC_C += {args.dir}/{file}"), filter(lambda name: name.endswith('.c') or name.endswith(".S"), onlyfiles)))