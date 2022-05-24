import subprocess
import os
import argparse

parser = argparse.ArgumentParser(
    description='Create grasps for ply files in objects folder. \
    Grasps will be saved in grasps folder')

parser.add_argument('num_grasps', type=int, nargs='?',
                    help='Number of grasps that shall be created. \
                        Default value is 15')
parser.add_argument('object', type=str, nargs='?',
                    help='Specific object filename for which a grasp shall be created. \
                        No name -> create for all files in objects folder')
args = parser.parse_args()
if args.num_grasps == None:
    num_grasps = 15
else:
    num_grasps = args.num_grasps

dir = os.path.dirname(os.path.abspath(__file__))
# list files in objects directory
files = os.listdir(os.path.join(dir, 'objects'))
ply_files = [x for x in files if x.endswith(".ply")]

if args.object == None:
    for ply in ply_files:
        txt = ply.replace('.ply', '.txt')

        subprocess.run([os.path.join(dir, 'build', 'detect_grasps_hsrb'),
                        os.path.join(dir, 'cfg/hsrb_params.cfg'),
                        os.path.join(dir, 'objects', ply),
                        str(num_grasps),
                        os.path.join(dir, 'grasps', txt)])
else: 
    ply = args.object
    txt = ply.replace('.ply', '.txt')

    subprocess.run([os.path.join(dir, 'build', 'detect_grasps_hsrb'),
                    os.path.join(dir, 'cfg/hsrb_params.cfg'),
                    os.path.join(dir, 'objects', ply),
                    str(num_grasps),
                    os.path.join(dir, 'grasps', txt)])

