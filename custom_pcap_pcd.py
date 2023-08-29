from ouster import client, pcap
import argparse
import os

parser = argparse.ArgumentParser(description='pcap to pcd')
parser.add_argument('pcap',type=str)
parser.add_argument('json',type=str)
parser.add_argument('frame',type=int)
# parser.add_argument('outname',type=str)

args = parser.parse_args()


with open(args.json, 'r') as f:
    metadata = client.SensorInfo(f.read())


source = pcap.Pcap(args.pcap, metadata)


from itertools import islice
try:
    import open3d as o3d  # type: ignore
except ModuleNotFoundError:
    print(
        "This example requires open3d, which may not be available on all "
        "platforms. Try running `pip3 install open3d` first.")
    exit(1)

# precompute xyzlut to save computation in a loop
xyzlut = client.XYZLut(metadata)

# create an iterator of LidarScans from pcap and bound it if num is specified
scans = iter(client.Scans(source))

scans = islice(scans, args.frame)

out_name = args.pcap.split('.')[0].split('/')[-1]

for idx, scan in enumerate(scans):
    if idx==args.frame-1:
        xyz = xyzlut(scan.field(client.ChanField.RANGE))

        pcd = o3d.geometry.PointCloud()  # type: ignore

        pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1,
                                                            3))  # type: ignore

        # pcd_path = os.path.join(pcd_dir, f'{pcd_base}_{idx:06d}.{pcd_ext}')
        # print(f'write frame #{idx} to file: {pcd_path}')
        pcd_path = out_name+'.pcd'
        print(pcd_path)
        o3d.io.write_point_cloud(pcd_path, pcd)  # type: ignore
