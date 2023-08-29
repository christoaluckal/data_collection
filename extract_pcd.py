import subprocess
import os
import sys
import argparse
import datetime

parser = argparse.ArgumentParser(description='Extract pcd files from a bag file')
# parser.add_argument('--pcap', type=str, help='bag file path')
# parser.add_argument('--json', type=str, help='json file path')
parser.add_argument('--src', type=str, help='src path')
parser.add_argument('--frame', type=int, help='frame number')
args = parser.parse_args()

frame_num = args.frame
src_path = args.src

pcap_list = sorted(x for x in os.listdir(src_path) if x.endswith('.pcap'))
json_list = sorted(x for x in os.listdir(src_path) if x.endswith('.json'))

mydir = os.path.join(os.getcwd(), datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
os.mkdir(mydir)
os.chdir(mydir)

try:
    for i in range(len(pcap_list)):
        pcap = pcap_list[i]
        json = json_list[i]
        fname = pcap.split('.')[0]
        print('Extracting frame {} from {} and {}'.format(frame_num, pcap, json))
        subprocess.call(['python3', '-m', 'ouster.sdk.examples.pcap', os.path.join(src_path, pcap), os.path.join(src_path, json), 'pcap-to-pcd', '--scan-num', str(frame_num)])
        for n in range(frame_num-1):
            if n < 10:
                os.remove(f'pcd_out_00000{n}.pcd')
            else:
                os.remove(f'pcd_out_0000{n}.pcd')
        os.rename(f'pcd_out_0000{frame_num-1}.pcd', f'{fname}.pcd')
except Exception as e:
    print(e)
    # python3 -m ouster.sdk.examples.pcap $SAMPLE_DATA_PCAP_PATH $SAMPLE_DATA_JSON_PATH pcap-to-pcd --scan-num 5

