import os
import os.path as osp
import matplotlib.pyplot as plt
import glob
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('nodes_names', nargs='+', choices=['yolov8_node', 'bot_sort_node'])
    parser.add_argument('--log-time', type=str)
    return parser


def read_logs(nodes_names, log_time=None, folder=osp.join(osp.dirname(__file__), "logs")):
    if isinstance(nodes_names, str):
        nodes_names = [nodes_names]
    nodes_folders = [osp.join(folder, node_name) for node_name in nodes_names]
    all_log_files = [sorted(glob.glob(node_folder + "/*")) for node_folder in nodes_folders]
    assert all(len(node_log_files) % 2 == 0 for node_log_files in all_log_files)
    all_input_log_files = [node_log_files[::2] for node_log_files in all_log_files]
    all_output_log_files = [node_log_files[1::2] for node_log_files in all_log_files]

    input_log_files = [node_input_log_files[-1] for node_input_log_files in all_input_log_files]
    output_log_files = [node_output_log_files[-1] for node_output_log_files in all_output_log_files]

    all_input_stamps = list()
    all_input_delays = list()
    all_output_stamps = list()
    all_output_delays = list()
    for input_log_file, output_log_file in zip(input_log_files, output_log_files):
        with open(input_log_file) as f:
            lines = f.readlines()
            stamps, delays = zip(*[line.split() for line in lines])
            stamps = [int(stamp) / 10 ** 9 for stamp in stamps]
            delays = [int(delay) / 10 ** 9 for delay in delays]
            all_input_stamps.append(stamps)
            all_input_delays.append(delays)
        with open(output_log_file) as f:
            lines = f.readlines()
            stamps, delays = zip(*[line.split() for line in lines])
            stamps = [int(stamp) / 10 ** 9 for stamp in stamps]
            delays = [int(delay) / 10 ** 9 for delay in delays]
            all_output_stamps.append(stamps)
            all_output_delays.append(delays)

    return all_input_stamps, all_input_delays, all_output_stamps, all_output_delays


def plot_delays(nodes_names, all_input_stamps, all_input_delays, all_output_stamps, all_output_delays):
    for input_stamps, input_delays, output_stamps, output_delays in \
            zip(all_input_stamps, all_input_delays, all_output_stamps, all_output_delays):
        plt.plot(input_stamps, input_delays)
        plt.plot(output_stamps, output_delays)


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()

    all_input_stamps, all_input_delays, all_output_stamps, all_output_delays = read_logs(args.nodes_names)
    plot_delays(args.nodes_names, all_input_stamps, all_input_delays, all_output_stamps, all_output_delays)
    plt.show()
