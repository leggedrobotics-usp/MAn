import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
import os

def main():
    parser = argparse.ArgumentParser(prog="Compare Simulations Per Joints",
                                     description="Generate Time x [variables] plots to compare")
    parser.add_argument("-f", "--files", action="append")
    parser.add_argument("-l", "--labels", action="append")
    parser.add_argument("-c", "--columns", action="append")
    parser.add_argument("-y", "--ylabels", action="append")
    parser.add_argument("-w", "--time_window", action="append")
    parser.add_argument("-t", "--title")
    args = parser.parse_args()
    njnt = 2 # number of joints
    print(args.files)
    print(args.labels)
    print(args.ylabels)
    print(args.columns)
    print(args.time_window)
    title = 'Time x Interaction Torques' if args.title is None else args.title
    
    # Open first file to get inital columns
    input_tables = list()
    out_tables = list()
    time_table = pd.read_csv(args.files[0])
    if len(args.time_window) == 2:
        time_table = time_table[(time_table['time'] >= float(args.time_window[0])) & (time_table['time'] <= float(args.time_window[1]))]
        time_table = time_table[['time']]
    
    for file in args.files:
        table = pd.read_csv(file)
        input_tables.append(table)
    
    col_labels = list(args.columns)
    labels = list(args.labels) + list(table.columns[(1 + len(args.labels)):])

    for c in col_labels:
        out_tables.append(time_table.copy())

    for (o, c) in enumerate(col_labels):
        for i in range(len(input_tables)):
            in_table = input_tables[i][['time',c]]
            out_table = out_tables[o]
            out_tables[o] = pd.merge(out_table, in_table, left_on='time', right_on='time', how='left')
    
    fig, axs = plt.subplots(len(out_tables))
    fig.suptitle(f'{title}',fontsize=20)
    fig
    
    fig.dpi = 150
    fig.set_size_inches((21/2, 9/2))


    for (ax, table, ylabel) in zip(axs, out_tables, args.ylabels):
        tcolumns = table.columns[1:]
        ys = np.array([table[c].to_list() for c in tcolumns]).T
        x = table['time'].to_list()
        mx = np.min(x)
        Mx = np.max(x)
        ax.plot(x,ys)
        ax.title.set_fontsize(20)
        ax.xaxis.label.set_fontsize(20)
        ax.yaxis.label.set_fontsize(20)
        ax.grid(True)
        ax.set_xlim([mx, Mx])
        ax.set_ylabel(ylabel)
        linspace = np.linspace(mx, Mx, 11).round().astype(np.int32).tolist()
        print(f'linspace:{linspace}')
        ax.set_xticks(linspace)

    plt.legend(labels,fontsize=14)
    fig.supxlabel('Time (s)',fontsize=20)
    fig.supylabel('Interaction Torques (N.m)',fontsize=20)

    figname = f"{os.getcwd().split(os.path.sep)[-1]}"
    print(f'figname:{figname}')
    fig.savefig(f"{figname}.svg", format="svg")
    plt.show()


if __name__ == "__main__":
    main()