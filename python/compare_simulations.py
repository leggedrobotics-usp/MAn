import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse

def main():
    parser = argparse.ArgumentParser(prog="Compare Simulations",
                                     description="Generate Time x [variables] plots to compare")
    parser.add_argument("-f", "--files", action="append")
    parser.add_argument("-l", "--labels", action="append")
    parser.add_argument("-t", "--title")
    args = parser.parse_args()
    print(args.files)
    print(args.labels)
    title = 'Time x Accumulated Interaction Forces' if args.title is None else args.title
    
    # Open first file to get inital columns
    big_table = pd.read_csv(args.files[0])
    column_count = 0
    column_mapper = lambda t : { x: x if x == 'time' else f"{x}_{column_count}" for x in t.columns}
    big_table_rename = column_mapper(big_table)
    big_table = big_table.rename(columns=big_table_rename)
    
    for file in args.files[1:]:
        table = pd.read_csv(file)
        column_count += 1
        table = table.rename(columns=column_mapper(table))
        # big_table = pd.concat([big_table, table["time"]], axis=1)
        big_table = pd.merge(big_table, table, left_on='time', right_on='time', how='left')
        # big_table = big_table.join(table.set_index('time'), on='time')
    
    col_labels = list(big_table.columns[1:])
    labels = list(args.labels) + list(big_table.columns[(1 + len(args.labels)):])
    plt.figure(figsize=(21/2, 9/2), dpi=150)
    plt.plot(big_table['time'].to_list(),np.array([big_table[c].to_list() for c in col_labels]).T)
    plt.legend(labels)
    # plt.title(f'Time x Joint Positions')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Joint Positions (rad)')
    plt.title(f'{title}')
    plt.xlabel('Time (s)')
    plt.ylabel('Accumulated Interaction Forces (N)')
    plt.grid(True)
    plt.savefig("test.svg", format="svg")
    plt.show()


if __name__ == "__main__":
    main()