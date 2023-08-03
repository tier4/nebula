#!env python3

from matplotlib import pyplot as plt
import pandas as pd
import re

def try_eval(s):
    try:
        return eval(s)
    except:
        return None

def parse(fn="out.log", exclude=None):
    if exclude is None:
        exclude = []

    with open(fn, "r") as f:
        lines = f.readlines()[:]

    # get rid of [component-container1] etc.
    lines = [re.sub(r"\[.*?\]\s*", "", line) for line in lines]
    text = "\n".join(lines)
    records = re.findall(r"\{.*?\}", text, re.MULTILINE | re.DOTALL)
    records = [try_eval(record) for record in records]
    records = [record for record in records if record is not None]
    df = pd.DataFrame(records)

    exclude = [col for col in exclude if col in df.columns]
    df = df.drop(columns=exclude)
    return df


def plot(df):
    fig, ax = plt.subplots()
    ref_labels = {}

    cycler = ax._get_lines.prop_cycler

    cur_ax = ax
    for i, col in enumerate(df.columns):
        if i != 0:
            cur_ax = ax.twinx()
        color = next(cycler)["color"]
        line = cur_ax.plot(df[col], color=color)
        cur_ax.fill_between(df.index, df[col], color=color, alpha=0.2)
        ref_labels[f"{col} | {df[col].min()}-{df[col].max()}"] = line[0]
    
    cur_ax.legend(ref_labels.values(), ref_labels.keys())
    plt.show()

def scatter3d(df):
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(df["point.x"], df["point.y"], df["point.z"])
    plt.show()

if __name__ == "__main__":
    df = parse(exclude=["block_id", "n_returns"])
    #df = df[~df["scan count per package"].isnull()]
    plot(df)
