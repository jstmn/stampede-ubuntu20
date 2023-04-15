import csv
import pandas as pd

def save_updated(path_name: str):

    df = pd.read_csv(f"cppflow_paths/{path_name}.csv")

    with open("cppflow__" + path_name, "w") as f:

        for row in df.itertuples():
            row_str = "{};{},{},{};{},{},{},{}\n".format(
                row.time,
                row.x,
                row.y,
                row.z,
                row.qw,
                row.qx,
                row.qy,
                row.qz
            )
            f.write(row_str)

    print(path_name)
    # print(df)
    # exit()





for pname in ("circle", "hello", "rot_yz", "s", "square"):
    save_updated(pname)
