import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Path to your Excel file
excel_path = "/home/nexus/nexus_4wd_mecanum_ws/src/odom_tests.xlsx"
sheets = pd.read_excel(excel_path, sheet_name=None)

def analyze_and_plot(df, title):
    df.columns = [c.strip() for c in df.columns]
    odom_cols = [c for c in df.columns if "Odom X" in c or "Odom Y" in c]
    real_cols = [c for c in df.columns if "Real X" in c or "Real Y" in c]

    if len(odom_cols) < 2 or len(real_cols) < 2:
        print(f"Skipping {title} (missing data)")
        return

    odom = df[odom_cols].dropna().to_numpy()
    real = df[real_cols].dropna().to_numpy()

    cov_odom = np.cov(odom.T)
    cov_real = np.cov(real.T)

    print(f"\n===== {title} =====")
    print("Odom covariance:\n", cov_odom)
    print("Real covariance:\n", cov_real)

    plt.figure(figsize=(6,6))
    plt.scatter(odom[:,0], odom[:,1], color='red', label='Odom')
    plt.scatter(real[:,0], real[:,1], color='blue', label='Real')
    plt.scatter(np.mean(real[:,0]), np.mean(real[:,1]),
                color='black', marker='x', s=100, label='Real Mean')
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title(f"Odom vs Real - {title}")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()

for name, df in sheets.items():
    analyze_and_plot(df, name)
