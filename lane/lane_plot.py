import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="Plot lane_windows points from CSV or Excel")
    parser.add_argument(
        "data_path",
        nargs="?",
        default="lane_windows.xlsx",
        help="Path to lane_windows.csv or lane_windows.xlsx (default: lane_windows.xlsx)"
    )
    parser.add_argument(
        "--output", "-o",
        help="Output image file (e.g., plot.png)",
        default=None
    )
    args = parser.parse_args()

    # Determine file path (xlsx/csv)
    data_path = args.data_path
    if not os.path.exists(data_path):
        base, ext = os.path.splitext(data_path)
        if ext.lower() in [".xls", ".xlsx"]:
            alt_csv = f"{base}.csv"
            if os.path.exists(alt_csv):
                print(f"Default file not found. Using '{alt_csv}' instead.")
                data_path = alt_csv
        elif ext.lower() == ".csv":
            alt_xlsx = f"{base}.xlsx"
            if os.path.exists(alt_xlsx):
                print(f"Default file not found. Using '{alt_xlsx}' instead.")
                data_path = alt_xlsx

    if not os.path.exists(data_path):
        raise FileNotFoundError(f"파일을 찾을 수 없습니다: {data_path}")

    # Load data
    ext = os.path.splitext(data_path)[1].lower()
    if ext in [".xls", ".xlsx"]:
        df = pd.read_excel(data_path, engine="openpyxl")
    elif ext == ".csv":
        df = pd.read_csv(data_path)
    else:
        raise ValueError(f"지원하지 않는 파일 형식입니다: {ext}")

    # Normalize columns
    df.columns = df.columns.str.strip().str.lower()

    # Plot
    plt.figure(figsize=(10, 6))

    # If side-based data exists
    if all(col in df.columns for col in ['side', 'x', 'y']):
        df['side'] = df['side'].astype(str).str.strip().str.lower()
        colors = {'l': 'blue', 'r': 'red'}
        for side, color in colors.items():
            side_df = df[df['side'] == side]
            plt.scatter(side_df['x'], side_df['y'], s=10, label=f"Side {side.upper()}", color=color, alpha=0.6)
    # If left_x, right_x, and y_center columns exist
    elif all(col in df.columns for col in ['left_x', 'right_x', 'y_center']):
        plt.scatter(df['left_x'], df['y_center'], s=10, label='Left lane', color='blue', alpha=0.6)
        plt.scatter(df['right_x'], df['y_center'], s=10, label='Right lane', color='red', alpha=0.6)
    else:
        print("Available columns:", df.columns.tolist())
        raise KeyError("필요한 컬럼이 없습니다. 'side','x','y' 또는 'left_x','right_x','y_center' 컬럼을 확인하세요.")

    # Invert y-axis for image coordinates
    plt.gca().invert_yaxis()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Lane Windows Points by Side')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # Output
    if args.output:
        plt.savefig(args.output)
        print(f"Saved plot to {args.output}")
    else:
        plt.show()


if __name__ == '__main__':
    main()
