import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import make_pipeline
from pathlib import Path

# 1) 데이터 읽기
file_path = Path("lane_windows.xlsx")      # 위치 맞게 수정
df = pd.read_excel(file_path)
print(df.head(15))

# 2) long 형식 변환
left  = df[['frame', 'y_center', 'left_x' ]].rename(columns={'y_center':'y', 'left_x':'x'})
left['lane']  = 'left'

right = df[['frame', 'y_center', 'right_x']].rename(columns={'y_center':'y', 'right_x':'x'})
right['lane'] = 'right'

lanes = pd.concat([left, right], ignore_index=True).dropna(subset=['x','y'])
lanes = lanes[lanes['x'] > 0]

# 3) 프레임·차선별 2차 회귀
def fit_quadratic(group):
    X = group[['y']].values
    y = group['x'].values
    if len(group) < 3:
        return pd.Series({'a': np.nan, 'b': np.nan, 'c': np.nan, 'n_pts': len(group)})
    model = make_pipeline(PolynomialFeatures(degree=2, include_bias=False),
                          LinearRegression())
    model.fit(X, y)
    a, b = model.named_steps['linearregression'].coef_
    c = model.named_steps['linearregression'].intercept_
    return pd.Series({'a': a, 'b': b, 'c': c, 'n_pts': len(group)})

coefs = lanes.groupby(['frame', 'lane']).apply(fit_quadratic).reset_index()
coefs.to_csv("lane_quadratic_coeffs.csv", index=False)
print(coefs.head(20))

# 4) 시각화 (예: frame 0)
frame_id = 0
plot_data = lanes[lanes['frame'] == frame_id]

plt.figure(figsize=(6,6))
for lane_id, color in [('left', 'tab:orange'), ('right', 'tab:blue')]:
    pts = plot_data[plot_data['lane'] == lane_id]
    plt.scatter(pts['y'], pts['x'], s=20, label=f'{lane_id} pts', marker='x')
    row = coefs[(coefs['frame'] == frame_id) & (coefs['lane'] == lane_id)]
    if not row.empty and row['n_pts'].values[0] >= 3:
        a, b, c = row[['a','b','c']].values[0]
        y_line = np.linspace(pts['y'].min(), pts['y'].max(), 200)
        x_pred = a*y_line**2 + b*y_line + c
        plt.plot(y_line, x_pred, linewidth=2, label=f'{lane_id} fit')

plt.gca().invert_xaxis(); plt.gca().invert_yaxis()
plt.xlabel('y (pixel)'); plt.ylabel('x (pixel)')
plt.title(f'Quadratic lane fits – Frame {frame_id}')
plt.legend(); plt.tight_layout()
plt.show()