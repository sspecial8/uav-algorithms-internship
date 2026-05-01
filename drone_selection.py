import numpy as np

data = np.array([
    [15,  2.7, 55,  82,  15, 3000],   # DJI Matrice 300 RTK
    [7,  40.0, 22,  54,  12, 5000],   # DJI Agras T40
    [150, 55.0, 600, 130, 20, 8000],  # Bayraktar Mini TB2
    [60,  0.4,  59,  90,  18, 6000],  # WingtraOne GEN II
    [9,   0.8,  40,  72,  10, 2000],  # Autel EVO II
], dtype=float)

weights = np.array([0.25, 0.20, 0.20, 0.15, 0.10, 0.10])  # sum = 1.0
maximize = [True, True, True, True, True, False]

models = [
    "DJI Matrice 300 RTK",
    "DJI Agras T40",
    "Bayraktar Mini TB2",
    "WingtraOne GEN II",
    "Autel EVO II"
]

def wsm_selection(data, weights, maximize):
    norm = np.zeros_like(data)
    for j in range(data.shape[1]):
        if maximize[j]:
            norm[:, j] = data[:, j] / data[:, j].max()
        else:
            norm[:, j] = data[:, j].min() / data[:, j]
    scores = norm @ weights
    return scores

scores = wsm_selection(data, weights, maximize)
ranked = sorted(zip(models, scores), key=lambda x: -x[1])

print("UAV ranking using WSM:")
for rank, (model, score) in enumerate(ranked, 1):
    print(f"{rank}. {model}: {score:.4f}")