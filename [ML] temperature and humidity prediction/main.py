import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression

plt.style.use("fivethirtyeight")

df = pd.read_csv("history.csv", delimiter=";", parse_dates=["date_time"])
df["minute"] = df["date_time"].dt.hour * 60 + df["date_time"].dt.minute
df["day"] = df["date_time"].dt.day_name()
df["hour"] = df["date_time"].dt.hour
df["month"] = df["date_time"].dt.month
if "id" in df.columns: df.drop(columns=["id"], inplace=True)

days = df["day"].unique()

def scatter(group, col, name):
    g = group.groupby(col, as_index=False).mean()
    plt.figure(figsize=(12,6))
    plt.scatter(g[col], g["temp"])
    plt.savefig(f"{name}.jpg")
    plt.close()

for d in days:
    scatter(df[df["day"] == d], "minute", f"{d[:3].upper()}_MIN")

def poly_plot(group, xcol, title):
    g = group.groupby(xcol, as_index=False).mean()
    x = g[xcol].values.reshape(-1,1)
    y = g["temp"].values
    poly = PolynomialFeatures(4)
    x2 = poly.fit_transform(x)
    model = LinearRegression().fit(x2, y)
    s = np.argsort(x[:,0])
    plt.figure(figsize=(12,6))
    plt.scatter(x,y)
    plt.plot(x[s], model.predict(poly.transform(x))[s])
    plt.title(title)
    plt.show()
    plt.close()

for d in days:
    poly_plot(df[df["day"] == d], "minute", d)

poly_plot(df, "minute", "All Days Minute")
poly_plot(df, "hour", "All Days Hour")

for d in days:
    g = df[df["day"] == d]
    h = g.groupby("minute", as_index=False).mean()
    if "humidity" in h.columns:
        plt.figure(figsize=(12,6))
        plt.scatter(h["minute"], h["humidity"])
        plt.savefig(f"{d[:3].upper()}_HUM.jpg")
        plt.close()