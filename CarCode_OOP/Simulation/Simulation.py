import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ANTAL_TOTAL_PUNKTER = 20
INTERVAL_MS = 1000


def laes_y_vaerdier():
    print(f"Indtast {ANTAL_TOTAL_PUNKTER} y-værdier")

    while True:
        raw = "99 99 99 80 65 65 65 40 40 90 90 88 87 75 77 76 76 99 99 99"
        parts = raw.split()

        if len(parts) != ANTAL_TOTAL_PUNKTER:
            print("Du skal indtaste præcis 20 tal")
            continue

        try:
            return [float(x) for x in parts]
        except ValueError:
            print("Alle værdier skal være tal")


def laes_vinduesstoerrelse():
    while True:
        try:
            n = int(input("Hvor mange punkter skal vises i grafen ad gangen? "))
            if 2 <= n <= ANTAL_TOTAL_PUNKTER:
                return n
            else:
                print("Værdien skal være mellem 2 og 20")
        except:
            print("Indtast et helt tal")


def hent_vindue(data, start, window):
    result = []
    for i in range(window):
        index = (start + i) % len(data)
        result.append(data[index])
    return result


def regression(y_values):
    x = np.arange(len(y_values))
    y = np.array(y_values)

    slope, intercept = np.polyfit(x, y, 1)
    y_fit = slope * x + intercept

    return slope, x, y, y_fit


def main():

    data = laes_y_vaerdier()
    window = laes_vinduesstoerrelse()

    fig, ax = plt.subplots()

    points, = ax.plot([], [], "o", markersize=8)
    line, = ax.plot([], [])
    text = ax.text(0.02, 0.95, "", transform=ax.transAxes)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)

    ymin = min(data)
    ymax = max(data)
    margin = max(10, (ymax - ymin) * 0.15)

    ax.set_ylim(ymin - margin, ymax + margin)
    ax.set_xlim(-0.2, window - 1 + 0.2)

    def update(frame):

        y_window = hent_vindue(data, frame, window)

        slope, x, y, y_fit = regression(y_window)

        slope_modsat = -slope

        points.set_data(x, y)
        line.set_data(x, y_fit)

        text.set_text(f"hældning = {slope_modsat:.4f}")

        return points, line, text

    anim = FuncAnimation(
        fig,
        update,
        interval=INTERVAL_MS,
        blit=False,
        cache_frame_data=False
    )

    plt.show()


if __name__ == "__main__":
    main()