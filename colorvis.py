import threading

import pyglet
from serial import Serial

rgbs: list[tuple[int, int, int]] = [(-1, -1, -1) for _ in range(8)]
frame = 0


def update_readings():
    with Serial("/dev/ttyUSB0", 115200) as ser:
        while True:
            if not ser.in_waiting:
                continue
            try:
                line = ser.readline().decode("utf-8").rstrip()
            except UnicodeDecodeError:
                continue
            if not line.startswith("rgb"):
                continue
            words = line.split()
            try:
                index = int(words[1])
                rgbs[index] = (int(words[2]), int(words[3]), int(words[4]))
            except ValueError:
                continue


window = pyglet.window.Window(500, 600, "colorvis")
label = pyglet.text.Label("0", x=20, y=20, font_size=20)
squares = pyglet.graphics.Batch()
square_graphics = [
    pyglet.shapes.Rectangle(
        x=150,
        y=100,
        width=100,
        height=100,
        color=(255, 79, 0),
        batch=squares,
    ),
    pyglet.shapes.Rectangle(
        x=250,
        y=100,
        width=100,
        height=100,
        color=(255, 255, 255),
        batch=squares,
    ),
    pyglet.shapes.Rectangle(
        x=100,
        y=400,
        width=100,
        height=100,
        color=(255, 79, 0),
        batch=squares,
    ),
    pyglet.shapes.Rectangle(
        x=200,
        y=400,
        width=100,
        height=100,
        color=(255, 255, 255),
        batch=squares,
    ),
    pyglet.shapes.Rectangle(
        x=300,
        y=400,
        width=100,
        height=100,
        color=(255, 79, 0),
        batch=squares,
    ),
    pyglet.shapes.Rectangle(
        x=100,
        y=300,
        width=100,
        height=100,
        color=(255, 255, 255),
        batch=squares,
    ),
    pyglet.shapes.Rectangle(
        x=200,
        y=300,
        width=100,
        height=100,
        color=(255, 79, 0),
        batch=squares,
    ),
    pyglet.shapes.Rectangle(
        x=300,
        y=300,
        width=100,
        height=100,
        color=(255, 255, 255),
        batch=squares,
    ),
]


@window.event
def on_draw():
    global frame
    window.clear()

    for i, s in enumerate(square_graphics):
        s.color = rgbs[i]
    squares.draw()

    frame += 1
    label.text = f"Frame {frame}"
    label.draw()


def main():
    t = threading.Thread(target=update_readings, daemon=True)
    t.start()
    pyglet.clock.schedule_interval(lambda dt: None, 1 / 60)
    pyglet.app.run()


if __name__ == "__main__":
    main()
