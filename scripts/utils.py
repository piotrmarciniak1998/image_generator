from random import uniform


def position_on_item(item):
    x_min, x_max, y_min, y_max, height = item.get_bounding_box_of_normalized_item()
    a = x_max * 0.9
    b = y_max * 0.9
    while True:
        x = uniform(x_min, x_max)
        y = uniform(y_min, y_max)

        # TODO: check for collisions with other objects
        if (x * x) / (a * a) + (y * y) / (b * b) < 1:
            return x, y, height


def get_average_x_y(pos1, pos2):
    x = (pos1[0] + pos2[0]) / 2
    y = (pos1[1] + pos2[1]) / 2
    return x, y
