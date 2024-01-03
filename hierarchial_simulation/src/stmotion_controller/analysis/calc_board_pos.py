import numpy as np


x_offset = 18 * 0.0078

x_close = (0.60799134 + 0.60791194) / 2
x_far = (0.84902476 + 0.84979215) / 2 + x_offset

board_x = x_close + (x_far - x_close) / 2

board_y1 = -0.13944964 + (0.13944964 + 0.08155208) / 2
board_y2 = -0.18135717 + (0.18135717 + 0.12010248) / 2
board_y = (board_y1 + board_y2) / 2
print(board_x)
print(board_y)
print(board_y1, board_y2)