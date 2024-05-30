import pygame
import numpy as np
import sys
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# 定义颜色
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
LIGHT_BLUE = (173, 216, 230)

window_size = (1600, 800)

pygame.init()

# 设置窗口大小
screen = pygame.display.set_mode(window_size, DOUBLEBUF | OPENGL | RESIZABLE)
pygame.display.set_caption('3D Arm Simulation')

# 设置OpenGL参数
glEnable(GL_DEPTH_TEST)
gluPerspective(45, (window_size[0] / window_size[1]), 0.1, 100.0)
glTranslatef(-10, 0.0, -50)  # 调整摄像机位置
glRotatef(25, 2, 1, 0)  # 调整视角


# 定义长方体的顶点
def create_cube(width, height, depth):
    return [
        [-width / 2, -height / 2, -depth / 2],
        [width / 2, -height / 2, -depth / 2],
        [width / 2, height / 2, -depth / 2],
        [-width / 2, height / 2, -depth / 2],
        [-width / 2, -height / 2, depth / 2],
        [width / 2, -height / 2, depth / 2],
        [width / 2, height / 2, depth / 2],
        [-width / 2, height / 2, depth / 2]
    ]

edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]

# 定义长方体的面和颜色
faces = [
    (0, 1, 2, 3), (3, 2, 6, 7), (7, 6, 5, 4),
    (4, 5, 1, 0), (1, 5, 6, 2), (4, 0, 3, 7)
]
colors = [
    (1, 0, 0), (0, 1, 0), (0, 0, 1),
    (1, 1, 0), (1, 0, 1), (0, 1, 1)
]

# 初始化长方体
upper_arm = create_cube(30, 12, 12)
lower_arm = create_cube(30, 10, 10)

# 定义长方体的初始位置
upper_arm_position = np.array([0, 0, 0])
lower_arm_position = np.array([30, 0, 0])


# 定义旋转矩阵
def rotation_matrix(pitch, roll, yaw):
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)]
    ])
    
    Ry = np.array([
        [np.cos(roll), 0, np.sin(roll)],
        [0, 1, 0],
        [-np.sin(roll), 0, np.cos(roll)]
    ])
    
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    return np.dot(Rz, np.dot(Ry, Rx))

# 定义长方体的投影
def project(points, screen_size, fov, viewer_distance):
    projected_points = []
    for point in points:
        x = point[0] * fov / (viewer_distance + point[2]) + screen_size[0] / 2
        y = -point[1] * fov / (viewer_distance + point[2]) + screen_size[1] / 2
        projected_points.append([x, y])
    return projected_points

# 画长方体并上色
def draw_cube(vertices):
    glBegin(GL_QUADS)
    for face, color in zip(faces, colors):
        glColor3fv(color)
        for vertex in face:
            glVertex3fv(vertices[vertex])
    glEnd()

# 绘制背景上的文字
def draw_text(screen, text, position, color=WHITE, font_size=36):
    font = pygame.font.SysFont(None, font_size)
    text_surface = font.render(text, True, color)
    screen.blit(text_surface, position)